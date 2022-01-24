/*
 * atc_tod.c -- ATC TOD driver with PPS support
 *
 * Copyright (C) 2021 Doug Crawford <doug.crawford@intelight-its.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/completion.h>
#include <linux/pvclock_gtod.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pps_kernel.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/signal.h>
#include <linux/rtc.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/atc.h>

static char *timesrc = "LINESYNC";
module_param(timesrc, charp, 0644);
MODULE_PARM_DESC(timesrc, "ATC Time Source Name");

/* PCF8564 delay before the top of the next second */
#define RTC_UPDATE_DELAY 507874000L

/* tweak of the RTC_UPDATE_DELAY based on measured evidence */
#define RTC_TWEAK_DELAY 14600000L

/* Minimum time between rtc trim requests */
#define RTC_LOCKOUT_SEC 16

/* The tolerated rtc offset before the rtc is trimmed */
#define RTC_TRIM_TOLERANCE_NS 1000000L

struct atc_pps_data {
	int irq;
	int pin;
	struct pps_device *pps;
};

struct atc_tod_data {
	struct atc_pps_data linesync;
	struct atc_pps_data rtc;
	int linesync_count;
	int linesync_aligned;
	int linesync_frequency;
	bool linesync_frequency_locked;
	bool rtc_initialized;
	bool rtc_skipped_first_irq;
	bool rtc_level;
	int rtc_lockout_seconds;
	bool rtc_write_request;
	struct timespec rtc_initial_ts;
	struct workqueue_struct *workqueue;
	struct work_struct rtc_read_work;
	struct work_struct rtc_write_work;
	struct completion rtc_sqwr_ready;
	bool rtc_read_worker_ready;
	struct miscdevice miscdev;
	struct fasync_struct *tick_async_queue;
	struct fasync_struct *onchange_async_queue;
	int tick_sig_num;
	int onchange_sig_num;
	struct timespec raw;
	int timesrc;
	unsigned int clock_step_seq;
	bool clock_step;
	bool detect_clock_step;
	int pwrdn_active_count;
	struct notifier_block clock_step_notifier;
	struct notifier_block atc_pwrdn_notifier;
};

static struct atc_tod_data *global_dev;

static int atc_tod_fasync(int fd, struct file *filp, int on)
{
	struct atc_tod_data *p_data = (struct atc_tod_data *)filp->private_data;

	return fasync_helper(fd, filp, on, &p_data->tick_async_queue);
}

static long atc_tod_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct atc_tod_data *dd = (struct atc_tod_data *)filp->private_data;
	unsigned long size;
	int ret = 0;
	char *timesrc_str = NULL;

	// lock structures
	size = (cmd & IOCSIZE_MASK) >> IOCSIZE_SHIFT;
	if (cmd & IOC_IN) {
		if (!access_ok(VERIFY_READ, (void __user *)arg, size))
			return -EFAULT;
	}
	if (cmd & IOC_OUT) {
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, size))
			return -EFAULT;
	}
	switch (cmd) {
	case ATC_TOD_GET_TIMESRC:
		ret = dd->timesrc;
		break;
	case ATC_TOD_SET_TIMESRC: {
		switch (arg) {
		case ATC_TIMESRC_LINESYNC:
			timesrc_str = "LINESYNC";
			break;
		case ATC_TIMESRC_RTCSQWR:
			timesrc_str = "RTCSQWR";
			break;
		case ATC_TIMESRC_CRYSTAL:
			timesrc_str = "CRYSTAL";
			break;
		case ATC_TIMESRC_EXTERNAL1:
			timesrc_str = "EXTERNAL1";
			break;
		case ATC_TIMESRC_EXTERNAL2:
			timesrc_str = "EXTERNAL2";
			break;
		default:
			ret = -EINVAL;
		}
		if (ret == 0) {
			pr_debug( "atc-tod: setting time source to %s\n", timesrc_str);
			dd->timesrc = arg;
		}
		break;
	}
	case ATC_TOD_GET_INPUT_FREQ:
		if ((dd->timesrc == ATC_TIMESRC_LINESYNC)
				|| (dd->timesrc == ATC_TIMESRC_RTCSQWR))
			ret = dd->linesync_frequency;
		else if (dd->timesrc == ATC_TIMESRC_CRYSTAL)
			ret = HZ;
		else
			ret = 0; // unknown
		break;
	case ATC_TOD_REQUEST_TICK_SIG: {
		unsigned long sig = arg;
		if (!valid_signal(sig) || (dd->timesrc != ATC_TIMESRC_LINESYNC)) {
			ret = -EINVAL;
		} else {
			dd->tick_sig_num = sig;
			f_setown(filp, current->pid, 1);
			filp->f_owner.signum = sig;
			if (fasync_helper(0, filp, 1, &dd->tick_async_queue) < 0) {
				pr_debug("atc-tod: ioctl tick sig err=%d\n", ret); 
				ret = -EINVAL;
			}
		}
		break;
	}
	case ATC_TOD_CANCEL_TICK_SIG:
		if (dd->tick_sig_num)
			dd->tick_sig_num = 0;
		fasync_helper(0, filp, 0, &dd->tick_async_queue);
		break;
	case ATC_TOD_REQUEST_ONCHANGE_SIG: {
		unsigned long sig = arg;
		if (!valid_signal(sig)) {
			ret = -EINVAL;
		} else {
			dd->onchange_sig_num = sig;
			f_setown(filp, current->pid, 1);
			filp->f_owner.signum = sig;
			if (fasync_helper(0, filp, 1, &dd->onchange_async_queue) < 0) {
				pr_debug("atc-tod: ioctl onchange sig err=%d\n", ret); 
				ret = -EINVAL;
			}
		}
		break;
	}
	case ATC_TOD_CANCEL_ONCHANGE_SIG:
		if (dd->onchange_sig_num)
			dd->onchange_sig_num = 0;
		fasync_helper(0, filp, 0, &dd->onchange_async_queue);
		break;
	default:
		ret = -ENOTTY;
	}
	// unlock structures
	pr_debug("atc-tod: ioctl cmd=%x arg=%x ret=%d\n", (int)cmd, (int)arg, ret);
	return ret;
}

static int atc_tod_open(struct inode *inode, struct file *filp)
{
        filp->private_data = global_dev;
        return 0;
}

static int atc_tod_close(struct inode *inode, struct file *filp)
{
	atc_tod_fasync(-1, filp, 0);
	filp->private_data = NULL;
	return 0;
}

static const struct file_operations atc_tod_fops = {
	.owner = THIS_MODULE,
	.open = atc_tod_open,
	.release = atc_tod_close,
	.unlocked_ioctl	= atc_tod_ioctl,
	.fasync = atc_tod_fasync,
};

static int atc_tod_clock_step(struct notifier_block *self, unsigned long action, void *dev)
{
	struct atc_tod_data *dd = container_of(self, struct atc_tod_data, clock_step_notifier);

	/* action is true if the clock was stepped. */
	/* This may be called in an unknown context so, just set variables */
	if(action && dd->detect_clock_step) {
		dd->clock_step = true;
		dd->clock_step_seq++;
		dd->linesync_aligned = false;
		dd->rtc_write_request = true;
	}
	return NOTIFY_OK;
}

static int atc_tod_pwrdn(struct notifier_block *self, unsigned long action, void *dev)
{
	struct atc_tod_data *dd = container_of(self, struct atc_tod_data, atc_pwrdn_notifier);

	/* on powerdown lockout rtc writes for a timeout or until cleared.
	 * There is a bad corner case dring rtc writes where we stop the
	 * rtc write, then start.  If the power cut occurs during this
	 * write the rtc will be wrong.  The action function parameter is
	 * false when powerdown is active.
	 */
	if(action) {
		dd->pwrdn_active_count = 0;
	} else {
		dd->pwrdn_active_count = 3; /* one and half sec lockout */
	}

	return NOTIFY_OK;
}

static void atc_tod_rtc_read(struct work_struct *work)
{
	struct atc_tod_data *dd = container_of(work, struct atc_tod_data, rtc_read_work);
	struct rtc_device *rtc;
	struct rtc_time tm;

	rtc = rtc_class_open("rtc0");
	if(!rtc) {
		pr_err("atc-tod: failed to open rtc0\n");
		dd->rtc_initialized = true;
		return;
	}

	/* Let rtc IRQ know that the read worker is ready and then wait for
	 * the next rtc interrupt
	 */
	dd->rtc_read_worker_ready = true;
	wait_for_completion(&dd->rtc_sqwr_ready);

	if(rtc_read_time(rtc, &tm) || rtc_valid_tm(&tm)) {
		pr_err("atc-tod: failed to read rtc time\n");
		rtc_class_close(rtc);
		dd->rtc_initialized = true;
		return;
	}

	// prepare rtc initial_ts to be set on the next rtc half-second interrupt
	rtc_tm_to_time(&tm, &dd->rtc_initial_ts.tv_sec);
	rtc_class_close(rtc);

	/* If the rtc square wave is high then this work was triggered on the
	 * top of the second in which case the next IRQ will hit at the half
	 * second.  If rtc_level is low the next IRQ will hit at the next full
	 * second.
	 */ 
	pr_info("atc-tod: rtc read sqwr: %d\n", dd->rtc_level);
	if(dd->rtc_level) {
		dd->rtc_initial_ts.tv_nsec = 500000000L;
	} else {
		dd->rtc_initial_ts.tv_sec++;
	}
}

static void atc_tod_rtc_write(struct work_struct *work)
{
	struct atc_tod_data *dd = container_of(work, struct atc_tod_data, rtc_write_work);
	struct rtc_device *rtc;
	struct timespec ts_real;
	struct rtc_time tm = {0};
	ktime_t now;
	ktime_t timeout;
	unsigned int clock_step_seq;

	rtc = rtc_class_open("rtc0");
	if(!rtc) {
		pr_err("atc-tod: failed to open write rtc0\n");
		return;
	}

	getnstimeofday(&ts_real);
	now = timespec_to_ktime(ts_real);
	ts_real.tv_sec++;
	ts_real.tv_nsec = 1000000000L - RTC_UPDATE_DELAY + RTC_TWEAK_DELAY;
	rtc_time_to_tm(ts_real.tv_sec, &tm);
	timeout = ktime_sub(timespec_to_ktime(ts_real), now);
	clock_step_seq = dd->clock_step_seq;
	usleep_range(ktime_to_us(timeout) - 100, ktime_to_us(timeout) + 100);

	if(clock_step_seq != dd->clock_step_seq) {
		pr_err("atc-tod: clock step during rtc write\n");
		dd->rtc_write_request = true;
		rtc_class_close(rtc);
		return;
	}

	if(dd->pwrdn_active_count) {
		pr_err("atc-tod: pwrdn active during rtc write\n");
		dd->rtc_write_request = true;
		rtc_class_close(rtc);
		return;
	}

	if (rtc_set_time(rtc, &tm) != 0) {
		pr_err("atc-tod: rtc_set_time error\n");
		rtc_class_close(rtc);
		return;
	}

	pr_debug("atc-tod: setting rtc clock to "
		"%d-%02d-%02d %02d:%02d:%02d UTC\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);

	rtc_class_close(rtc);
}

static irqreturn_t atc_tod_linesync_irq_handler(int irq, void *data)
{
	struct atc_tod_data *dd = data;
	struct pps_event_time ts;
	struct timespec ts_real;
	struct timespec ts_raw;
	ktime_t delta_ns;
	long tolerance_ns;
	int delta_ms;
	int level;

	/* Get the real timestamp */
	getnstimeofday(&ts_real);

	/* send user space linesync tick signal */
	if (dd->tick_async_queue != NULL) {
		kill_fasync(&dd->tick_async_queue, SIGIO, POLL_IN);
	}

	level = gpio_get_value(dd->linesync.pin);
	if(!level) {
		/* count linesync cycles on falling edge only */
		dd->linesync_count++;
	}

	/* Realign linesync PPS if necessary */
	if(!level && dd->rtc_initialized && dd->linesync_frequency_locked && !dd->linesync_aligned) {
		tolerance_ns = (500000000L / dd->linesync_frequency) + 500000L;

		if((ts_real.tv_nsec < tolerance_ns) || (ts_real.tv_nsec > (1000000000L - tolerance_ns))) {
			dd->linesync_aligned = true;
			dd->linesync_count = dd->linesync_frequency;
			pr_debug("atc-tod: linesync pps aligned\n");
		}
	}

	if(!level && (dd->linesync_count >= dd->linesync_frequency)) {
		dd->linesync_count = 0;

		/* Get monotonic timestamp for measuring linesync frequency */
		getrawmonotonic(&ts_raw);

		/* Send PPS assert event if aligned */
		if(dd->linesync_aligned) {
			ts.ts_real = ts_real;
			pps_event(dd->linesync.pps, &ts, PPS_CAPTUREASSERT, NULL);
		}

		/* Compare timestamps if we are measuring frequency */
		if(dd->rtc_initialized && !dd->linesync_frequency_locked) {
			if(dd->raw.tv_sec > 0) {
				delta_ns = ktime_sub(timespec_to_ktime(ts_raw), timespec_to_ktime(dd->raw));
				delta_ms = (int)ktime_to_ms(delta_ns);
				if(delta_ms > 1150 && delta_ms < 1250) {
					dd->linesync_frequency = 50;
				}
				dd->linesync_frequency_locked = true;
				pr_info( "atc-tod: linesync frequency locked %dHz (%d)\n",
					dd->linesync_frequency, delta_ms);
			}
			dd->raw = ts_raw;
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t atc_tod_rtc_irq_handler(int irq, void *data)
{
	struct atc_tod_data *dd = data;
	struct pps_event_time ts;
	struct timespec ts_real;

	getnstimeofday(&ts_real);

	/* The first rtc sqwr IRQ edge is sometimes invalid */
	if(!dd->rtc_skipped_first_irq) {
		dd->rtc_skipped_first_irq = true;
		return IRQ_HANDLED;
	}

	if(dd->clock_step) {
		pr_debug("atc-tod: clock step\n");
		dd->clock_step = false;
		if (dd->onchange_async_queue != NULL) {
			kill_fasync(&dd->onchange_async_queue, SIGIO, POLL_IN);
		}
	}

	dd->rtc_level = gpio_get_value(dd->rtc.pin);
	if(dd->rtc_level) {
		ts.ts_real = ts_real;
		pps_event(dd->rtc.pps, &ts, PPS_CAPTUREASSERT, NULL);

		/* Ignore all clock steps during the first rtc lockout period */
		if(dd->rtc_lockout_seconds < RTC_LOCKOUT_SEC) { 
			dd->rtc_lockout_seconds++;
		} else {
			dd->detect_clock_step = true;
		}

		/* set the rtc only on clock step or when the square wave is
		 * one tick out of alignment.  When using the rtc square wave
		 * as a pps source this technique is important to minimize the
		 * number of times we stop/start/change the rtc.
		 */
		if(dd->rtc_initialized &&
			!work_pending(&dd->rtc_write_work) &&
			dd->rtc_lockout_seconds == RTC_LOCKOUT_SEC &&
			(ts_real.tv_nsec > RTC_TRIM_TOLERANCE_NS) && 
			(ts_real.tv_nsec < (1000000000L - RTC_TRIM_TOLERANCE_NS))) {
				dd->rtc_write_request = true;
				pr_debug("atc-tod: rtc trim %ld\n", ts.ts_real.tv_nsec);
		}
	} else {
		if(dd->rtc_write_request) {
			if(queue_work(dd->workqueue, &dd->rtc_write_work)) {
				dd->rtc_lockout_seconds = 0;
				dd->rtc_write_request = false;
			}
		}
	}

	/* Run the rtc read work on the first second or half second rtc edge
	 * The rtc read work item will look at the rtc_level and configure 
	 * rtc_initial_ts to the appropriate value for the next half sec IRQ
	 */
	if(!dd->rtc_initialized) {
		if(dd->rtc_initial_ts.tv_sec > 0) {
			do_settimeofday(&dd->rtc_initial_ts);
			dd->rtc_initialized = true;
			pr_info("atc-tod: rtc read settimeofday\n");
		} else if(dd->rtc_read_worker_ready) {
			complete(&dd->rtc_sqwr_ready);
		}
	}

	/* Automatically clear pwrdn active after a short timeout */
	if(dd->pwrdn_active_count > 0) {
		dd->pwrdn_active_count--;
	}

	return IRQ_HANDLED;
}

static int atc_tod_setup_pps(struct platform_device *pdev, int pin_index, struct atc_pps_data* data, const char* label) {
	struct device_node *np = pdev->dev.of_node;
	struct pps_source_info pps_info;
	int ret;

	/* device tree setup */
	ret = of_get_gpio(np, pin_index);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get GPIO from device tree\n");
		return ret;
	}
	data->pin = ret;

	/* GPIO pinmux */
	ret = gpio_request(data->pin, label);
	if (ret) {
		dev_err(&pdev->dev, "failed to request GPIO %u\n", data->pin);
		return ret;
	}
	ret = gpio_direction_input(data->pin);
	if (ret) {
		dev_err(&pdev->dev, "failed to set pin direction\n");
		return -EINVAL;
	}

	/* IRQ setup */
	ret = gpio_to_irq(data->pin);
	if (ret <= 0) {
		ret = of_irq_to_resource(np, pin_index, NULL);
		if (ret <= 0) {
			dev_err(&pdev->dev, "failed to map GPIO to IRQ: %d\n", ret);
			return -EINVAL;
		}
	}
	data->irq = ret;

	/* initialize PPS specific parts of the bookkeeping data structure. */
	memset(&pps_info, 0, sizeof(pps_info));
	pps_info.mode = PPS_CAPTUREASSERT | PPS_OFFSETASSERT |
		PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC;
	pps_info.owner = THIS_MODULE;

	/* register PPS source */
	snprintf(pps_info.name, PPS_MAX_NAME_LEN - 1, label);
	data->pps = pps_register_source(&pps_info, PPS_CAPTUREASSERT | PPS_OFFSETASSERT);
	if (data->pps == NULL) {
		dev_err(&pdev->dev, "failed to register PPS source\n");
		return -EINVAL;
	}

	return 0;
}

static int atc_tod_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct atc_tod_data *dd;
	int ret;

	/* allocate space and intialize device info */
	dd = devm_kzalloc(&pdev->dev, sizeof(struct atc_tod_data), GFP_KERNEL);
	if (!dd) {
		return -ENOMEM;
	}

	/* assign device structure to a global poiner */
	global_dev = dd;

	/* Initialize non-zero parameters */
	dd->linesync_frequency = 60;

	/* Setup linesync irq and pps */
	ret = atc_tod_setup_pps(pdev, 0, &dd->linesync, "atc-linesync");
	if(ret) {
		return ret;
	}

	/* Setup rtc irq and pps */
	ret = atc_tod_setup_pps(pdev, 1, &dd->rtc, "atc-rtc");
	if(ret) {
		return ret;
	}

	/* Setup rtc workqueue and start the rtc_read_work */
	init_completion(&dd->rtc_sqwr_ready);
	dd->workqueue = alloc_workqueue("atc-tod", WQ_HIGHPRI, 0);
	INIT_WORK(&dd->rtc_read_work, atc_tod_rtc_read);
	INIT_WORK(&dd->rtc_write_work, atc_tod_rtc_write);
	queue_work(dd->workqueue, &dd->rtc_read_work);

	/* Setup ioctl handling */
	dd->miscdev.minor = MISC_DYNAMIC_MINOR;
	dd->miscdev.fops = &atc_tod_fops;
	dd->miscdev.name = kstrdup(np->name, GFP_KERNEL);
	misc_register(&dd->miscdev);

	/* Enable linesync interrupt handler on falling edge only */
	ret = request_irq(dd->linesync.irq, atc_tod_linesync_irq_handler, 0, "atc-linesync", dd);
	if (ret) {
		dev_err(&pdev->dev, "failed to acquire IRQ %d\n", dd->linesync.irq);
		return -EINVAL;
	}

	/* Enable rtc interrupt handler on both edges */
	ret = request_irq(dd->rtc.irq, atc_tod_rtc_irq_handler, 0, "atc-rtc", dd);
	if (ret) {
		dev_err(&pdev->dev, "failed to acquire IRQ %d\n", dd->rtc.irq);
		return -EINVAL;
	}

	/* Register clock step notifier */
	dd->clock_step_notifier.notifier_call = atc_tod_clock_step;
	ret = pvclock_gtod_register_notifier(&dd->clock_step_notifier);
	if(ret) {
		dev_err(&pdev->dev, "failed to register clock step notifier\n");
		return -EINVAL;
	}

	/* Register power down notifier */
	dd->atc_pwrdn_notifier.notifier_call = atc_tod_pwrdn;
	ret = atc_pwrdn_register_notifier(&dd->atc_pwrdn_notifier);
	if(ret) {
		dev_err(&pdev->dev, "failed to register pwrdn notifier\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, dd);
	return 0;
}

static int atc_tod_remove_pps(struct platform_device *pdev, struct atc_pps_data* data) {
	free_irq(data->irq, global_dev);
	pps_unregister_source(data->pps);
	gpio_free(data->pin);
	return 0;
}

static int atc_tod_remove(struct platform_device *pdev)
{
	struct atc_tod_data *dd = platform_get_drvdata(pdev);

	atc_pwrdn_unregister_notifier(&dd->atc_pwrdn_notifier);
	pvclock_gtod_unregister_notifier(&dd->clock_step_notifier);
	misc_deregister(&dd->miscdev);
	destroy_workqueue(dd->workqueue);
	atc_tod_remove_pps(pdev, &dd->linesync);
	atc_tod_remove_pps(pdev, &dd->rtc);
	return 0;
}

static const struct of_device_id atc_tod_dt_ids[] = {
	{ .compatible = "linux,atc-tod", },
	{ }
};
MODULE_DEVICE_TABLE(of, atc_tod_dt_ids);

static struct platform_driver atc_tod_driver = {
	.probe		= atc_tod_probe,
	.remove		= atc_tod_remove,
	.driver		= {
		.name	= "atc_tod",
		.owner	= THIS_MODULE,
		.of_match_table	= atc_tod_dt_ids,
	},
};

static int __init atc_tod_init(void)
{
	return platform_driver_register(&atc_tod_driver);
}

static void __exit atc_tod_exit(void)
{
	platform_driver_unregister(&atc_tod_driver);
}

late_initcall(atc_tod_init); /* allow rtc driver init first */
module_exit(atc_tod_exit);

MODULE_DESCRIPTION("ATC platform time-of-day handler");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
