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

#define RTC_UPDATE_DELAY 500000000
#define RTC_POLL_INTERVAL_MIN 4000
#define RTC_POLL_INTERVAL_MAX 6000
#define RTC_SYNC_SECONDS 60

/* Info for each registered platform device */
struct atc_tod_data {
	int irq;
	struct workqueue_struct *workqueue;
	struct work_struct rtc_read_work;
	struct work_struct rtc_write_work;
	struct pps_device *pps;
	struct pps_source_info info;
	struct miscdevice miscdev;
	struct fasync_struct *tick_async_queue;
	struct fasync_struct *onchange_async_queue;
	int tick_sig_num;
	int onchange_sig_num;
	int count;
	int frequency;
	bool frequency_locked;
	bool pps_aligned;
	bool rtc_read_complete;
	int rtc_sync_seconds;
	int old_second;
	ktime_t raw;
	int timesrc;
	unsigned int gpio_pin;
	unsigned int clock_was_set_seq;
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
			// probably should re-sync to RTC?
			break;
		//case ATC_TIMESRC_RTCSQWR:
			//timesrc_str = "RTCSQWR";
			//pl_freq = 50;
			//break;
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
			pr_info( "atc_tod: setting time source to %s\n", timesrc_str);
			dd->timesrc = arg;
		}
		break;
	}
	case ATC_TOD_GET_INPUT_FREQ:
		if ((dd->timesrc == ATC_TIMESRC_LINESYNC)
				|| (dd->timesrc == ATC_TIMESRC_RTCSQWR))
			ret = dd->frequency;
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
				pr_debug("atc_tod_ioctl: tick sig err=%d\n", ret); 
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
				pr_debug("atc_tod_ioctl: onchange sig err=%d\n", ret); 
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
	pr_debug("atc_tod_ioctl: cmd=%x arg=%x ret=%d\n", (int)cmd, (int)arg, ret);
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

static void atc_tod_rtc_read(struct work_struct *work)
{
	struct atc_tod_data *dd = container_of(work, struct atc_tod_data, rtc_read_work);
	struct rtc_device *rtc;
	struct system_time_snapshot snapshot;
	struct timespec ts;
	struct rtc_time tm;
	int old_sec = 0;

	rtc = rtc_class_open("rtc0");
	if(!rtc) {
		pr_err("failed to open read rtc0\n");
		dd->rtc_read_complete = true;
		return;
	}

	/* Look for a RTC second rollover */
	while(1) {
		if(rtc_read_time(rtc, &tm) || rtc_valid_tm(&tm)) {
			pr_err("failed to read rtc time\n");
			rtc_class_close(rtc);
			dd->rtc_read_complete = true;
			return;
		}
		if((tm.tm_sec != old_sec) && (old_sec != 0)) {
			rtc_tm_to_time(&tm, &ts.tv_sec);
			do_settimeofday(&ts);
			pr_info("setting system clock from rtc to "
				"%d-%02d-%02d %02d:%02d:%02d UTC\n",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec);
			break;
		}
		old_sec = tm.tm_sec;
		usleep_range(RTC_POLL_INTERVAL_MIN, RTC_POLL_INTERVAL_MAX);
	}
	rtc_class_close(rtc);
	dd->rtc_read_complete = true;

	/* Initialize the clock was set sequence */
	ktime_get_snapshot(&snapshot);
	dd->clock_was_set_seq = snapshot.clock_was_set_seq;
}

static void atc_tod_rtc_write(struct work_struct *work)
{
	//struct atc_tod_data *dd = container_of(work, struct atc_tod_data, rtc_write_work);
	struct rtc_device *rtc;
	struct system_time_snapshot snapshot;
	struct timespec64 ts_real;
	struct rtc_time tm = {0};
	ktime_t future;
	ktime_t timeout;

	rtc = rtc_class_open("rtc0");
	if(!rtc) {
		pr_err("failed to open write rtc0\n");
		return;
	}
	
	ktime_get_snapshot(&snapshot);
	ts_real = ktime_to_timespec64(snapshot.real);
	ts_real.tv_sec++;
	ts_real.tv_nsec = 1000000000 - RTC_UPDATE_DELAY;
	future = timespec64_to_ktime(ts_real);
	timeout = ktime_sub(future, snapshot.real);
	rtc_time_to_tm(ts_real.tv_sec, &tm);
	__set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout_range(&timeout, 1000, HRTIMER_MODE_REL);

	if (rtc_set_time(rtc, &tm) == 0) {
		pr_debug("setting rtc clock to "
			"%d-%02d-%02d %02d:%02d:%02d UTC\n",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec);
	} else {
		pr_err("rtc_set_time error\n");
	}

	rtc_class_close(rtc);
}

static irqreturn_t atc_tod_irq_handler(int irq, void *data)
{
	struct atc_tod_data *dd = data;
	struct pps_event_time ts;
	struct system_time_snapshot snapshot;
	ktime_t delta_ns;
	int delta_ms;

	if (dd->tick_async_queue != NULL) {
		kill_fasync(&dd->tick_async_queue, SIGIO, POLL_IN);
	}
	
	dd->count++;
	if(dd->count >= dd->frequency * 2) {
		dd->count = 0;
			
		/* Get interrupt timestamp */
		ktime_get_snapshot(&snapshot);

		/* Check for time step */
		if(snapshot.clock_was_set_seq != dd->clock_was_set_seq) {
			dd->clock_was_set_seq = snapshot.clock_was_set_seq;
			dd->pps_aligned = false;
			dd->rtc_sync_seconds = 0;
			ts.ts_real = ktime_to_timespec64(snapshot.real);
			dd->old_second = ts.ts_real.tv_sec;
			if (dd->onchange_async_queue != NULL) {
				kill_fasync(&dd->onchange_async_queue, SIGIO, POLL_IN);
			}
			queue_work(dd->workqueue, &dd->rtc_write_work);
		}

		/* Send PPS assert event if aligned */
		if(dd->pps_aligned && dd->frequency_locked) {
			ts.ts_real = ktime_to_timespec64(snapshot.real);
			pps_event(dd->pps, &ts, PPS_CAPTUREASSERT, NULL);
		}

		/* Compare timestamps if we are measuring frequency */
		if(!dd->frequency_locked) {
			if(dd->raw) {
				delta_ns = ktime_sub(snapshot.raw, dd->raw);
				delta_ms = (int)ktime_to_ms(delta_ns);
				if(delta_ms > 1150 && delta_ms < 1250) {
					dd->frequency = 50;
				}
				dd->frequency_locked = true;
				pr_info( "atc_tod: linesync freq locked %dHz (%d)\n",
					dd->frequency, delta_ms);
			}
			dd->raw = snapshot.raw;
		}

		/* check for RTC write interval */
		if(dd->rtc_sync_seconds >= RTC_SYNC_SECONDS) {
			dd->rtc_sync_seconds = 0;
			queue_work(dd->workqueue, &dd->rtc_write_work);
		}
		dd->rtc_sync_seconds++;
	}

	/* Realign linesync PPS if necessary */
	if(dd->rtc_read_complete && dd->frequency_locked && !dd->pps_aligned) {
		ktime_get_snapshot(&snapshot);
		ts.ts_real = ktime_to_timespec64(snapshot.real);
		if((dd->old_second != 0) && (ts.ts_real.tv_sec != dd->old_second)) {
			pps_event(dd->pps, &ts, PPS_CAPTUREASSERT, NULL);
			dd->pps_aligned = true;
			dd->count = 0;
			pr_info("linesync pps re-aligned with second\n");
		}
		dd->old_second = ts.ts_real.tv_sec;
	}

	return IRQ_HANDLED;
}


static int atc_tod_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct atc_tod_data *data;
	int ret;
	int pps_default_params;

	/* allocate space for device info */
	data = devm_kzalloc(&pdev->dev, sizeof(struct atc_tod_data),
			GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* assign device structure to a global poiner */
	global_dev = data;

	/* get linesync gpio from device tree */
	ret = of_get_gpio(np, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get GPIO from device tree\n");
		return ret;
	}
	data->gpio_pin = ret;

	/* Initialize variables */
	data->frequency = 60;
	data->frequency_locked = false;
	data->old_second = 0;
	data->raw = 0;
	data->count = 0;
	data->tick_async_queue = NULL;
	data->onchange_async_queue = NULL;
	data->tick_sig_num = 0;
	data->onchange_sig_num = 0;
	data->pps_aligned = false;
	data->rtc_read_complete = false;
	data->rtc_sync_seconds = 0;
	data->clock_was_set_seq = 0;

	/* Setup atc-tod specific workqueue */
	data->workqueue = create_singlethread_workqueue("atc-tod");
	INIT_WORK(&data->rtc_read_work, atc_tod_rtc_read);
	INIT_WORK(&data->rtc_write_work, atc_tod_rtc_write);
	queue_work(data->workqueue, &data->rtc_read_work);

	/* setup ioctl handling */
	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.fops = &atc_tod_fops;
	data->miscdev.name = kstrdup(np->name, GFP_KERNEL);
	ret = misc_register(&data->miscdev);

	/* GPIO setup */
	ret = gpio_request(data->gpio_pin, "atc-linesync");
	if (ret) {
		dev_err(&pdev->dev, "failed to request GPIO %u\n",
			data->gpio_pin);
		return ret;
	}

	ret = gpio_direction_input(data->gpio_pin);
	if (ret) {
		dev_err(&pdev->dev, "failed to set pin direction\n");
		return -EINVAL;
	}


	/* IRQ setup */
	ret = gpio_to_irq(data->gpio_pin);
	if (ret <= 0) {
		ret = of_irq_to_resource(np, 0, NULL);
		if (ret <= 0) {
			dev_err(&pdev->dev, "failed to map GPIO to IRQ: %d\n", ret);
			return -EINVAL;
		}
	}
	data->irq = ret;

	/* initialize PPS specific parts of the bookkeeping data structure. */
	data->info.mode = PPS_CAPTUREASSERT | PPS_OFFSETASSERT |
		PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC;
	data->info.owner = THIS_MODULE;
	snprintf(data->info.name, PPS_MAX_NAME_LEN - 1, "%s", pdev->name);

	/* register PPS source */
	pps_default_params = PPS_CAPTUREASSERT | PPS_OFFSETASSERT;
	data->pps = pps_register_source(&data->info, pps_default_params);
	if (data->pps == NULL) {
		dev_err(&pdev->dev, "failed to register IRQ %d as PPS source\n",
			data->irq);
		return -EINVAL;
	}

	/* register IRQ interrupt handler */
	ret = request_irq(data->irq, atc_tod_irq_handler,
			0, "atc-linesync", data);
	if (ret) {
		pps_unregister_source(data->pps);
		misc_deregister(&data->miscdev);
		dev_err(&pdev->dev, "failed to acquire IRQ %d\n", data->irq);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, data);
	dev_info(data->pps->dev, "Registered IRQ %d as PPS source\n",
		 data->irq);


	return 0;
}

static int atc_tod_remove(struct platform_device *pdev)
{
	struct atc_tod_data *data = platform_get_drvdata(pdev);

	destroy_workqueue(data->workqueue);
	free_irq(data->irq, data);
	pps_unregister_source(data->pps);
	gpio_free(data->gpio_pin);
	misc_deregister(&data->miscdev);
	dev_info(&pdev->dev, "removed IRQ %d as PPS source\n", data->irq);
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
