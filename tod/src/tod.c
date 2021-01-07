/*
 * Copyright 2014, 2015 AASHTO/ITE/NEMA.
 * American Association of State Highway and Transportation Officials,
 * Institute of Transportation Engineers and
 * National Electrical Manufacturers Association.
 *  
 * This file is part of the Advanced Transportation Controller (ATC)
 * Application Programming Interface Reference Implementation (APIRI).
 *  
 * The APIRI is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version.
 *  
 * The APIRI is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *  
 * You should have received a copy of the GNU Lesser General Public
 * License along with the APIRI.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <limits.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <stdbool.h>
#include <atc.h>
#include "tod.h"
#include "tzfile.h"

#ifndef __isdigit_char
#define __isdigit_char(c)   (((unsigned char)((c) - '0')) <= 9)  
#endif
#ifndef __isleap
#define __isleap(y) ( !((y) % 4) && ( ((y) % 100) || !((y) %400) ) )
#endif

static int g_is_tzif_cached = 0;
static long g_timezone;
static int g_daylight;
static dst_info_t g_dst_info;

static int read_dst(dst_info_t*);

static void read_tzif() {
	if(g_is_tzif_cached == 0) {
		tzset();
		g_timezone = timezone;
		g_daylight = daylight;
		if(daylight) {
			read_dst(&g_dst_info);
		}
// glibc/uclibc will modify timezone/daylight based on current /etc/localtime
// musl will only modify timezone/daylight once at program start
#if !defined(__GNU_LIBRARY__) && !defined(__UCLIBC__)
		g_is_tzif_cached = 1;
#endif
	}
}

char *tod_apiver(void)
{
	return "APIRI, 1.1, 2.17";
}

int tod_ioctl_call(int command, int arg)
{
	int ret = 0;
	int tod_fd;
	
	/* open tod device */
	if ((ret = open("/dev/tod", O_RDONLY)) < 0)
		return ret;
	tod_fd = ret;
	ret = ioctl(tod_fd, command, &arg);
	close (tod_fd);
	
	return ret;
	
}

int tod_get_timesrc(void)
{
	int arg = 0;
	
	return tod_ioctl_call(ATC_TOD_GET_TIMESRC, arg);
}

int tod_get_timesrc_freq(void)
{
	int arg = 0;
	
	return tod_ioctl_call(ATC_TOD_GET_INPUT_FREQ, arg);
}

int tod_set_timesrc(int timesrc)
{
	int ret = 0;
	int tod_fd;
	if ((ret = open("/dev/tod", O_RDWR)) < 0)
		return ret;
	tod_fd = ret;
	ret = ioctl(tod_fd, ATC_TOD_SET_TIMESRC, timesrc);
	close (tod_fd);
	return ret;
}

int tod_request_tick_signal(int signal)
{
	int tod_fd;
	int ret;
	
	if ((ret = open("/dev/tod", O_RDWR)) < 0)
		return ret;
	tod_fd = ret;
	ret = ioctl(tod_fd, ATC_TOD_REQUEST_TICK_SIG, signal);
	if (ret < 0) {
		close(tod_fd);
		return ret;
	}
	
	return tod_fd;
}

int tod_cancel_tick_signal(int fd)
{
	int ret;
	
	ret = ioctl(fd, ATC_TOD_CANCEL_TICK_SIG, 0);
	if (ret < 0)
		return ret;
		
	close(fd);
	return 0;
}

int tod_request_onchange_signal(int signal)
{
	int tod_fd;
	int ret;
	
	if ((ret = open("/dev/tod", O_RDWR)) < 0)
		return ret;
	tod_fd = ret;
	ret = ioctl(tod_fd, ATC_TOD_REQUEST_ONCHANGE_SIG, signal);
	if (ret < 0) {
		close(tod_fd);
		return ret;
	}
		
	return tod_fd;
}

int tod_cancel_onchange_signal(int fd)
{
	int ret = 0;
	
	ret = ioctl(fd, ATC_TOD_CANCEL_ONCHANGE_SIG, 0);
	if (ret < 0)
		return ret;
		
	close(fd);
	return 0;
}


// read the TZif2 file into buffer passed.
// return a pointer to the full POSIX TZ string within.
#ifndef TZNAME_MAX
#define TZNAME_MAX 6
#endif
#define TZ_BUFLEN   (2*TZNAME_MAX + 56)
static char *read_TZ_file(char *buf)
{
	int r;
	int fd;
	char *p = NULL;

	fd = open("/etc/localtime", O_RDONLY);
	if (fd >= 0) {
		r = read(fd, buf, TZ_BUFLEN);
		if ((r != TZ_BUFLEN) || (strncmp(buf, "TZif", 4) != 0)
			|| ((unsigned char)buf[4] < 2)
			|| (lseek(fd, -TZ_BUFLEN, SEEK_END) < 0)) {
			goto ERROR;
		}
		/* tzfile.h from tzcode database says about TZif2+ files:
		**
		** If tzh_version is '2' or greater, the above is followed by a second instance
		** of tzhead and a second instance of the data in which each coded transition
		** time uses 8 rather than 4 chars,
		** then a POSIX-TZ-environment-variable-style string for use in handling
		** instants after the last transition time stored in the file
		** (with nothing between the newlines if there is no POSIX representation for
		** such instants).
		*/
		r = read(fd, buf, TZ_BUFLEN);
		if ((r <= 0) || (buf[--r] != '\n'))
			goto ERROR;
		buf[r] = 0;
		while (r != 0) {
			if (buf[--r] == '\n') {
				p = buf + r + 1;
				break;
			}
		} /* else ('\n' not found): p remains NULL */
		close(fd);
	} else {
ERROR:
		p = NULL;
		close(fd);
	}

	return p;
}

typedef struct {
        long gmt_offset;
        long dst_offset;
        short day;                          /* for J or normal */
        short week;
        short month;
        short rule_type;                    /* J, M, \0 */
        char tzname[TZNAME_MAX+1];
} rule_struct;

/*
 * Extract offset from POSIX time zone string. 
 */
static char *getoffset(char *e, long *pn)
{
	const char *s = "\0\x19\x3c\x3c\x01";
	long n;
	int f;

	n = 0;
	f = -1;
	do {
		++s;
		if (__isdigit_char(*e)) {
			f = *e++ - '0';
		}
		if (__isdigit_char(*e)) {
			f = 10 * f + (*e++ - '0');
		}
		if (((unsigned int)f) >= *s) {
			return NULL;
		}
		n = (*s) * n + f;
		f = 0;
		if (*e == ':') {
			++e;
			--f;
		}
	} while (*s > 1);

	*pn = n;
	return e;
}

/*
 * Extract number from POSIX time zone string. 
 */
static char *getnumber(char *e, int *pn)
{
	int n, f;

	n = 3;
	f = 0;
	while (n && __isdigit_char(*e)) {
		f = 10 * f + (*e++ - '0');
		--n;
	}

	*pn = f;
	return (n == 3) ? NULL : e;
}

int get_dst_rules(rule_struct *rules) {
	char buf[TZ_BUFLEN];
	char *e, *s;
	long off = 0;
	short *p;
	int n, count, f;
	char c;

	if (rules == NULL)
		return -1;
	
	// Read tzif file
	e = read_TZ_file(buf);
	
	if (!e	|| !*e)	{	/* no TZfile (or bad TZfile) */
	 			/* or set to empty string. */
		printf("read_TZ_file\n");
		goto ILLEGAL;
	}

	if (*e == ':') {	/* Ignore leading ':'. */
		++e;
	}

//uClibc Method
	count = 0;
	rules[1].tzname[0] = 0;
LOOP:
	/* Get std or dst name. */
	c = 0;
	if (*e == '<') {
		++e;
		c = '>';
	}

	s = rules[count].tzname;
	n = 0;
	while (*e
	    && isascii(*e)		/* SUSv3 requires char in portable char set. */
	    && (isalpha(*e)
		|| (c && (isalnum(*e) || (*e == '+') || (*e == '-')))
	       )
	) {
		*s++ = *e++;
		if (++n > TZNAME_MAX) {
			printf("TZNAME_MAX\n");
			goto ILLEGAL;
		}
	}
	*s = 0;

	if ((n < 3)			/* Check for minimum length. */
	 || (c && (*e++ != c))	/* Match any quoting '<'. */
	) {
		printf("min len\n");
		goto ILLEGAL;
	}

	/* Get offset */
	s = (char *) e;
	if ((*e != '-') && (*e != '+')) {
		if (count && !__isdigit_char(*e)) {
			off -= 3600;		/* Default to 1 hour ahead of std. */
			goto SKIP_OFFSET;
		}
		--e;
	}

	++e;
	e = getoffset(e, &off);
	if (!e) {
		printf("getoffset\n");
		goto ILLEGAL;
	}

	if (*s == '-') {
		off = -off;				/* Save off in case needed for dst default. */
	}
SKIP_OFFSET:
	rules[count].gmt_offset = off;

	if (!count) {
		rules[1].gmt_offset = off; /* Shouldn't be needed... */
		if (*e) {
			++count;
			goto LOOP;
		}
	} else {					/* OK, we have dst, so get some rules. */
		count = 0;
		if (!*e) {				/* No rules so default to US 2007+ rules. */
			e = ",M3.2.0,M11.1.0";
		}

		do {
			if (*e++ != ',') {
				printf("first comma\n");
				goto ILLEGAL;
			}

			n = 365;
			s = (char *) "\x01.\x01\x05.\x01\x06\x00\x00\x00\x01\x00";
			c = *e++;
			if (c == 'M') {
				n = 12;
			} else if (c == 'J') {
				s += 8;
			} else {
				--e;
				c = 0;
				s += 6;
			}

			p = &rules[count].rule_type;
			*p = c;
			if (c != 'M') {
				p -= 2;
			}

			do {
				++s;
				e = getnumber(e, &f);
				if (!e
				 || ((unsigned int)(f - s[1]) > n)
				 || (*s && (*e++ != *s))
				) {
					printf("rule %d, s=%x e=%x\n", count, *s, *--e);
					goto ILLEGAL;
				}
				*--p = f;
				s += 2;
				n = *s;
			} while (n > 0);

			off = 2 * 60 * 60;	/* Default to 2:00:00 */
			if (*e == '/') {
				++e;
				e = getoffset(e, &off);
				if (!e) {
					printf("offset %d\n",count);
					goto ILLEGAL;
				}
			}
			rules[count].dst_offset = off;
		} while (++count < 2);

		if (*e) {
ILLEGAL:
			return -1;
		}
	}
	
	return 0;
}

int tod_get_dst_info(dst_info_t *dst_info)
{
	read_tzif();
	if(g_dst_info.type == 1) {
		*dst_info = g_dst_info;
		return 0;
	}
	return -1;
}

int read_dst(dst_info_t *dst_info)
{
	rule_struct new_rules[2];

	if ((dst_info == NULL) || (get_dst_rules(new_rules) == -1)) {
		// Error getting DST rule info or no rule in place
		errno = EINVAL;
		return -1;
	} else if (new_rules[1].tzname[0] == 0) {
		errno = ENOENT;
		return -1;
	}
		
	// Populate dst_info_t from POSIX rules, if any.
	// Rules are stored in new_rules[] array
	dst_info->type = 1; //generic style rules
	// DST Begin Rule
	dst_info->begin.generic.month = new_rules[0].month;
	if (new_rules[0].week == 5) {
		// Specify last occurence in month of dow
		dst_info->begin.generic.dom_type = 2;
		dst_info->begin.generic.generic_dom.reverse_occurrences_of_dow.dow = new_rules[0].day;
		dst_info->begin.generic.generic_dom.reverse_occurrences_of_dow.occur = 1;
		dst_info->begin.generic.generic_dom.reverse_occurrences_of_dow.on_before_dom = 0; //END_OF_MONTH
	} else {
		// Specify nth ocurence in month of dow
		dst_info->begin.generic.dom_type = 1;
		dst_info->begin.generic.generic_dom.forward_occurrences_of_dow.dow = new_rules[0].day;
		dst_info->begin.generic.generic_dom.forward_occurrences_of_dow.occur = new_rules[0].week;
		dst_info->begin.generic.generic_dom.forward_occurrences_of_dow.on_after_dom = 0; //BEGINNING_OF_MONTH
	}
	dst_info->begin.generic.secs_from_midnight_to_transition = new_rules[0].dst_offset;
	dst_info->begin.generic.seconds_to_adjust = new_rules[0].gmt_offset - new_rules[1].gmt_offset;
	
	// DST End Rule
	dst_info->end.generic.month = new_rules[1].month;
	if (new_rules[1].week == 5) {
		// Specify last occurence in month of dow
		dst_info->end.generic.dom_type = 2;
		dst_info->end.generic.generic_dom.reverse_occurrences_of_dow.dow = new_rules[1].day;
		dst_info->end.generic.generic_dom.reverse_occurrences_of_dow.occur = 1;
		dst_info->end.generic.generic_dom.reverse_occurrences_of_dow.on_before_dom = 0; //END_OF_MONTH
	} else {
		// Specify nth ocurence in month of dow
		dst_info->end.generic.dom_type = 1;
		dst_info->end.generic.generic_dom.forward_occurrences_of_dow.dow = new_rules[1].day;
		dst_info->end.generic.generic_dom.forward_occurrences_of_dow.occur = new_rules[1].week;
		dst_info->end.generic.generic_dom.forward_occurrences_of_dow.on_after_dom = 0; //BEGINNING_OF_MONTH
	}
	dst_info->end.generic.secs_from_midnight_to_transition = new_rules[1].dst_offset;
	dst_info->end.generic.seconds_to_adjust = new_rules[1].gmt_offset - new_rules[0].gmt_offset;
	
	return 0;
}

void to_tzif2(const int32_t val, char * const buf)
{
	int i;
	for (i=0; i<4; i++) {
		buf[3-i] = val>>(i*8);
	}
}

void to_tzif2_64(const int64_t val, char * const buf)
{
	int i;
	for (i=0; i<8; i++) {
		buf[7-i] = val>>(i*8);
	}
}

static const int mon_lengths[2][MONSPERYEAR] = {
        { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
        { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

static const int year_lengths[2] = {
        DAYSPERNYEAR, DAYSPERLYEAR
};

static int32_t
transtime(const int year, register const struct rule *const rulep,
	  const int32_t offset)
{
	register bool leapyear;
	register int32_t value = 0;
	register int i;
	int d, m1, y, yy0, yy1, yy2, dow;

	leapyear = isleap(year);
	switch (rulep->r_type) {

	case JULIAN_DAY:
		/*
		** Jn - Julian day, 1 == January 1, 60 == March 1 even in leap
		** years.
		** In non-leap years, or if the day number is 59 or less, just
		** add SECSPERDAY times the day number-1 to the time of
		** January 1, midnight, to get the day.
		*/
		value = (rulep->r_day - 1) * SECSPERDAY;
		if (leapyear && rulep->r_day >= 60)
			value += SECSPERDAY;
		break;

	case DAY_OF_YEAR:
		/*
		** n - day of year.
		** Just add SECSPERDAY times the day number to the time of
		** January 1, midnight, to get the day.
		*/
		value = rulep->r_day * SECSPERDAY;
		break;

	case MONTH_NTH_DAY_OF_WEEK:
		/*
		** Mm.n.d - nth "dth day" of month m.
		*/

		/*
		** Use Zeller's Congruence to get day-of-week of first day of
		** month.
		*/
		m1 = (rulep->r_mon + 9) % 12 + 1;
		yy0 = (rulep->r_mon <= 2) ? (year - 1) : year;
		yy1 = yy0 / 100;
		yy2 = yy0 % 100;
		dow = ((26 * m1 - 2) / 10 +
			1 + yy2 + yy2 / 4 + yy1 / 4 - 2 * yy1) % 7;
		if (dow < 0)
			dow += DAYSPERWEEK;

		/*
		** "dow" is the day-of-week of the first day of the month. Get
		** the day-of-month (zero-origin) of the first "dow" day of the
		** month.
		*/
		d = rulep->r_day - dow;
		if (d < 0)
			d += DAYSPERWEEK;
		for (i = 1; i < rulep->r_week; ++i) {
			if (d + DAYSPERWEEK >=
				mon_lengths[leapyear][rulep->r_mon - 1])
					break;
			d += DAYSPERWEEK;
		}

		/*
		** "d" is the day-of-month (zero-origin) of the day we want.
		*/
		value = d * SECSPERDAY;
		for (i = 0; i < rulep->r_mon - 1; ++i)
			value += mon_lengths[leapyear][i] * SECSPERDAY;
		break;
	}

	/* Add seconds since EPOCH to start of year in question */
	for(y=1970; y<year; y++) {
		value += isleap(y)?(366*24*60*60):(365*24*60*60);
	}

	/*
	** "value" is the year-relative time of 00:00:00 UT on the day in
	** question. To get the year-relative time of the specified local
	** time on that day, add the transition time and the current offset
	** from UT.
	*/
	return value + rulep->r_time + offset;
}

int set_tzif2(const dst_info_t *dst_info, int tz_offset, int dst_offset)
{
	char tzbuf[TZ_BUFLEN];
	char *tzstr = tzbuf;
	char tzif2buf[8];
	int fd;
	struct tzhead tzh = { .tzh_magic = TZ_MAGIC, .tzh_version = '2',
		.tzh_ttisgmtcnt = {0, 0, 0, 1}, .tzh_ttisstdcnt = { 0, 0, 0, 1},
		.tzh_typecnt = {0, 0, 0, 1}, .tzh_charcnt = {0, 0, 0, 6} };
	struct rule dst_rule = { .r_type = MONTH_NTH_DAY_OF_WEEK };
	struct rule std_rule = { .r_type = MONTH_NTH_DAY_OF_WEEK };
	int64_t transition_times[2*(2038-2019)];
	int i;
	int pass;

	// Create POSIX-sytle TZ string
	// Fill std offset from "timezone" global
	tzstr += sprintf(tzstr, "\nATCST%2.2ld:%2.2ld:%2.2ld",
		tz_offset/3600, (tz_offset%3600)/60, (tz_offset%3600)%60);

	if (dst_info != NULL) {
		to_tzif2(2, tzh.tzh_ttisgmtcnt);
		to_tzif2(2, tzh.tzh_ttisstdcnt);
		to_tzif2(2, tzh.tzh_typecnt);
		to_tzif2(12, tzh.tzh_charcnt);
		dst_rule.r_time = dst_info->begin.generic.secs_from_midnight_to_transition;
		std_rule.r_time = dst_info->end.generic.secs_from_midnight_to_transition;
		// We need Month, DOW, occurence number
		dst_rule.r_mon = dst_info->begin.generic.month;
		if ((dst_info->begin.generic.dom_type == 1)
			&& (dst_info->begin.generic.generic_dom.forward_occurrences_of_dow.on_after_dom < 2)) {
			// nth occurence of day from beginning of month
			dst_rule.r_day = dst_info->begin.generic.generic_dom.forward_occurrences_of_dow.dow;
			dst_rule.r_week = dst_info->begin.generic.generic_dom.forward_occurrences_of_dow.occur;
		} else if ((dst_info->begin.generic.dom_type == 2)
			&& (dst_info->begin.generic.generic_dom.reverse_occurrences_of_dow.on_before_dom == 0)
			&& (dst_info->begin.generic.generic_dom.reverse_occurrences_of_dow.occur == 1)) {
			// last occurence of day from end of month
			dst_rule.r_day = dst_info->begin.generic.generic_dom.reverse_occurrences_of_dow.dow;
			dst_rule.r_week = 5;
		} else {
			//error can't represent?
			errno = EINVAL;
			return -1;
		}
		std_rule.r_mon = dst_info->end.generic.month;
		if ((dst_info->end.generic.dom_type == 1)
			&& (dst_info->end.generic.generic_dom.forward_occurrences_of_dow.on_after_dom < 2)) {
			// nth occurence of day from beginning of month
			std_rule.r_day = dst_info->end.generic.generic_dom.forward_occurrences_of_dow.dow;
			std_rule.r_week = dst_info->end.generic.generic_dom.forward_occurrences_of_dow.occur;
		} else if ((dst_info->end.generic.dom_type == 2)
			&& (dst_info->end.generic.generic_dom.reverse_occurrences_of_dow.on_before_dom == 0)
			&& (dst_info->end.generic.generic_dom.reverse_occurrences_of_dow.occur == 1)) {
			// last occurence of day from end of month
			std_rule.r_day = dst_info->end.generic.generic_dom.reverse_occurrences_of_dow.dow;
			std_rule.r_week = 5;
		} else {
			//error can't represent?
			errno = EINVAL;
			return -1;
		}
		
		// Append DST rule to POSIX-style TZ string
		// Fill dst offset
		tzstr += sprintf(tzstr, "ATCDT%2.2ld:%2.2ld:%2.2ld",
			(tz_offset - dst_offset)/3600,
			((tz_offset - dst_offset)%3600)/60,
			((tz_offset - dst_offset)%3600)%60);
		// Fill begin dst rule
		tzstr += sprintf(tzstr, ",M%d.%d.%d/%2.2d:%2.2d:%2.2d",
			dst_rule.r_mon,
			dst_rule.r_week,
			dst_rule.r_day,
			dst_rule.r_time/3600,
			(dst_rule.r_time%3600)/60,
			(dst_rule.r_time%3600)%60);
		// Fill end dst rule
		tzstr += sprintf(tzstr, ",M%d.%d.%d/%2.2d:%2.2d:%2.2d",
			std_rule.r_mon,
			std_rule.r_week,
			std_rule.r_day,
			std_rule.r_time/3600,
			(std_rule.r_time%3600)/60,
			(std_rule.r_time%3600)%60);

		// Create array of transition times for years 2019-2038
		for (i=0; i<(2038-2019); i++) {
			transition_times[i*2] = transtime((2019+i), &dst_rule, tz_offset);
			transition_times[(i*2)+1] = transtime((2019+i), &std_rule, tz_offset-dst_offset);
		} 
	}
	
	// Write TZif2 data to /etc/localtime
	remove("/etc/localtime");
	fd = open("/etc/localtime", O_RDWR|O_CREAT|O_TRUNC,
			S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH);	
	if (!fd)
		return -1;
	
	// TZif2 requires 32-bit data followed by 64-bit version of same 
	for (pass = 1; pass <= 2; ++pass) {
		write(fd, tzh.tzh_magic, sizeof tzh.tzh_magic);
		write(fd, tzh.tzh_version, sizeof tzh.tzh_version);
		write(fd, tzh.tzh_reserved, sizeof tzh.tzh_reserved);
		write(fd, tzh.tzh_ttisgmtcnt, sizeof tzh.tzh_ttisgmtcnt);
		write(fd, tzh.tzh_ttisstdcnt, sizeof tzh.tzh_ttisstdcnt);
		write(fd, tzh.tzh_leapcnt, sizeof tzh.tzh_leapcnt);
		if (dst_info != NULL) {
			// calculate timecnt as (rules per year)*(years to 2038), i.e. 2019 to 2038
			to_tzif2( 2*(2038-2019), tzh.tzh_timecnt);
			write(fd, tzh.tzh_timecnt, sizeof tzh.tzh_timecnt);
			write(fd, tzh.tzh_typecnt, sizeof tzh.tzh_typecnt);
			write(fd, tzh.tzh_charcnt, sizeof tzh.tzh_charcnt);
			// write out timecnt # of transition time_t values
			for (i=0; i<(2038-2019); i++) {
				if (pass == 1) {
					to_tzif2(transition_times[i*2], tzif2buf);
					to_tzif2(transition_times[(i*2)+1], &tzif2buf[4]);
					write(fd, tzif2buf, 8);
				} else {
					to_tzif2_64(transition_times[i*2], tzif2buf);
					write(fd, tzif2buf, 8);
					to_tzif2_64(transition_times[(i*2)+1], tzif2buf);
					write(fd, tzif2buf, 8);
				}
			}
			// write out timecnt # of type values for the above transitions
			for (i=0; i<(2038-2019); i++) {
				tzif2buf[0] = 0;
				tzif2buf[1] = 1; 
				write(fd, tzif2buf, 2);
			}
			// write out the dst offset ttinfo entry
			to_tzif2(-tz_offset + dst_offset, tzif2buf);
			tzif2buf[4] = 1;
			tzif2buf[5] = 0;
			write(fd, tzif2buf, 6);
			// write out the std offset ttinfo entry
			to_tzif2(-tz_offset, tzif2buf);
			tzif2buf[4] = 0;
			tzif2buf[5] = 6;
			write(fd, tzif2buf, 6);
			// write the rule names
			sprintf(tzif2buf, "ATCDT");
			write(fd, tzif2buf, 6);
			sprintf(tzif2buf, "ATCST");
			write(fd, tzif2buf, 6);
			// write std/wall indicators
			tzif2buf[0] = tzif2buf[1] = '\0';
			write(fd, tzif2buf, 2);
			// write UT/local indicators
			write(fd, tzif2buf, 2);
		} else {
			write(fd, tzh.tzh_timecnt, sizeof tzh.tzh_timecnt);
			write(fd, tzh.tzh_typecnt, sizeof tzh.tzh_typecnt);
			write(fd, tzh.tzh_charcnt, sizeof tzh.tzh_charcnt);
			// write out the std offset ttinfo entry
			to_tzif2(-tz_offset, tzif2buf);
			tzif2buf[4] = 0;
			tzif2buf[5] = 0;
			write(fd, tzif2buf, 6);
			// write the rule names
			sprintf(tzif2buf, "ATCST");
			write(fd, tzif2buf, 6);
			// write std/wall indicators
			tzif2buf[0] = '\0';
			write(fd, tzif2buf, 1);
			// write UT/local indicators
			write(fd, tzif2buf, 1);
		}
	}

	// write POSIX-style TZ string
	*tzstr++ = '\n';
	write(fd, tzbuf, (tzstr-tzbuf));
	fsync(fd);
	close(fd);

	// Rename to actual filename (or move to /usr/share/zoneinfo and link?)
	//if (rename("/etc/localtime~", "/etc/localtime") == -1)
	//	return -1;

	return 0;
}

static int write_tzif() {
	if((g_daylight != 0) && (g_dst_info.type == 1)) {
		return set_tzif2(&g_dst_info, g_timezone, g_dst_info.begin.generic.seconds_to_adjust);
	} else {
		return set_tzif2(NULL, g_timezone, 0);
	}
}

int tod_set_dst_info(const dst_info_t *dst_info)
{
	read_tzif();
	if (dst_info == NULL) {
		g_dst_info.type = 0;
		errno = EINVAL;
		return -1;
	}
	g_dst_info = *dst_info;
	return write_tzif();
}

int tod_get_dst_state(void)
{
	read_tzif();
	return (g_daylight?1:0);
}

int tod_set_dst_state(int state)
{
	read_tzif();
	g_daylight = state;
	return write_tzif();
}

int tod_get(struct timeval *tv, int *tzsec_off, int *dst_off)
{
	struct tm local_time;
	struct timeval utc;
	
	read_tzif();
	if ((tv != NULL) || (dst_off != NULL)) {
		gettimeofday(&utc, NULL);
		tzset();
		localtime_r(&utc.tv_sec, &local_time);
	}
	if (tv != NULL) {
		*tv = utc;
	}
	if (tzsec_off != NULL) {
		*tzsec_off = -g_timezone;
	}
	if (dst_off != NULL) {
		/* Correct for all but the 360 residents of Lord Howe Island! */
		*dst_off = ((g_daylight && local_time.tm_isdst)?3600:0);
	}

	return 0;
}

int tod_set(const struct timeval *tv, const int *tzsec_off)
{
	struct timespec tsval;
	struct tm tm;
	int ret = 0;

	read_tzif();
	tzset();
	if (tzsec_off != NULL) {
		g_timezone = -(*tzsec_off);
		ret = write_tzif();
	}
	
	if (tv != NULL) {
		tsval.tv_sec = tv->tv_sec;
		tsval.tv_nsec = tv->tv_usec * 1000;
		if (clock_settime(CLOCK_REALTIME, &tsval) != 0)
			return -1;
	}
	return ret;
}



