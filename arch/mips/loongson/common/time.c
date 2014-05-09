/*
 * Copyright (C) 2007 Lemote, Inc. & Institute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/rtc.h>

#include <asm/mc146818-time.h>
#include <asm/time.h>

#include <loongson.h>
#include <cs5536/cs5536_mfgpt.h>

void __init plat_time_init(void)
{
	/* setup mips r4k timer */
	mips_hpt_frequency = cpu_clock_freq / 2;

	setup_mfgpt0_timer();
}

#ifdef CONFIG_LOONGSON_MC146818
void read_persistent_clock(struct timespec *ts)
{
	ts->tv_sec = mc146818_get_cmos_time();
	ts->tv_nsec = 0;
}
#else

/* If no CMOS RTC, use the one below */

/*
 * Cloned from drivers/rtc/hctosys.c
 *
 * If CONFIG_RTC_HCTOSYS=y is enabled, the system time can be set from the
 * hardware clock(when boot and resuming from suspend), this may be also done
 * (duplicately) by the timekeeper, which may need to be avoided(TODO).
 *
 * read_persistent_clock() may be useful in some places, e.g. there is not
 * peristent clock in the system, we can use this to recover the system time.
 *
 * Note: The device indicated by CONFIG_RTC_HCTOSYS_DEVICE must be the one
 * created by the RTC driver. Use Gdium as an example, We must disable the
 * rt_cmos driver If we want to use the rtc_m41t80 driver for
 * CONFIG_RTC_HCTOSYS_DEVICE is configured as /dev/rtc0, if rtc_cmos is
 * enabled, rtc_cmos driver will be used, but it is not supported by Gdium.
 * So, for Gdium, please ensure "# CONFIG_RTC_DRV_CMOS is not set"
 */

#ifdef CONFIG_RTC_HCTOSYS
void read_persistent_clock(struct timespec *ts)
{
	int err = -ENODEV;
	struct rtc_time tm;
	struct rtc_device *rtc;

	/* We can not access the RTC device before it is initialized ... */
	if (rtc_hctosys_ret != 0) {
		ts->tv_sec = 0;
		ts->tv_nsec = 0;
		return;
	}

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);

	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		goto err_open;
	}

	err = rtc_read_time(rtc, &tm);
	if (err) {
		dev_err(rtc->dev.parent,
			"hctosys: unable to read the hardware clock\n");
		goto err_read;

	}

	err = rtc_valid_tm(&tm);
	if (err) {
		dev_err(rtc->dev.parent,
			"hctosys: invalid date/time\n");
		goto err_invalid;
	}

	ts->tv_nsec = NSEC_PER_SEC >> 1,
	rtc_tm_to_time(&tm, &ts->tv_sec);

err_invalid:
err_read:
	rtc_class_close(rtc);

err_open:
	rtc_hctosys_ret = err;
}
#endif /* CONFIG_RTC_HCTOSYS */

#endif /* !CONFIG_LOONGSON_MC146818 */
