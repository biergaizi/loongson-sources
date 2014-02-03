/*
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/err.h>
#include <linux/platform_device.h>

#include <asm/bootinfo.h>

static struct platform_device yeeloong_pdev = {
	.name = "yeeloong_laptop",
	.id = -1,
};

static struct platform_device lynloong_pdev = {
	.name = "lynloong_pc",
	.id = -1,
};

static int __init lemote2f_platform_init(void)
{
	struct platform_device *pdev = NULL;

	switch (mips_machtype) {
	case MACH_LEMOTE_YL2F89:
		pdev = &yeeloong_pdev;
		break;
	case MACH_LEMOTE_LL2F:
		pdev = &lynloong_pdev;
		break;
	default:
		break;

	}

	if (pdev != NULL)
		return platform_device_register(pdev);

	return -ENODEV;
}

arch_initcall(lemote2f_platform_init);
