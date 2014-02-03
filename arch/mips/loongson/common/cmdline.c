/*
 * Based on Ocelot Linux port, which is
 * Copyright 2001 MontaVista Software Inc.
 * Author: jsun@mvista.com or jsun@junsun.net
 *
 * Copyright 2003 ICT CAS
 * Author: Michael Guo <guoyi@ict.ac.cn>
 *
 * Copyright (C) 2007 Lemote Inc. & Insititute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <asm/bootinfo.h>

#include <loongson.h>

/* the kernel command line copied from arcs_cmdline */
char loongson_cmdline[COMMAND_LINE_SIZE];
EXPORT_SYMBOL(loongson_cmdline);

void __init prom_init_cmdline(void)
{
	int prom_argc;
	/* pmon passes arguments in 32bit pointers */
	int *_prom_argv;
	int i;
	long l;

	/* firmware arguments are initialized in head.S */
	prom_argc = fw_arg0;
	_prom_argv = (int *)fw_arg1;

	/* arg[0] is "g", the rest is boot parameters */
	arcs_cmdline[0] = '\0';
	for (i = 1; i < prom_argc; i++) {
		l = (long)_prom_argv[i];
		if (strlen(arcs_cmdline) + strlen(((char *)l) + 1)
		    >= sizeof(arcs_cmdline))
			break;
		strcat(arcs_cmdline, ((char *)l));
		strcat(arcs_cmdline, " ");
	}

	prom_init_machtype();

	/* append machine specific command line */
	switch (mips_machtype) {
	case MACH_LEMOTE_LL2F:
		if ((strstr(arcs_cmdline, "video=")) == NULL)
			strcat(arcs_cmdline, " video=sisfb:1360x768-16@60");
		break;
	case MACH_LEMOTE_FL2F:
		if ((strstr(arcs_cmdline, "ide_core.ignore_cable=")) == NULL)
			strcat(arcs_cmdline, " ide_core.ignore_cable=0");
		break;
	case MACH_LEMOTE_ML2F7:
		/* Mengloong-2F has a 800x480 screen */
		if ((strstr(arcs_cmdline, "vga=")) == NULL)
			strcat(arcs_cmdline, " vga=0x313");
		break;
	case MACH_DEXXON_GDIUM2F10:
		/* gdium has a 1024x600 screen */
		if ((strstr(arcs_cmdline, "video=")) == NULL)
			strcat(arcs_cmdline, " video=sm501fb:1024x600@60");
		break;
	default:
		break;
	}

	/* copy arcs_cmdline into loongson_cmdline */
	strncpy(loongson_cmdline, arcs_cmdline, COMMAND_LINE_SIZE);
}
