/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2007 Ralf Baechle (ralf@linux-mips.org)
 *
 * Copyright (C) 2009 Lemote, Inc.
 * Author: Yan hua (yanhua@lemote.com)
 * Author: Wu Zhangjin (wuzhangjin@gmail.com)
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/serial_8250.h>

#include <asm/bootinfo.h>

#include <loongson.h>
#include <machine.h>

#define PORT(int, base_baud, io_type, port)			\
{								\
	.irq		= int,					\
	.uartclk	= base_baud,				\
	.iotype		= io_type,				\
	.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,	\
	.regshift	= 0,					\
	.iobase	= port,						\
}

static struct plat_serial8250_port uart8250_data[] = {
	/* ttyS0: cpu_uart0 Yeeloong, Gdium, UNAS, ...  */
	PORT((MIPS_CPU_IRQ_BASE + 3), 3686400, UPIO_MEM, 0x3f8),
	/* ttyS1: sb_uart1 2E */
	PORT(4, 1843200, UPIO_PORT, 0x3f8),
	/* ttyS2: sb_uart2 fuloong2f */
	PORT(3, 1843200, UPIO_PORT, 0x2f8),
	{},
};

static struct platform_device uart8250_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = uart8250_data,
	},
};

static int __init serial_init(void)
{
	uart8250_data[0].membase = (void __iomem *)ioremap_nocache(
			LOONGSON_LIO1_BASE + uart8250_data[0].iobase, 8);

	platform_device_register(&uart8250_device);
	return 0;
}

device_initcall(serial_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("liu shiwei <liushiwei@anheng.com.cn>");
MODULE_DESCRIPTION("loongson serial");
