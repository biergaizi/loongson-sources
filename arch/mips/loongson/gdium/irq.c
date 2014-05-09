/*
 * Copyright (C) 2007 Lemote Inc.
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 * Copyright (c) 2010 yajin <yajin@vm-kernel.org>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/module.h>

#include <loongson.h>
#include <machine.h>

#define LOONGSON_TIMER_IRQ      (MIPS_CPU_IRQ_BASE + 7) /* cpu timer */
#define LOONGSON_NORTH_BRIDGE_IRQ       (MIPS_CPU_IRQ_BASE + 6) /* bonito */
#define LOONGSON_UART_IRQ       (MIPS_CPU_IRQ_BASE + 3) /* cpu serial port */

void mach_irq_dispatch(unsigned int pending)
{
	if (pending & CAUSEF_IP7)
		do_IRQ(LOONGSON_TIMER_IRQ);
	else if (pending & CAUSEF_IP6) {        /* North Bridge, Perf counter */
		do_perfcnt_IRQ();
		bonito_irqdispatch();
	} else if (pending & CAUSEF_IP3)        /* CPU UART */
		do_IRQ(LOONGSON_UART_IRQ);
#if defined(CONFIG_GDIUM_PWM_CLOCK) || defined(CONFIG_GDIUM_PWM_CLOCK_MODULE)
	else if (pending & CAUSEF_IP4)		/* SM501 PWM clock */
		do_IRQ(MIPS_CPU_IRQ_BASE + 4);
#endif
	else
		spurious_interrupt();
}

static irqreturn_t ip6_action(int cpl, void *dev_id)
{
	return IRQ_HANDLED;
}

struct irqaction ip6_irqaction = {
	.handler = ip6_action,
	.name = "cascade",
	.flags = IRQF_SHARED,
};

void __init mach_init_irq(void)
{
	/* setup north bridge irq (bonito) */
	setup_irq(LOONGSON_NORTH_BRIDGE_IRQ, &ip6_irqaction);
}
