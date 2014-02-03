/*
 * CS5536 General timer functions
 *
 * Copyright (C) 2007 Lemote Inc. & Insititute of Computing Technology
 * Author: Yanhua, yanh@lemote.com
 *
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu zhangjin, wuzhangjin@gmail.com
 *
 * Copyright (C) 2010 Lemote Inc.
 * Author: Gang Liang, randomizedthinking@gmail.com
 *
 * Reference: AMD Geode(TM) CS5536 Companion Device Data Book
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */

/*
 * The MFGPT base address is variable, i.e., it could change over time. In
 * reality, it only changes once when setting up the PCI memory mapping (occurs
 * about 0.2 second from boot).  But because of this, we have to read in the
 * mfgpt base address repeatly in the beginning of various routines, most
 * noticeably, mfgpt1_read_cycle (for sched_clock), and mfgpt1_interrupt.
 *
 * The source of problem is that PMON and the current cs5536 set up pci
 * register window differently (to be further confirmed). Can we set
 * them the same so as to save the trouble here?
 *
 * Now an ugly hack is used to save a few CPU cycles... likely an
 * over-optimization. Feel free to remove it.
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>

#include <asm/time.h>

#include <cs5536/cs5536_mfgpt.h>

static void mfgpt0_set_mode(enum clock_event_mode, struct clock_event_device*);
static int mfgpt0_next_event(unsigned long, struct clock_event_device*);
static irqreturn_t mfgpt0_interrupt(int irq, void *dev_id);
static void mfgpt0_start_timer(u16 delta);

static cycle_t mfgpt1_read_cycle(struct clocksource *cs);

static enum clock_event_mode mfgpt0_mode = CLOCK_EVT_MODE_SHUTDOWN;
static u32 mfgpt_base;

static struct clock_event_device mfgpt0_clockevent = {
	.name = "mfgpt0",
	.features = CLOCK_EVT_MODE_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.set_mode = mfgpt0_set_mode,
	.set_next_event = mfgpt0_next_event,
	.rating = 220,
	.irq = CS5536_MFGPT_INTR,
};

static struct irqaction irq5 = {
	.handler = mfgpt0_interrupt,
	.flags = IRQF_DISABLED | IRQF_NOBALANCING | IRQF_TIMER,
	.name = "mfgpt0-timer"
};

static struct clocksource mfgpt1_clocksource = {
	.name = "mfgpt1",
	.rating = 210,
	.read = mfgpt1_read_cycle,
	.mask = CLOCKSOURCE_MASK(16),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS
};

static inline void enable_mfgpt0_counter(void)
{
	u32 basehi;
	_rdmsr(DIVIL_MSR_REG(DIVIL_LBAR_MFGPT), &basehi, &mfgpt_base);

	/* clockevent: 14M, divisor = 8 (scale=3), CMP2 event mode */
	outw(MFGPT_SETUP_ACK | MFGPT_SETUP_CMP2EVT |
	     MFGPT_SETUP_CLOCK(1) | MFGPT_SETUP_SCALE(3), MFGPT0_SETUP);
	outw(0, MFGPT0_CNT);
	outw(MFGPT_COMPARE(1, 3), MFGPT0_CMP2);
	outw(0xFFFF, MFGPT0_SETUP);
}

static inline void enable_mfgpt1_counter(void)
{
	u32 basehi;
	_rdmsr(DIVIL_MSR_REG(DIVIL_LBAR_MFGPT), &basehi, &mfgpt_base);

	/* clocksource: 32K w/ divisor = 2 (scale=1) */
	outw(MFGPT_SETUP_ACK | MFGPT_SETUP_CLOCK(0) |
		MFGPT_SETUP_SCALE(1), MFGPT1_SETUP);

	outw(0, MFGPT1_CNT);
	outw(0xFFFF, MFGPT1_CMP2);  /* CNT won't tick with no CMP set */
	outw(0xFFFF, MFGPT1_SETUP);
}

void enable_mfgpt_counter(void)
{
	/* TODO: add a mfgpt system hard reset here
	 * timers might not reset correctly when OS crashes
	 */

	enable_mfgpt0_counter();
	enable_mfgpt1_counter();
}
EXPORT_SYMBOL(enable_mfgpt_counter);

void disable_mfgpt_counter(void)
{
	outw(0x7FFF, MFGPT0_SETUP);
	outw(0x7FFF, MFGPT1_SETUP);
}
EXPORT_SYMBOL(disable_mfgpt_counter);

static void mfgpt0_start_timer(u16 delta)
{
	outw(0x7FFF, MFGPT0_SETUP);
	outw(0,      MFGPT0_CNT);
	outw(delta,  MFGPT0_CMP2);
	outw(0xFFFF, MFGPT0_SETUP);
}

static void mfgpt0_set_mode(enum clock_event_mode mode,
		struct clock_event_device *evt)
{
	outw(0x7FFF, MFGPT0_SETUP);
	if (mode == CLOCK_EVT_MODE_PERIODIC)
		mfgpt0_start_timer(MFGPT_COMPARE(1, 3));

	mfgpt0_mode = mode;
}

static int mfgpt0_next_event(unsigned long delta,
		struct clock_event_device *evt)
{
	mfgpt0_start_timer(delta);
	return 0;
}

static irqreturn_t mfgpt0_interrupt(int irq, void *dev_id)
{
	u32 basehi;
	_rdmsr(DIVIL_MSR_REG(DIVIL_LBAR_MFGPT), &basehi, &mfgpt_base);

	/* stop the timer and ack the interrupt */
	outw(0x7FFF, MFGPT0_SETUP);

	if (mfgpt0_mode == CLOCK_EVT_MODE_SHUTDOWN)
		return IRQ_HANDLED;

	/* restart timer for periodic mode */
	if (mfgpt0_mode == CLOCK_EVT_MODE_PERIODIC)
		outw(0xFFFF, MFGPT0_SETUP);

	mfgpt0_clockevent.event_handler(&mfgpt0_clockevent);
	return IRQ_HANDLED;
}

/*
 * Initialize the conversion factor and the min/max deltas of the clock event
 * structure and register the clock event source with the framework.
 */
void __init setup_mfgpt0_timer(void)
{
	struct clock_event_device *cd = &mfgpt0_clockevent;
	unsigned int cpu = smp_processor_id();
	cd->cpumask = cpumask_of(cpu);

	cd->shift = 22;
	cd->mult  = div_sc(MFGPT_TICK_RATE(1, 3), NSEC_PER_SEC, cd->shift);

	cd->min_delta_ns = clockevent_delta2ns(0xF, cd);
	cd->max_delta_ns = clockevent_delta2ns(0xFFFF, cd);

	/* Enable MFGPT0 Comparator 2 Output to the Interrupt Mapper */
	_wrmsr(DIVIL_MSR_REG(MFGPT_IRQ), 0, 0x100);

	/* Enable Interrupt Gate 5 */
	_wrmsr(DIVIL_MSR_REG(PIC_ZSEL_LOW), 0, 0x50000);

	enable_mfgpt0_counter();
	clockevents_register_device(cd);
	setup_irq(CS5536_MFGPT_INTR, &irq5);
}

static cycle_t mfgpt1_read_cycle(struct clocksource *cs)
{
	return inw(MFGPT1_CNT);
}

int __init init_mfgpt1_clocksource(void)
{
	if (num_possible_cpus() > 1)	/* MFGPT does not scale! */
		return 0;

	enable_mfgpt1_counter();

	return clocksource_register_hz(&mfgpt1_clocksource, MFGPT_TICK_RATE(0, 1));
}

arch_initcall(init_mfgpt1_clocksource);
