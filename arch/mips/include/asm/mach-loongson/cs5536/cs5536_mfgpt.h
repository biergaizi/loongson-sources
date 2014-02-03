/*
 * cs5536 mfgpt header file
 */

#ifndef _CS5536_MFGPT_H
#define _CS5536_MFGPT_H

#include <cs5536/cs5536.h>
#include <cs5536/cs5536_pci.h>

#ifdef CONFIG_CS5536_MFGPT
extern void setup_mfgpt0_timer(void);
extern void disable_mfgpt_counter(void);
extern void enable_mfgpt_counter(void);
#else
static inline void __maybe_unused setup_mfgpt0_timer(void)
{
}
static inline void __maybe_unused disable_mfgpt_counter(void)
{
}
static inline void __maybe_unused enable_mfgpt_counter(void)
{
}
#endif

#define MFGPT_CLK_RATE(c)		((14318000UL-32768)*c + 32768)
#define MFGPT_TICK_RATE(c, scale)	(MFGPT_CLK_RATE(c) / (1 << scale))
#define MFGPT_COMPARE(c, scale)		((MFGPT_TICK_RATE(c, scale)+HZ/2)/HZ)

#define MFGPT_SETUP_ENABLE		(1 << 15)
#define MFGPT_SETUP_ACK			(3 << 13)
#define MFGPT_SETUP_SETUP		(1 << 12)
#define MFGPT_SETUP_CMP2EVT		(3 <<  8)
#define MFGPT_SETUP_CMP1EVT		(3 <<  6)
#define MFGPT_SETUP_CLOCK(c)		(c <<  4)
#define MFGPT_SETUP_SCALE(scale)	scale

#define MFGPT0_CMP1	mfgpt_base
#define MFGPT0_CMP2	(mfgpt_base + 0x02)
#define MFGPT0_CNT	(mfgpt_base + 0x04)
#define MFGPT0_SETUP	(mfgpt_base + 0x06)

#define MFGPT1_CMP1	(mfgpt_base + 0x08)
#define MFGPT1_CMP2	(mfgpt_base + 0x0A)
#define MFGPT1_CNT	(mfgpt_base + 0x0C)
#define MFGPT1_SETUP	(mfgpt_base + 0x0E)

#define MFGPT2_CMP1	(mfgpt_base + 0x10)
#define MFGPT2_CMP2	(mfgpt_base + 0x12)
#define MFGPT2_CNT	(mfgpt_base + 0x14)
#define MFGPT2_SETUP	(mfgpt_base + 0x16)

#endif /*!_CS5536_MFGPT_H */
