/*
 * Copyright (c) 2011 Zhang, Keguang <keguang.zhang@gmail.com>
 *
 * Loongson1 Interrupt register definitions.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_MACH_LOONGSON1_REGS_INTC_H
#define __ASM_MACH_LOONGSON1_REGS_INTC_H

#define LS1X_INTC_REG(n, x) \
		(ioremap(LS1X_INTC_BASE + (n * 0x18) + (x), 4))

#define LS1X_INTC_INTISR(n)		LS1X_INTC_REG(n, 0x0)
#define LS1X_INTC_INTIEN(n)		LS1X_INTC_REG(n, 0x4)
#define LS1X_INTC_INTSET(n)		LS1X_INTC_REG(n, 0x8)
#define LS1X_INTC_INTCLR(n)		LS1X_INTC_REG(n, 0xc)
#define LS1X_INTC_INTPOL(n)		LS1X_INTC_REG(n, 0x10)
#define LS1X_INTC_INTEDGE(n)		LS1X_INTC_REG(n, 0x14)

#endif /* __ASM_MACH_LOONGSON1_REGS_INTC_H */
