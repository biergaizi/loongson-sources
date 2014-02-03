/*
 * the read/write interfaces for Virtual Support Module(VSM)
 *
 * Copyright (C) 2009 Lemote, Inc.
 * Author: Wu Zhangjin <wuzhangjin@gmail.com>
 */

#ifndef _CS5536_VSM_H
#define _CS5536_VSM_H

#include <linux/types.h>

typedef void (*cs5536_pci_vsm_write)(int reg, u32 value);
typedef u32 (*cs5536_pci_vsm_read)(int reg);

#define DECLARE_CS5536_MODULE(name) \
extern void pci_##name##_write_reg(int reg, u32 value); \
extern u32 pci_##name##_read_reg(int reg);

#define DEFINE_CS5536_MODULE(name) \
static void pci_##name##_write_reg(int reg, u32 value) {} \
static u32 pci_##name##_read_reg(int reg) { return 0; }

/* isa module */
#ifdef CONFIG_CS5536_ISA
DECLARE_CS5536_MODULE(isa)
#else
DEFINE_CS5536_MODULE(isa)
#endif

/* ide module */
#ifdef CONFIG_CS5536_IDE
DECLARE_CS5536_MODULE(ide)
#else
DEFINE_CS5536_MODULE(ide)
#endif

/* acc module */
#ifdef CONFIG_CS5536_AUDIO
DECLARE_CS5536_MODULE(acc)
#else
DEFINE_CS5536_MODULE(acc)
#endif

/* ohci module */
#ifdef CONFIG_CS5536_OHCI
DECLARE_CS5536_MODULE(ohci)
#else
DEFINE_CS5536_MODULE(ohci)
#endif

/* ehci module */
#ifdef CONFIG_CS5536_EHCI
DECLARE_CS5536_MODULE(ehci)
#else
DEFINE_CS5536_MODULE(ehci)
#endif

#endif				/* _CS5536_VSM_H */
