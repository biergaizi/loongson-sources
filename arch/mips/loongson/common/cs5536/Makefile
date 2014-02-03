#
# Makefile for CS5536 support.
#

obj-$(CONFIG_CS5536)	+= cs5536_pci.o

obj-$(CONFIG_ISA)		+= cs5536_isa.o
obj-$(CONFIG_CS5536_IDE)	+= cs5536_ide.o
obj-$(CONFIG_CS5536_AUDIO)	+= cs5536_acc.o
obj-$(CONFIG_CS5536_OHCI)	+= cs5536_ohci.o
obj-$(CONFIG_CS5536_EHCI)	+= cs5536_ehci.o

#
# Enable cs5536 mfgpt Timer
#
obj-$(CONFIG_CS5536_MFGPT) += cs5536_mfgpt.o
