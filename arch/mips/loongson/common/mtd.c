/*
 *  Driver for flushing/dumping ROM of PMON on loongson family machines
 *
 *  Copyright (C) 2008-2009 Lemote Inc.
 *  Author: Yan Hua <yanh@lemote.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

#include <loongson.h>

#define FLASH_PHYS_ADDR LOONGSON_BOOT_BASE
#define FLASH_SIZE 0x080000

#define FLASH_PARTITION0_ADDR 0x00000000
#define FLASH_PARTITION0_SIZE 0x00080000

struct map_info flash_map = {
	.name = "flash device",
	.size = FLASH_SIZE,
	.bankwidth = 1,
};

struct mtd_partition flash_parts[] = {
	{
	 .name = "Bootloader",
	 .offset = FLASH_PARTITION0_ADDR,
	 .size = FLASH_PARTITION0_SIZE},
};

#define PARTITION_COUNT ARRAY_SIZE(flash_parts)

static struct mtd_info *mymtd;

int __init init_flash(void)
{
	printk(KERN_NOTICE "flash device: %x at %x\n",
	       FLASH_SIZE, FLASH_PHYS_ADDR);

	flash_map.phys = FLASH_PHYS_ADDR;
	flash_map.virt = ioremap(FLASH_PHYS_ADDR, FLASH_SIZE);

	if (!flash_map.virt) {
		printk(KERN_NOTICE "Failed to ioremap\n");
		return -EIO;
	}

	simple_map_init(&flash_map);

	mymtd = do_map_probe("cfi_probe", &flash_map);
	if (mymtd) {
		add_mtd_partitions(mymtd, flash_parts, PARTITION_COUNT);
		printk(KERN_NOTICE "pmon flash device initialized\n");
		return 0;
	}

	iounmap((void *)flash_map.virt);
	return -ENXIO;
}

static void __exit cleanup_flash(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
	}
	if (flash_map.virt) {
		iounmap((void *)flash_map.virt);
		flash_map.virt = 0;
	}
}

module_init(init_flash);
module_exit(cleanup_flash);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yanhua <yanh@lemote.com>");
MODULE_DESCRIPTION("MTD driver for pmon flushing/dumping");
