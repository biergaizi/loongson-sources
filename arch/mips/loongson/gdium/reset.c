/* Board-specific reboot/shutdown routines
 *
 * Copyright (C) 2010 yajin <yajin@vm-kernel.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <loongson.h>

void mach_prepare_shutdown(void)
{
	LOONGSON_GPIOIE &= ~(1<<1);
	LOONGSON_GPIODATA |= (1<<1);
}

void mach_prepare_reboot(void)
{
	LOONGSON_GPIOIE &= ~(1<<2);
	LOONGSON_GPIODATA &= ~(1<<2);
}
