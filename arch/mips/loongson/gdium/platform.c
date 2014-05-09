/*
 * Copyright (c) 2009 Philippe Vachon <philippe@cowpig.ca>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

#define GDIUM_GPIO_BASE 224

static struct i2c_board_info __initdata sm502dev_i2c_devices[] = {
	{
		I2C_BOARD_INFO("lm75", 0x48),
	},
	{
		I2C_BOARD_INFO("m41t83", 0x68),
	},
	{
		I2C_BOARD_INFO("gdium-laptop", 0x40),
	},
};

static int sm502dev_backlight_init(struct device *dev)
{
	/* Add gpio request stuff here */
	return 0;
}

static void sm502dev_backlight_exit(struct device *dev)
{
	/* Add gpio free stuff here */
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 15,
	.dft_brightness	= 8,
	.pwm_period_ns	= 50000, /* 20 kHz */
	.init		= sm502dev_backlight_init,
	.exit		= sm502dev_backlight_exit,
};

static struct platform_device backlight = {
	.name = "pwm-backlight",
	.dev  = {
		.platform_data = &backlight_data,
	},
	.id   = -1,
};

/*
 * Warning this stunt is very dangerous
 * as the sm501 gpio have dynamic numbers...
 */
/* bus 0 is the one for the ST7, DS75 etc... */
static struct i2c_gpio_platform_data i2c_gpio0_data = {
#if CONFIG_GDIUM_VERSION > 2
	.sda_pin	= GDIUM_GPIO_BASE + 13,
	.scl_pin	= GDIUM_GPIO_BASE + 6,
#else
	.sda_pin        = 192+15,
	.scl_pin        = 192+14,
#endif
	.udelay		= 5,
	.timeout	= HZ / 10,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
};

static struct platform_device i2c_gpio0_device = {
	.name	= "i2c-gpio",
	.id	= 0,
	.dev	= { .platform_data  = &i2c_gpio0_data, },
};

/* bus 1 is for the CRT/VGA external screen */
static struct i2c_gpio_platform_data i2c_gpio1_data = {
	.sda_pin	= GDIUM_GPIO_BASE + 10,
	.scl_pin	= GDIUM_GPIO_BASE + 9,
	.udelay		= 5,
	.timeout	= HZ / 10,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
};

static struct platform_device i2c_gpio1_device = {
	.name	= "i2c-gpio",
	.id	= 1,
	.dev	= { .platform_data  = &i2c_gpio1_data, },
};

static struct platform_device gdium_clock = {
	.name		= "gdium-pwmclk",
	.id		= -1,
};

static struct platform_device *devices[] __initdata = {
	&i2c_gpio0_device,
	&i2c_gpio1_device,
	&backlight,
	&gdium_clock,
};

static int __init gdium_platform_devices_setup(void)
{
	int ret;

	pr_info("Registering gdium platform devices\n");

	ret = i2c_register_board_info(0, sm502dev_i2c_devices,
		ARRAY_SIZE(sm502dev_i2c_devices));

	if (ret != 0) {
		pr_info("Error while registering platform devices: %d\n", ret);
		return ret;
	}

	platform_add_devices(devices, ARRAY_SIZE(devices));

	return 0;
}

/*
 * some devices are on the pwm stuff which is behind the mfd which is
 * behind the pci bus so arch_initcall can't work because too early
 */
late_initcall(gdium_platform_devices_setup);
