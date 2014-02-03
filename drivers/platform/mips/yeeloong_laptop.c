/*
 * Driver for YeeLoong laptop extras
 *
 *  Copyright (C) 2009 Lemote Inc.
 *  Author: Wu Zhangjin <wuzhangjin@gmail.com>, Liu Junliang <liujl@lemote.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>	/* for backlight subdriver */
#include <linux/fb.h>
#include <linux/hwmon.h>	/* for hwmon subdriver */
#include <linux/hwmon-sysfs.h>
#include <linux/video_output.h>	/* for video output subdriver */
#include <linux/lcd.h>		/* for lcd output subdriver */
#include <linux/input.h>	/* for hotkey subdriver */
#include <linux/input/sparse-keymap.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/power_supply.h>	/* for AC & Battery subdriver */
#include <linux/reboot.h>	/* for register_reboot_notifier */
#include <linux/suspend.h>	/* for register_pm_notifier */

#include <cs5536/cs5536.h>

#include <loongson.h>		/* for loongson_cmdline */
#include <ec_kb3310b.h>

#define ON	1
#define OFF	0
#define EVENT_START EVENT_LID

/* common function */
#define EC_VER_LEN 64

static int ec_version_before(char *version)
{
	char *p, ec_ver[EC_VER_LEN];

	p = strstr(loongson_cmdline, "EC_VER=");
	if (!p)
		memset(ec_ver, 0, EC_VER_LEN);
	else {
		strncpy(ec_ver, p, EC_VER_LEN);
		p = strstr(ec_ver, " ");
		if (p)
			*p = '\0';
	}

	return (strncasecmp(ec_ver, version, 64) < 0);
}

/* backlight subdriver */
#define MIN_BRIGHTNESS	1
#define MAX_BRIGHTNESS	8

static int yeeloong_set_brightness(struct backlight_device *bd)
{
	unsigned char level;
	static unsigned char old_level;

	level = (bd->props.fb_blank == FB_BLANK_UNBLANK &&
		 bd->props.power == FB_BLANK_UNBLANK) ?
	    bd->props.brightness : 0;

	level = clamp_val(level, MIN_BRIGHTNESS, MAX_BRIGHTNESS);

	/* Avoid to modify the brightness when EC is tuning it */
	if (old_level != level) {
		if (ec_read(REG_DISPLAY_BRIGHTNESS) == old_level)
			ec_write(REG_DISPLAY_BRIGHTNESS, level);
		old_level = level;
	}

	return 0;
}

static int yeeloong_get_brightness(struct backlight_device *bd)
{
	return ec_read(REG_DISPLAY_BRIGHTNESS);
}

static struct backlight_ops backlight_ops = {
	.get_brightness = yeeloong_get_brightness,
	.update_status = yeeloong_set_brightness,
};

static struct backlight_device *yeeloong_backlight_dev;

static int yeeloong_backlight_init(void)
{
	int ret;
	struct backlight_properties props;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = MAX_BRIGHTNESS;
	props.type = BACKLIGHT_PLATFORM;
	yeeloong_backlight_dev = backlight_device_register("backlight0", NULL,
			NULL, &backlight_ops, &props);

	if (IS_ERR(yeeloong_backlight_dev)) {
		ret = PTR_ERR(yeeloong_backlight_dev);
		yeeloong_backlight_dev = NULL;
		return ret;
	}

	yeeloong_backlight_dev->props.brightness =
		yeeloong_get_brightness(yeeloong_backlight_dev);
	backlight_update_status(yeeloong_backlight_dev);

	return 0;
}

static void yeeloong_backlight_exit(void)
{
	if (yeeloong_backlight_dev) {
		backlight_device_unregister(yeeloong_backlight_dev);
		yeeloong_backlight_dev = NULL;
	}
}

/* AC & Battery subdriver */

static struct power_supply yeeloong_ac, yeeloong_bat;

#define RET (val->intval)

#define BAT_CAP_CRITICAL 5
#define BAT_CAP_HIGH     95

#define get_bat(type) \
	ec_read(REG_BAT_##type)

#define get_bat_l(type) \
	((get_bat(type##_HIGH) << 8) | get_bat(type##_LOW))

static int yeeloong_get_ac_props(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	if (psp == POWER_SUPPLY_PROP_ONLINE)
		RET = !!(get_bat(POWER) & BIT_BAT_POWER_ACIN);

	return 0;
}

static enum power_supply_property yeeloong_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply yeeloong_ac = {
	.name = "yeeloong-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = yeeloong_ac_props,
	.num_properties = ARRAY_SIZE(yeeloong_ac_props),
	.get_property = yeeloong_get_ac_props,
};

static inline bool is_bat_in(void)
{
	return !!(get_bat(STATUS) & BIT_BAT_STATUS_IN);
}

static int get_bat_temp(void)
{
	return get_bat_l(TEMPERATURE) * 10;
}

static int get_bat_current(void)
{
	return -(s16)get_bat_l(CURRENT);
}

static int get_bat_voltage(void)
{
	return get_bat_l(VOLTAGE);
}

static char *get_manufacturer(void)
{
	return (get_bat(VENDOR) == FLAG_BAT_VENDOR_SANYO) ? "SANYO" : "SIMPLO";
}

static int get_relative_cap(void)
{
	/*
	 * When the relative capacity becomes 2, the hardware is observed to
	 * have been turned off forcely. so, we must tune it be suitable to
	 * make the software do related actions.
	 */
	int tmp = get_bat_l(RELATIVE_CAP);

	if (tmp <= (BAT_CAP_CRITICAL * 2))
		tmp -= 3;

	return tmp;
}

static int yeeloong_get_bat_props(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	switch (psp) {
	/* Fixed information */
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		/* mV -> µV */
		RET = get_bat_l(DESIGN_VOL) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		/* mAh->µAh */
		RET = get_bat_l(DESIGN_CAP) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		/* µAh */
		RET = get_bat_l(FULLCHG_CAP) * 1000;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = get_manufacturer();
		break;
	/* Dynamic information */
	case POWER_SUPPLY_PROP_PRESENT:
		RET = is_bat_in();
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* mA -> µA */
		RET = is_bat_in() ? get_bat_current() * 1000 : 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* mV -> µV */
		RET = is_bat_in() ? get_bat_voltage() * 1000 : 0;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		/* Celcius */
		RET = is_bat_in() ? get_bat_temp() : 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		RET = is_bat_in() ? get_relative_cap() : 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL: {
		int status;

		if (!is_bat_in()) {
			RET = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
			break;
		}

		status = get_bat(STATUS);
		RET = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

		if (unlikely(status & BIT_BAT_STATUS_DESTROY)) {
			RET = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
			break;
		}

		if (status & BIT_BAT_STATUS_FULL)
			RET = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else {
			int curr_cap = get_relative_cap();

			if (status & BIT_BAT_STATUS_LOW) {
				RET = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
				if (curr_cap <= BAT_CAP_CRITICAL)
					RET = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
			} else if (curr_cap >= BAT_CAP_HIGH)
				RET = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		}
	} break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		/* seconds */
		RET = is_bat_in() ? (get_relative_cap() - 3) * 54 + 142 : 0;
		break;
	case POWER_SUPPLY_PROP_STATUS: {
			int charge = get_bat(CHARGE);

			RET = POWER_SUPPLY_STATUS_UNKNOWN;
			if (charge & FLAG_BAT_CHARGE_DISCHARGE)
				RET = POWER_SUPPLY_STATUS_DISCHARGING;
			else if (charge & FLAG_BAT_CHARGE_CHARGE)
				RET = POWER_SUPPLY_STATUS_CHARGING;
	} break;
	case POWER_SUPPLY_PROP_HEALTH: {
			int status;

			if (!is_bat_in()) {
				RET = POWER_SUPPLY_HEALTH_UNKNOWN;
				break;
			}

			status = get_bat(STATUS);
			RET = POWER_SUPPLY_HEALTH_GOOD;

			if (status & (BIT_BAT_STATUS_DESTROY |
						BIT_BAT_STATUS_LOW))
				RET = POWER_SUPPLY_HEALTH_DEAD;
			if (get_bat(CHARGE_STATUS) &
					BIT_BAT_CHARGE_STATUS_OVERTEMP)
				RET = POWER_SUPPLY_HEALTH_OVERHEAT;
	} break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:	/* 1/100(%)*1000 µAh */
		RET = get_relative_cap() * get_bat_l(FULLCHG_CAP) * 10;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#undef RET

static enum power_supply_property yeeloong_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static struct power_supply yeeloong_bat = {
	.name = "yeeloong-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = yeeloong_bat_props,
	.num_properties = ARRAY_SIZE(yeeloong_bat_props),
	.get_property = yeeloong_get_bat_props,
};

static int ac_bat_initialized;

static int yeeloong_bat_init(void)
{
	int ret;

	ret = power_supply_register(NULL, &yeeloong_ac);
	if (ret)
		return ret;
	ret = power_supply_register(NULL, &yeeloong_bat);
	if (ret) {
		power_supply_unregister(&yeeloong_ac);
		return ret;
	}
	ac_bat_initialized = 1;

	return 0;
}

static void yeeloong_bat_exit(void)
{
	ac_bat_initialized = 0;

	power_supply_unregister(&yeeloong_ac);
	power_supply_unregister(&yeeloong_bat);
}
/* hwmon subdriver */

#define MIN_FAN_SPEED 0
#define MAX_FAN_SPEED 3

#define get_fan(type) \
	ec_read(REG_FAN_##type)

#define set_fan(type, val) \
	ec_write(REG_FAN_##type, val)

static inline int get_fan_speed_level(void)
{
	return get_fan(SPEED_LEVEL);
}
static inline void set_fan_speed_level(int speed)
{
	set_fan(SPEED_LEVEL, speed);
}

static inline int get_fan_mode(void)
{
	return get_fan(AUTO_MAN_SWITCH);
}
static inline void set_fan_mode(int mode)
{
	set_fan(AUTO_MAN_SWITCH, mode);
}

/*
 * 3 different modes: Full speed(0); manual mode(1); auto mode(2)
 */
static int get_fan_pwm_enable(void)
{
	return (get_fan_mode() == BIT_FAN_AUTO) ? 2 :
		(get_fan_speed_level() == MAX_FAN_SPEED) ? 0 : 1;
}

static void set_fan_pwm_enable(int mode)
{
	set_fan_mode((mode == 2) ? BIT_FAN_AUTO : BIT_FAN_MANUAL);
	if (mode == 0)
		set_fan_speed_level(MAX_FAN_SPEED);
}

static int get_fan_pwm(void)
{
	return get_fan_speed_level();
}

static void set_fan_pwm(int value)
{
	if (get_fan_mode() != BIT_FAN_MANUAL)
		return;

	value = clamp_val(value, MIN_FAN_SPEED, MAX_FAN_SPEED);

	/* We must ensure the fan is on */
	if (value > 0)
		set_fan(CONTROL, ON);

	set_fan_speed_level(value);
}

static inline int get_fan_speed(void)
{
	return ((get_fan(SPEED_HIGH) & 0x0f) << 8) | get_fan(SPEED_LOW);
}

static int get_fan_rpm(void)
{
	return FAN_SPEED_DIVIDER / get_fan_speed();
}

static int get_cpu_temp(void)
{
	return (s8)ec_read(REG_TEMPERATURE_VALUE) * 1000;
}

static int get_cpu_temp_max(void)
{
	return 60 * 1000;
}

static int get_bat_temp_alarm(void)
{
	return !!(get_bat(CHARGE_STATUS) & BIT_BAT_CHARGE_STATUS_OVERTEMP);
}

static ssize_t store_sys_hwmon(void (*set) (int), const char *buf, size_t count)
{
	int ret;
	unsigned long value;

	if (!count)
		return 0;

	ret = strict_strtoul(buf, 10, &value);
	if (ret)
		return ret;

	set(value);

	return count;
}

static ssize_t show_sys_hwmon(int (*get) (void), char *buf)
{
	return sprintf(buf, "%d\n", get());
}

#define CREATE_SENSOR_ATTR(_name, _mode, _set, _get)		\
	static ssize_t show_##_name(struct device *dev,			\
				    struct device_attribute *attr,	\
				    char *buf)				\
	{								\
		return show_sys_hwmon(_set, buf);			\
	}								\
	static ssize_t store_##_name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     const char *buf, size_t count)	\
	{								\
		return store_sys_hwmon(_get, buf, count);		\
	}								\
	static SENSOR_DEVICE_ATTR(_name, _mode, show_##_name, store_##_name, 0);

CREATE_SENSOR_ATTR(fan1_input, S_IRUGO, get_fan_rpm, NULL);
CREATE_SENSOR_ATTR(pwm1, S_IRUGO | S_IWUSR, get_fan_pwm, set_fan_pwm);
CREATE_SENSOR_ATTR(pwm1_enable, S_IRUGO | S_IWUSR, get_fan_pwm_enable,
		set_fan_pwm_enable);
CREATE_SENSOR_ATTR(temp1_input, S_IRUGO, get_cpu_temp, NULL);
CREATE_SENSOR_ATTR(temp1_max, S_IRUGO, get_cpu_temp_max, NULL);
CREATE_SENSOR_ATTR(temp2_input, S_IRUGO, get_bat_temp, NULL);
CREATE_SENSOR_ATTR(temp2_max_alarm, S_IRUGO, get_bat_temp_alarm, NULL);
CREATE_SENSOR_ATTR(curr1_input, S_IRUGO, get_bat_current, NULL);
CREATE_SENSOR_ATTR(in1_input, S_IRUGO, get_bat_voltage, NULL);

static ssize_t
show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "yeeloong\n");
}

static SENSOR_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);

static struct attribute *hwmon_attributes[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm1_enable.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp2_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_name.dev_attr.attr,
	NULL
};

static struct attribute_group hwmon_attribute_group = {
	.attrs = hwmon_attributes
};

static struct device *yeeloong_hwmon_dev;

static int yeeloong_hwmon_init(void)
{
	int ret;

	yeeloong_hwmon_dev = hwmon_device_register(NULL);
	if (IS_ERR(yeeloong_hwmon_dev)) {
		yeeloong_hwmon_dev = NULL;
		return PTR_ERR(yeeloong_hwmon_dev);
	}
	ret = sysfs_create_group(&yeeloong_hwmon_dev->kobj,
				 &hwmon_attribute_group);
	if (ret) {
		hwmon_device_unregister(yeeloong_hwmon_dev);
		yeeloong_hwmon_dev = NULL;
		return ret;
	}
	/* ensure fan is set to auto mode */
	set_fan_pwm_enable(2);

	return 0;
}

static void yeeloong_hwmon_exit(void)
{
	if (yeeloong_hwmon_dev) {
		sysfs_remove_group(&yeeloong_hwmon_dev->kobj,
				   &hwmon_attribute_group);
		hwmon_device_unregister(yeeloong_hwmon_dev);
		yeeloong_hwmon_dev = NULL;
	}
}

/* video output subdriver */

#define LCD	0
#define CRT	1
#define VOD_NUM	2	/* The total number of video output device*/

static struct output_device *vod[VOD_NUM];

static int vor[] = {REG_DISPLAY_LCD, REG_CRT_DETECT};

static int get_vo_dev(struct output_device *od)
{
	int i, dev;

	dev = -1;
	for (i = 0; i < VOD_NUM; i++)
		if (od == vod[i])
			dev = i;

	return dev;
}

static int vo_get_status(int dev)
{
	return ec_read(vor[dev]);
}

static int yeeloong_vo_get_status(struct output_device *od)
{
	int vd;

	vd = get_vo_dev(od);
	if (vd != -1)
		return vo_get_status(vd);

	return -ENODEV;
}

static void vo_set_state(int dev, int state)
{
	int addr;
	unsigned long value;

	switch (dev) {
	case LCD:
		addr = 0x31;
		break;
	case CRT:
		addr = 0x21;
		break;
	default:
		/* return directly if the wrong video output device */
		return;
	}

	outb(addr, 0x3c4);
	value = inb(0x3c5);

	switch (dev) {
	case LCD:
		value |= (state ? 0x03 : 0x02);
		break;
	case CRT:
		if (state)
			clear_bit(7, &value);
		else
			set_bit(7, &value);
		break;
	default:
		break;
	}

	outb(addr, 0x3c4);
	outb(value, 0x3c5);

	if (dev == LCD)
		ec_write(REG_BACKLIGHT_CTRL, state);
}

static int yeeloong_vo_set_state(struct output_device *od)
{
	int vd;

	vd = get_vo_dev(od);
	if (vd == -1)
		return -ENODEV;

	if (vd == CRT && !vo_get_status(vd))
		return 0;

	vo_set_state(vd, !!od->request_state);

	return 0;
}

static struct output_properties vop = {
	.set_state = yeeloong_vo_set_state,
	.get_status = yeeloong_vo_get_status,
};

static int yeeloong_vo_init(void)
{
	int ret, i;
	char dev_name[VOD_NUM][4] = {"LCD", "CRT"};

	/* Register video output device: lcd, crt */
	for (i = 0; i < VOD_NUM; i++) {
		vod[i] = video_output_register(dev_name[i], NULL, NULL, &vop);
		if (IS_ERR(vod[i])) {
			if (i != 0)
				video_output_unregister(vod[i-1]);
			ret = PTR_ERR(vod[i]);
			vod[i] = NULL;
			return ret;
		}
	}
	/* Ensure LCD is on by default */
	vo_set_state(LCD, ON);

	/*
	 * Turn off CRT by default, and will be enabled when the CRT
	 * connectting event reported by SCI
	 */
	vo_set_state(CRT, OFF);

	return 0;
}

static void yeeloong_vo_exit(void)
{
	int i;

	for (i = 0; i < VOD_NUM; i++) {
		if (vod[i]) {
			video_output_unregister(vod[i]);
			vod[i] = NULL;
		}
	}
}

/* lcd subdriver */

struct lcd_device *lcd[VOD_NUM];

static int get_lcd_dev(struct lcd_device *ld)
{
	int i, dev;

	dev = -1;
	for (i = 0; i < VOD_NUM; i++)
		if (ld == lcd[i])
			dev = i;

	return dev;
}

static int yeeloong_lcd_set_power(struct lcd_device *ld, int power)
{
	int dev = get_lcd_dev(ld);

	if (power == FB_BLANK_UNBLANK)
		vo_set_state(dev, ON);
	if (power == FB_BLANK_POWERDOWN)
		vo_set_state(dev, OFF);

	return 0;
}

static int yeeloong_lcd_get_power(struct lcd_device *ld)
{
	return vo_get_status(get_lcd_dev(ld));
}

static struct lcd_ops lcd_ops = {
	.set_power = yeeloong_lcd_set_power,
	.get_power = yeeloong_lcd_get_power,
};

static int yeeloong_lcd_init(void)
{
	int ret, i;
	char dev_name[VOD_NUM][4] = {"LCD", "CRT"};

	/* Register video output device: lcd, crt */
	for (i = 0; i < VOD_NUM; i++) {
		lcd[i] = lcd_device_register(dev_name[i], NULL, NULL, &lcd_ops);
		if (IS_ERR(lcd[i])) {
			if (i != 0)
				lcd_device_unregister(lcd[i-1]);
			ret = PTR_ERR(lcd[i]);
			lcd[i] = NULL;
			return ret;
		}
	}
#if 0
	/* This has been done by the vide output driver */

	/* Ensure LCD is on by default */
	vo_set_state(LCD, ON);

	/*
	 * Turn off CRT by default, and will be enabled when the CRT
	 * connectting event reported by SCI
	 */
	vo_set_state(CRT, OFF);
#endif
	return 0;
}

static void yeeloong_lcd_exit(void)
{
	int i;

	for (i = 0; i < VOD_NUM; i++) {
		if (lcd[i]) {
			lcd_device_unregister(lcd[i]);
			lcd[i] = NULL;
		}
	}
}

/* hotkey subdriver */

static struct input_dev *yeeloong_hotkey_dev;

static atomic_t reboot_flag, sleep_flag;
#define in_sleep() (&sleep_flag)
#define in_reboot() (&reboot_flag)

static const struct key_entry yeeloong_keymap[] = {
	{KE_SW, EVENT_LID, { SW_LID } },
	{KE_KEY, EVENT_CAMERA, { KEY_CAMERA } }, /* Fn + ESC */
	{KE_KEY, EVENT_SLEEP, { KEY_SLEEP } }, /* Fn + F1 */
	{KE_KEY, EVENT_BLACK_SCREEN, { KEY_DISPLAYTOGGLE } }, /* Fn + F2 */
	{KE_KEY, EVENT_DISPLAY_TOGGLE, { KEY_SWITCHVIDEOMODE } }, /* Fn + F3 */
	{KE_KEY, EVENT_AUDIO_MUTE, { KEY_MUTE } }, /* Fn + F4 */
	{KE_KEY, EVENT_WLAN, { KEY_WLAN } }, /* Fn + F5 */
	{KE_KEY, EVENT_DISPLAY_BRIGHTNESS, { KEY_BRIGHTNESSUP } }, /* Fn + up */
	{KE_KEY, EVENT_DISPLAY_BRIGHTNESS, { KEY_BRIGHTNESSDOWN } }, /* Fn + down */
	{KE_KEY, EVENT_AUDIO_VOLUME, { KEY_VOLUMEUP } }, /* Fn + right */
	{KE_KEY, EVENT_AUDIO_VOLUME, { KEY_VOLUMEDOWN } }, /* Fn + left */
	{KE_END, 0}
};

static int is_fake_event(u16 keycode)
{
	switch (keycode) {
	case KEY_SLEEP:
	case SW_LID:
		return atomic_read(in_sleep()) | atomic_read(in_reboot());
		break;
	default:
		break;
	}
	return 0;
}

static struct key_entry *get_event_key_entry(int event, int status)
{
	struct key_entry *ke;
	static int old_brightness_status = -1;
	static int old_volume_status = -1;

	ke = sparse_keymap_entry_from_scancode(yeeloong_hotkey_dev, event);
	if (!ke)
		return NULL;

	switch (event) {
	case EVENT_DISPLAY_BRIGHTNESS:
		/* current status > old one, means up */
		if ((status < old_brightness_status) || (0 == status))
			ke++;
		old_brightness_status = status;
		break;
	case EVENT_AUDIO_VOLUME:
		if ((status < old_volume_status) || (0 == status))
			ke++;
		old_volume_status = status;
		break;
	default:
		break;
	}

	return ke;
}

static int report_lid_switch(int status)
{
	static int old_status;

	/*
	 * LID is a switch button, so, two continuous same status should be
	 * ignored
	 */
	if (old_status != status) {
		input_report_switch(yeeloong_hotkey_dev, SW_LID, !status);
		input_sync(yeeloong_hotkey_dev);
	}
	old_status = status;

	return status;
}

static int crt_detect_handler(int status)
{
	/*
	 * When CRT is inserted, enable its output and disable the LCD output,
	 * otherwise, do reversely.
	 */
	vo_set_state(CRT, status);
	vo_set_state(LCD, !status);

	return status;
}

static int displaytoggle_handler(int status)
{
	/* EC(>=PQ1D26) does this job for us, we can not do it again,
	 * otherwise, the brightness will not resume to the normal level! */
	if (ec_version_before("EC_VER=PQ1D26"))
		vo_set_state(LCD, status);

	return status;
}

static int mypow(int x, int y)
{
	int i, j = x;

	for (i = 1; i < y; i++)
		j *= j;

	return j;
}

static int switchvideomode_handler(int status)
{
	/* Default status: CRT|LCD = 0|1 = 1 */
	static int bin_state = 1;
	int i;

	/*
	 * Only enable switch video output button
	 * when CRT is connected
	 */
	if (!vo_get_status(CRT))
		return 0;
	/*
	 * 2. no CRT connected: LCD on, CRT off
	 * 3. BOTH on
	 * 0. BOTH off
	 * 1. LCD off, CRT on
	 */

	bin_state++;
	if (bin_state > mypow(2, VOD_NUM) - 1)
		bin_state = 0;

	for (i = 0; i < VOD_NUM; i++)
		vo_set_state(i, bin_state & (1 << i));

	return bin_state;
}

static int camera_handler(int status)
{
	int value;

	value = ec_read(REG_CAMERA_CONTROL);
	ec_write(REG_CAMERA_CONTROL, value | (1 << 1));

	return status;
}

static int usb2_handler(int status)
{
	pr_emerg("USB2 Over Current occurred\n");

	return status;
}

static int usb0_handler(int status)
{
	pr_emerg("USB0 Over Current occurred\n");

	return status;
}

static int ac_bat_handler(int status)
{
	if (ac_bat_initialized) {
		power_supply_changed(&yeeloong_ac);
		power_supply_changed(&yeeloong_bat);
	}

	return status;
}

struct sci_event {
	int reg;
	sci_handler handler;
};

static const struct sci_event se[] = {
	[EVENT_AC_BAT] = {0, ac_bat_handler},
	[EVENT_AUDIO_MUTE] = {REG_AUDIO_MUTE, NULL},
	[EVENT_AUDIO_VOLUME] = {REG_AUDIO_VOLUME, NULL},
	[EVENT_CRT_DETECT] = {REG_CRT_DETECT, crt_detect_handler},
	[EVENT_CAMERA] = {REG_CAMERA_STATUS, camera_handler},
	[EVENT_BLACK_SCREEN] = {REG_DISPLAY_LCD, displaytoggle_handler},
	[EVENT_DISPLAY_BRIGHTNESS] = {REG_DISPLAY_BRIGHTNESS, NULL},
	[EVENT_LID] = {REG_LID_DETECT, NULL},
	[EVENT_DISPLAY_TOGGLE] = {0, switchvideomode_handler},
	[EVENT_USB_OC0] = {REG_USB2_FLAG, usb0_handler},
	[EVENT_USB_OC2] = {REG_USB2_FLAG, usb2_handler},
};

static void do_event_action(int event)
{
	int status;
	struct key_entry *ke;
	struct sci_event *sep;

	sep = (struct sci_event *)&se[event];

	if (sep->reg != 0)
		status = ec_read(sep->reg);

	if (sep->handler != NULL)
		status = sep->handler(status);

	pr_debug("%s: event: %d status: %d\n", __func__, event, status);

	/* Report current key to user-space */
	ke = get_event_key_entry(event, status);

	/*
	 * Ignore the LID and SLEEP event when we are already in sleep or
	 * reboot state, this will avoid the recursive pm operations. but note:
	 * the report_lid_switch() called in arch/mips/loongson/lemote-2f/pm.c
	 * is necessary, because it is used to wake the system from sleep
	 * state. In the future, perhaps SW_LID should works like SLEEP, no
	 * need to function as a SWITCH, just report the state when the LID is
	 * closed is enough, this event can tell the software to "SLEEP", no
	 * need to tell the softwares when we are resuming from "SLEEP".
	 */
	if (ke && !is_fake_event(ke->keycode)) {
		if (ke->keycode == SW_LID)
			report_lid_switch(status);
		else
			sparse_keymap_report_entry(yeeloong_hotkey_dev, ke, 1,
					true);
	}
}

/*
 * SCI(system control interrupt) main interrupt routine
 *
 * We will do the query and get event number together so the interrupt routine
 * should be longer than 120us now at least 3ms elpase for it.
 */
static irqreturn_t sci_irq_handler(int irq, void *dev_id)
{
	int ret, event;

	if (SCI_IRQ_NUM != irq)
		return IRQ_NONE;

	/* Query the event number */
	ret = ec_query_event_num();
	if (ret < 0)
		return IRQ_NONE;

	event = ec_get_event_num();
	if (event < EVENT_START || event > EVENT_END)
		return IRQ_NONE;

	/* Execute corresponding actions */
	do_event_action(event);

	return IRQ_HANDLED;
}

/*
 * Config and init some msr and gpio register properly.
 */
static int sci_irq_init(void)
{
	u32 hi, lo;
	u32 gpio_base;
	unsigned long flags;
	int ret;

	/* Get gpio base */
	_rdmsr(DIVIL_MSR_REG(DIVIL_LBAR_GPIO), &hi, &lo);
	gpio_base = lo & 0xff00;

	/* Filter the former kb3310 interrupt for security */
	ret = ec_query_event_num();
	if (ret)
		return ret;

	/* For filtering next number interrupt */
	udelay(10000);

	/* Set gpio native registers and msrs for GPIO27 SCI EVENT PIN
	 * gpio :
	 *      input, pull-up, no-invert, event-count and value 0,
	 *      no-filter, no edge mode
	 *      gpio27 map to Virtual gpio0
	 * msr :
	 *      no primary and lpc
	 *      Unrestricted Z input to IG10 from Virtual gpio 0.
	 */
	local_irq_save(flags);
	_rdmsr(0x80000024, &hi, &lo);
	lo &= ~(1 << 10);
	_wrmsr(0x80000024, hi, lo);
	_rdmsr(0x80000025, &hi, &lo);
	lo &= ~(1 << 10);
	_wrmsr(0x80000025, hi, lo);
	_rdmsr(0x80000023, &hi, &lo);
	lo |= (0x0a << 0);
	_wrmsr(0x80000023, hi, lo);
	local_irq_restore(flags);

	/* Set gpio27 as sci interrupt
	 *
	 * input, pull-up, no-fliter, no-negedge, invert
	 * the sci event is just about 120us
	 */
	asm(".set noreorder\n");
	/*  input enable */
	outl(0x00000800, (gpio_base | 0xA0));
	/*  revert the input */
	outl(0x00000800, (gpio_base | 0xA4));
	/*  event-int enable */
	outl(0x00000800, (gpio_base | 0xB8));
	asm(".set reorder\n");

	return 0;
}

static int notify_reboot(struct notifier_block *nb, unsigned long event, void *buf)
{
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		atomic_set(in_reboot(), 1);
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static int notify_pm(struct notifier_block *nb, unsigned long event, void *buf)
{
	switch (event) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		atomic_inc(in_sleep());
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
	case PM_RESTORE_PREPARE:	/* do we need this ?? */
		atomic_dec(in_sleep());
		break;
	default:
		return NOTIFY_DONE;
	}

	pr_debug("%s: event = %lu, in_sleep() = %d\n", __func__, event,
			atomic_read(in_sleep()));

	return NOTIFY_OK;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = notify_reboot,
};

static struct notifier_block pm_notifier = {
	.notifier_call = notify_pm,
};

static int yeeloong_hotkey_init(void)
{
	int ret = 0;

	ret = register_reboot_notifier(&reboot_notifier);
	if (ret) {
		pr_err("Can't register reboot notifier\n");
		goto end;
	}

	ret = register_pm_notifier(&pm_notifier);
	if (ret) {
		pr_err("Can't register pm notifier\n");
		goto free_reboot_notifier;
	}

	ret = sci_irq_init();
	if (ret) {
		pr_err("Can't init SCI interrupt\n");
		goto free_pm_notifier;
	}

	ret = request_threaded_irq(SCI_IRQ_NUM, NULL, &sci_irq_handler,
			IRQF_ONESHOT, "sci", NULL);
	if (ret) {
		pr_err("Can't thread SCI interrupt handler\n");
		goto free_pm_notifier;
	}

	yeeloong_hotkey_dev = input_allocate_device();

	if (!yeeloong_hotkey_dev) {
		ret = -ENOMEM;
		goto free_irq;
	}

	yeeloong_hotkey_dev->name = "HotKeys";
	yeeloong_hotkey_dev->phys = "button/input0";
	yeeloong_hotkey_dev->id.bustype = BUS_HOST;
	yeeloong_hotkey_dev->dev.parent = NULL;

	ret = sparse_keymap_setup(yeeloong_hotkey_dev, yeeloong_keymap, NULL);
	if (ret) {
		pr_err("Failed to setup input device keymap\n");
		goto free_dev;
	}

	ret = input_register_device(yeeloong_hotkey_dev);
	if (ret)
		goto free_keymap;

	/* Update the current status of LID */
	report_lid_switch(ON);

#ifdef CONFIG_LOONGSON_SUSPEND
	/* Install the real yeeloong_report_lid_status for pm.c */
	yeeloong_report_lid_status = report_lid_switch;
#endif
	return 0;

free_keymap:
	sparse_keymap_free(yeeloong_hotkey_dev);
free_dev:
	input_free_device(yeeloong_hotkey_dev);
free_irq:
	free_irq(SCI_IRQ_NUM, NULL);
free_pm_notifier:
	unregister_pm_notifier(&pm_notifier);
free_reboot_notifier:
	unregister_reboot_notifier(&reboot_notifier);
end:
	return ret;
}

static void yeeloong_hotkey_exit(void)
{
	/* Free irq */
	free_irq(SCI_IRQ_NUM, NULL);

#ifdef CONFIG_LOONGSON_SUSPEND
	/* Uninstall yeeloong_report_lid_status for pm.c */
	if (yeeloong_report_lid_status == report_lid_switch)
		yeeloong_report_lid_status = NULL;
#endif

	if (yeeloong_hotkey_dev) {
		sparse_keymap_free(yeeloong_hotkey_dev);
		input_unregister_device(yeeloong_hotkey_dev);
		yeeloong_hotkey_dev = NULL;
	}
}

#ifdef CONFIG_PM
static void usb_ports_set(int status)
{
	status = !!status;

	ec_write(REG_USB0_FLAG, status);
	ec_write(REG_USB1_FLAG, status);
	ec_write(REG_USB2_FLAG, status);
}

static int yeeloong_suspend(struct device *dev)

{
	if (ec_version_before("EC_VER=PQ1D27"))
		vo_set_state(LCD, OFF);
	vo_set_state(CRT, OFF);
	usb_ports_set(OFF);

	return 0;
}

static int yeeloong_resume(struct device *dev)
{
	int ret;

	if (ec_version_before("EC_VER=PQ1D27"))
		vo_set_state(LCD, ON);
	vo_set_state(CRT, ON);
	usb_ports_set(ON);

	ret = sci_irq_init();
	if (ret)
		return -EFAULT;

	return 0;
}

static const SIMPLE_DEV_PM_OPS(yeeloong_pm_ops, yeeloong_suspend,
	yeeloong_resume);
#endif

static struct platform_device_id platform_device_ids[] = {
	{
		.name = "yeeloong_laptop",
	},
	{}
};

MODULE_DEVICE_TABLE(platform, platform_device_ids);

static struct platform_driver platform_driver = {
	.driver = {
		.name = "yeeloong_laptop",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &yeeloong_pm_ops,
#endif
	},
	.id_table = platform_device_ids,
};

static int __init yeeloong_init(void)
{
	int ret;

	pr_info("YeeLoong Laptop platform specific driver loaded.\n");

	/* Register platform stuff */
	ret = platform_driver_register(&platform_driver);
	if (ret) {
		pr_err("Failed to register YeeLoong platform driver.\n");
		return ret;
	}

#define yeeloong_init_drv(drv, alias) do {			\
	pr_info("Registered YeeLoong " alias " driver.\n");	\
	ret = yeeloong_ ## drv ## _init();			\
	if (ret) {						\
		pr_err("Failed to register YeeLoong " alias " driver.\n");	\
		yeeloong_ ## drv ## _exit();			\
		return ret;					\
	}							\
} while (0)

	yeeloong_init_drv(backlight, "backlight");
	yeeloong_init_drv(bat, "battery and AC");
	yeeloong_init_drv(hwmon, "hardware monitor");
	yeeloong_init_drv(vo, "video output");
	yeeloong_init_drv(lcd, "lcd output");
	/* yeeloong_init_drv(hotkey, "hotkey input"); */

	return 0;
}

static void __exit yeeloong_exit(void)
{
	yeeloong_hotkey_exit();
	yeeloong_lcd_exit();
	yeeloong_vo_exit();
	yeeloong_hwmon_exit();
	yeeloong_bat_exit();
	yeeloong_backlight_exit();
	platform_driver_unregister(&platform_driver);

	pr_info("YeeLoong platform specific driver unloaded.\n");
}

module_init(yeeloong_init);
module_exit(yeeloong_exit);

MODULE_AUTHOR("Wu Zhangjin <wuzhangjin@gmail.com>; Liu Junliang <liujl@lemote.com>");
MODULE_DESCRIPTION("YeeLoong laptop driver");
MODULE_LICENSE("GPL");
