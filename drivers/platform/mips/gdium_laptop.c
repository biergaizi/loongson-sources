/*
 * gdium_laptop  --  Gdium laptop extras
 *
 * Arnaud Patard <apatard@mandriva.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/gpio.h>

/* For input device */
#define SCAN_INTERVAL		150

/* For battery status */
#define BAT_SCAN_INTERVAL	500

#define EC_FIRM_VERSION		0

#if CONFIG_GDIUM_VERSION > 2
#define EC_REG_BASE		1
#else
#define EC_REG_BASE		0
#endif

#define EC_STATUS		(EC_REG_BASE+0)
#define EC_STATUS_LID		(1<<0)
#define EC_STATUS_PWRBUT	(1<<1)
#define EC_STATUS_BATID		(1<<2)		/* this bit has no real meaning on v2.         */
						/* Same as EC_STATUS_ADAPT                     */
						/* but on v3 it's BATID which mean bat present */
#define EC_STATUS_SYS_POWER	(1<<3)
#define EC_STATUS_WLAN		(1<<4)
#define EC_STATUS_ADAPT		(1<<5)

#define EC_CTRL			(EC_REG_BASE+1)
#define EC_CTRL_DDR_CLK		(1<<0)
#define EC_CTRL_CHARGE_LED	(1<<1)
#define EC_CTRL_BEEP		(1<<2)
#define EC_CTRL_SUSB		(1<<3)	/* memory power */
#define EC_CTRL_TRICKLE		(1<<4)
#define EC_CTRL_WLAN_EN		(1<<5)
#define EC_CTRL_SUSC		(1<<6) /* main power */
#define EC_CTRL_CHARGE_EN	(1<<7)

#define EC_BAT_LOW		(EC_REG_BASE+2)
#define EC_BAT_HIGH		(EC_REG_BASE+3)

#define EC_SIGN			(EC_REG_BASE+4)
#define EC_SIGN_OS		0xAE /* write 0xae to control pm stuff */
#define EC_SIGN_EC		0x00 /* write 0x00 to let the st7 manage pm stuff */

#if 0
#define EC_TEST			(EC_REG_BASE+5) /* Depending on firmware version this register */
						/* may be the programmation register so don't play */
						/* with it */
#endif

#define BAT_VOLT_PRESENT	500000	/* Min voltage to consider battery present uV */
#define BAT_MIN			7000000	/* Min battery voltage in uV */
#define BAT_MIN_MV		7000	/* Min battery voltage in mV */
#define BAT_TRICKLE_EN		8000000	/* Charging at 1.4A before  8.0V and then charging at 0.25A */
#define BAT_MAX			7950000	/* Max battery voltage ~8V in V */
#define BAT_MAX_MV		7950	/* Max battery voltage ~8V in V */
#define BAT_READ_ERROR		300000	/* battery read error of 0.3V */
#define BAT_READ_ERROR_MV	300	/* battery read error of 0.3V */

#define SM502_WLAN_ON		(224+16)/* SM502 GPIO16 may be used on gdium v2 (v3?) as wlan_on */
					/* when R422 is connected */

static unsigned char verbose;
static unsigned char gpio16;
static unsigned char ec;
module_param(verbose, byte, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(verbose, "Add some debugging messages");
module_param(gpio16, byte, S_IRUGO);
MODULE_PARM_DESC(gpio16, "Enable wlan_on signal on SM502");
module_param(ec, byte, S_IRUGO);
MODULE_PARM_DESC(ec, "Let the ST7 handle the battery (default OS)");

struct gdium_laptop_data {
	struct i2c_client		*client;
	struct input_polled_dev		*input_polldev;
	struct dentry			*debugfs;
	struct mutex			mutex;
	struct platform_device		*bat_pdev;
	struct power_supply		gdium_ac;
	struct power_supply		gdium_battery;
	struct workqueue_struct		*workqueue;
	struct delayed_work		work;
	char				charge_cmd;
	/* important registers value */
	char				status;
	char				ctrl;
	/* mV */
	int				battery_level;
	char				version;
};

/**********************************************************************/
/* Low level I2C functions                                            */
/* All are supposed to be called with mutex held                      */
/**********************************************************************/
/*
 * Return battery voltage in mV
 * >= 0 battery voltage
 * < 0 error
 */
static s32 ec_read_battery(struct i2c_client *client)
{
	unsigned char bat_low, bat_high;
	s32 data;
	unsigned int ret;

	/*
	 * a = battery high
	 * b = battery low
	 * bat = a << 2 | b & 0x03;
	 * battery voltage = (bat / 1024) * 5 * 2
	 */
	data = i2c_smbus_read_byte_data(client, EC_BAT_LOW);
	if (data < 0) {
		dev_err(&client->dev, "ec_read_bat: read bat_low failed\n");
		return data;
	}
	bat_low = data & 0xff;
	if (verbose)
		dev_info(&client->dev, "bat_low %x\n", bat_low);

	data = i2c_smbus_read_byte_data(client, EC_BAT_HIGH);
	if (data < 0) {
		dev_err(&client->dev, "ec_read_bat: read bat_high failed\n");
		return data;
	}
	bat_high = data & 0xff;
	if (verbose)
		dev_info(&client->dev, "bat_high %x\n", bat_high);

	ret = (bat_high << 2) | (bat_low & 3);
	/*
	 * mV
	 */
	ret = (ret * 5 * 2) * 1000 / 1024;

	return ret;
}

static s32 ec_read_version(struct i2c_client *client)
{
#if CONFIG_GDIUM_VERSION > 2
	return i2c_smbus_read_byte_data(client, EC_FIRM_VERSION);
#else
	return 0;
#endif
}

static s32 ec_read_status(struct i2c_client *client)
{
	return i2c_smbus_read_byte_data(client, EC_STATUS);
}

static s32 ec_read_ctrl(struct i2c_client *client)
{
	return i2c_smbus_read_byte_data(client, EC_CTRL);
}

static s32 ec_write_ctrl(struct i2c_client *client, unsigned char newvalue)
{
	return i2c_smbus_write_byte_data(client, EC_CTRL, newvalue);
}

static s32 ec_read_sign(struct i2c_client *client)
{
	return i2c_smbus_read_byte_data(client, EC_SIGN);
}

static s32 ec_write_sign(struct i2c_client *client, unsigned char sign)
{
	unsigned char value;
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, EC_SIGN, sign);
	if (ret < 0) {
		dev_err(&client->dev, "ec_set_control: write failed\n");
		return ret;
	}

	value = ec_read_sign(client);
	if (value != sign) {
		dev_err(&client->dev, "Failed to set control to %s\n",
				sign == EC_SIGN_OS ? "OS" : "EC");
		return -EIO;
	}

	return 0;
}

#if 0
static int ec_power_off(struct i2c_client *client)
{
	char value;
	int ret;

	value = ec_read_ctrl(client);
	if (value < 0) {
		dev_err(&client->dev, "ec_power_off: read failed\n");
		return value;
	}
	value &= ~(EC_CTRL_SUSB | EC_CTRL_SUSC);
	ret = ec_write_ctrl(client, value);
	if (ret < 0) {
		dev_err(&client->dev, "ec_power_off: write failed\n");
		return ret;
	}

	return 0;
}
#endif

static s32 ec_wlan_status(struct i2c_client *client)
{
	s32 value;

	value = ec_read_ctrl(client);
	if (value < 0)
		return value;

	return (value & EC_CTRL_WLAN_EN) ? 1 : 0;
}

static s32 ec_wlan_en(struct i2c_client *client, int on)
{
	s32 value;

	value = ec_read_ctrl(client);
	if (value < 0)
		return value;

	value &= ~EC_CTRL_WLAN_EN;
	if (on)
		value |= EC_CTRL_WLAN_EN;

	return ec_write_ctrl(client, value&0xff);
}

#if 0
static s32 ec_led_status(struct i2c_client *client)
{
	s32 value;

	value = ec_read_ctrl(client);
	if (value < 0)
		return value;

	return (value & EC_CTRL_CHARGE_LED) ? 1 : 0;
}
#endif

/* Changing the charging led status has never worked */
static s32 ec_led_en(struct i2c_client *client, int on)
{
#if 0
	s32 value;

	value = ec_read_ctrl(client);
	if (value < 0)
		return value;

	value &= ~EC_CTRL_CHARGE_LED;
	if (on)
		value |= EC_CTRL_CHARGE_LED;
	return ec_write_ctrl(client, value&0xff);
#else
	return 0;
#endif
}

static s32 ec_charge_en(struct i2c_client *client, int on, int trickle)
{
	s32 value;
	s32 set = 0;

	value = ec_read_ctrl(client);
	if (value < 0)
		return value;

	if (on)
		set |= EC_CTRL_CHARGE_EN;
	if (trickle)
		set |= EC_CTRL_TRICKLE;

	/* Be clever : don't change values if you don't need to */
	if ((value & (EC_CTRL_CHARGE_EN | EC_CTRL_TRICKLE)) == set)
		return 0;

	value &= ~(EC_CTRL_CHARGE_EN | EC_CTRL_TRICKLE);
	value |= set;
	ec_led_en(client, on);
	return ec_write_ctrl(client, (unsigned char)(value&0xff));

}

/**********************************************************************/
/* Input functions                                                    */
/**********************************************************************/
struct gdium_keys {
	int last_state;
	int key_code;
	int mask;
	int type;
};

static struct gdium_keys gkeys[] = {
	{
		.key_code	= KEY_WLAN,
		.mask		= EC_STATUS_WLAN,
		.type		= EV_KEY,
	},
	{
		.key_code	= KEY_POWER,
		.mask		= EC_STATUS_PWRBUT,
		.type		= EV_KEY, /*EV_PWR,*/
	},
	{
		.key_code	= SW_LID,
		.mask		= EC_STATUS_LID,
		.type		= EV_SW,
	},
};

static void gdium_laptop_keys_poll(struct input_polled_dev *dev)
{
	int state, i;
	struct gdium_laptop_data *data = dev->private;
	struct i2c_client *client = data->client;
	struct input_dev *input = dev->input;
	s32 status;

	mutex_lock(&data->mutex);
	status = ec_read_status(client);
	mutex_unlock(&data->mutex);

	if (status < 0) {
		/*
		 * Don't know exactly  which version of the firmware
		 * has this bug but when the power button is pressed
		 * there are i2c read errors :(
		 */
		if ((data->version >= 0x13) && !gkeys[1].last_state) {
			input_event(input, EV_KEY, KEY_POWER, 1);
			input_sync(input);
			gkeys[1].last_state = 1;
		}
		return;
	}

	for (i = 0; i < ARRAY_SIZE(gkeys); i++) {
		state = status & gkeys[i].mask;
		if (state != gkeys[i].last_state) {
			gkeys[i].last_state = state;
			/* for power key, we want power & key press/release event */
			if (gkeys[i].type == EV_PWR) {
				input_event(input, EV_KEY, gkeys[i].key_code, !!state);
				input_sync(input);
			}
			/* Disable wifi on key press but not key release */
			/*
			 * On firmware >= 0x13 the EC_STATUS_WLAN has it's
			 * original meaning of Wifi status and no more the
			 * wifi button status so we have to ignore the event
			 * on theses versions
			 */
			if (state && (gkeys[i].key_code == KEY_WLAN) && (data->version < 0x13)) {
				mutex_lock(&data->mutex);
				ec_wlan_en(client, !ec_wlan_status(client));
				if (gpio16)
					gpio_set_value(SM502_WLAN_ON, !ec_wlan_status(client));
				mutex_unlock(&data->mutex);
			}

			input_event(input, gkeys[i].type, gkeys[i].key_code, !!state);
			input_sync(input);
		}
	}
}

static int gdium_laptop_input_init(struct gdium_laptop_data *data)
{
	struct i2c_client *client = data->client;
	struct input_dev *input;
	int ret, i;

	data->input_polldev = input_allocate_polled_device();
	if (!data->input_polldev) {
		ret = -ENOMEM;
		goto err;
	}

	input = data->input_polldev->input;
	input->evbit[0] = BIT(EV_KEY) | BIT_MASK(EV_PWR) | BIT_MASK(EV_SW);
	data->input_polldev->poll = gdium_laptop_keys_poll;
	data->input_polldev->poll_interval = SCAN_INTERVAL;
	data->input_polldev->private = data;
	input->name = "gdium-keys";
	input->dev.parent = &client->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	for (i = 0; i < ARRAY_SIZE(gkeys); i++)
		input_set_capability(input, gkeys[i].type, gkeys[i].key_code);

	ret = input_register_polled_device(data->input_polldev);
	if (ret) {
		dev_err(&client->dev, "Unable to register button device\n");
		goto err_poll_dev;
	}

	return 0;

err_poll_dev:
	input_free_polled_device(data->input_polldev);
err:
	return ret;
}

static void gdium_laptop_input_exit(struct gdium_laptop_data *data)
{
	input_unregister_polled_device(data->input_polldev);
	input_free_polled_device(data->input_polldev);
}

/**********************************************************************/
/* Battery management                                                 */
/**********************************************************************/
static int gdium_ac_get_props(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	char status;
	struct gdium_laptop_data *data = container_of(psy, struct gdium_laptop_data, gdium_ac);
	int ret = 0;

	if (!data) {
		pr_err("gdium-ac: gdium_laptop_data not found\n");
		return -EINVAL;
	}

	status = data->status;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(status & EC_STATUS_ADAPT);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#undef RET
#define RET (val->intval)

static int gdium_battery_get_props(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	char status, ctrl;
	struct gdium_laptop_data *data = container_of(psy, struct gdium_laptop_data, gdium_battery);
	int percentage_capacity = 0, charge_now = 0, time_to_empty = 0;
	int ret = 0, tmp;

	if (!data) {
		pr_err("gdium-battery: gdium_laptop_data not found\n");
		return -EINVAL;
	}

	status = data->status;
	ctrl   = data->ctrl;
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		/* uAh */
		RET = 5000000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		/* This formula is gotten by gnuplot with the statistic data */
		time_to_empty = (data->battery_level - BAT_MIN_MV + BAT_READ_ERROR_MV) * 113 - 29870;
		if (psp == POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW) {
			/* seconds */
			RET = time_to_empty / 10;
			break;
		}
		/* fall through */
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY: {
		tmp = data->battery_level * 1000;
		/* > BAT_MIN to avoid negative values */
		percentage_capacity = 0;
		if ((status & EC_STATUS_BATID) && (tmp > BAT_MIN))
			percentage_capacity = (tmp-BAT_MIN)*100/(BAT_MAX-BAT_MIN);

		if (percentage_capacity > 100)
			percentage_capacity = 100;

		if (psp == POWER_SUPPLY_PROP_CAPACITY) {
			RET = percentage_capacity;
			break;
		}
		charge_now = 50000 * percentage_capacity;
		if (psp == POWER_SUPPLY_PROP_CHARGE_NOW) {
			/* uAh */
			RET = charge_now;
			break;
		}
	}	/* fall through */
	case POWER_SUPPLY_PROP_STATUS: {
		if (status & EC_STATUS_ADAPT)
			if (ctrl & EC_CTRL_CHARGE_EN)
				RET = POWER_SUPPLY_STATUS_CHARGING;
			else
				RET = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			RET = POWER_SUPPLY_STATUS_DISCHARGING;

		if (psp == POWER_SUPPLY_PROP_STATUS)
			break;
		/* mAh -> µA */
		switch (RET) {
		case POWER_SUPPLY_STATUS_CHARGING:
			RET = -(data->charge_cmd == 2) ? 1400000 : 250000;
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			RET = charge_now / time_to_empty * 36000;
			break;
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
		default:
			RET = 0;
			break;
		}
	} break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		RET = BAT_MAX+BAT_READ_ERROR;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		RET = BAT_MIN-BAT_READ_ERROR;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* mV -> uV */
		RET = data->battery_level * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
#if CONFIG_GDIUM_VERSION > 2
		RET = !!(status & EC_STATUS_BATID);
#else
		RET = !!(data->battery_level > BAT_VOLT_PRESENT);
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		tmp = data->battery_level * 1000;
		RET = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		if (status & EC_STATUS_BATID) {
			if (tmp >= BAT_MAX) {
				RET = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
				if (tmp >= BAT_MAX+BAT_READ_ERROR)
					RET = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
			} else if (tmp <= BAT_MIN+BAT_READ_ERROR) {
				RET = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
				if (tmp <= BAT_MIN)
					RET = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
			} else
				RET = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (ctrl & EC_CTRL_TRICKLE)
			RET = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else if (ctrl & EC_CTRL_CHARGE_EN)
			RET = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			RET = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		/* 1.4A ? */
		RET = 1400000;
		break;
	default:
		break;
	}

	return ret;
}
#undef RET

static enum power_supply_property gdium_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property gdium_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static void gdium_laptop_battery_work(struct work_struct *work)
{
	struct gdium_laptop_data *data = container_of(work, struct gdium_laptop_data, work.work);
	struct i2c_client *client;
	int ret;
	char old_status, old_charge_cmd;
	char present;
	s32 status;

	mutex_lock(&data->mutex);
	client	= data->client;
	status	= ec_read_status(client);
	ret	= ec_read_battery(client);

	if ((status < 0) || (ret < 0))
		goto i2c_read_error;

	old_status = data->status;
	old_charge_cmd = data->charge_cmd;
	data->status = status;

	/*
	 * Charge only if :
	 * - battery present
	 * - ac adapter plugged in
	 * - battery not fully charged
	 */
#if CONFIG_GDIUM_VERSION > 2
	present = !!(data->status & EC_STATUS_BATID);
#else
	present = !!(ret > BAT_VOLT_PRESENT);
#endif
	data->battery_level = 0;
	if (present) {
		data->battery_level = (unsigned int)ret;
		if (data->status & EC_STATUS_ADAPT)
			data->battery_level -= BAT_READ_ERROR_MV;
	}

	data->charge_cmd = 0;
	if ((data->status & EC_STATUS_ADAPT) && present && (data->battery_level <= BAT_MAX_MV))
		data->charge_cmd = (ret < BAT_TRICKLE_EN) ? 2 : 3;

	ec_charge_en(client, (data->charge_cmd >> 1) & 1, data->charge_cmd & 1);

	/*
	 * data->ctrl must be set _after_ calling ec_charge_en as this will change the
	 * control register content
	 */
	data->ctrl = ec_read_ctrl(client);

	if ((data->status & EC_STATUS_ADAPT) != (old_status & EC_STATUS_ADAPT)) {
		power_supply_changed(&data->gdium_ac);
		/* Send charging/discharging state change */
		power_supply_changed(&data->gdium_battery);
	} else if ((data->status & EC_STATUS_ADAPT) &&
			((old_charge_cmd&2) != (data->charge_cmd&2)))
		power_supply_changed(&data->gdium_battery);

i2c_read_error:
	mutex_unlock(&data->mutex);
	queue_delayed_work(data->workqueue, &data->work, msecs_to_jiffies(BAT_SCAN_INTERVAL));
}

static int gdium_laptop_battery_init(struct gdium_laptop_data *data)
{
	int ret;

	data->bat_pdev = platform_device_register_simple("gdium-battery", 0, NULL, 0);
	if (IS_ERR(data->bat_pdev))
		return PTR_ERR(data->bat_pdev);

	data->gdium_battery.name		= data->bat_pdev->name;
	data->gdium_battery.properties		= gdium_battery_props;
	data->gdium_battery.num_properties	= ARRAY_SIZE(gdium_battery_props);
	data->gdium_battery.get_property	= gdium_battery_get_props;
	data->gdium_battery.use_for_apm		= 1;

	ret = power_supply_register(&data->bat_pdev->dev, &data->gdium_battery);
	if (ret)
		goto err_platform;

	data->gdium_ac.name			= "gdium-ac";
	data->gdium_ac.type			= POWER_SUPPLY_TYPE_MAINS;
	data->gdium_ac.properties		= gdium_ac_props;
	data->gdium_ac.num_properties		= ARRAY_SIZE(gdium_ac_props);
	data->gdium_ac.get_property		= gdium_ac_get_props;
/*	data->gdium_ac.use_for_apm_ac		= 1,	*/

	ret = power_supply_register(&data->bat_pdev->dev, &data->gdium_ac);
	if (ret)
		goto err_battery;

	if (!ec) {
		INIT_DELAYED_WORK(&data->work, gdium_laptop_battery_work);
		data->workqueue = create_singlethread_workqueue("gdium-battery-work");
		if (!data->workqueue) {
			ret = -ESRCH;
			goto err_work;
		}
		queue_delayed_work(data->workqueue, &data->work, msecs_to_jiffies(BAT_SCAN_INTERVAL));
	}

	return 0;

err_work:
err_battery:
	power_supply_unregister(&data->gdium_battery);
err_platform:
	platform_device_unregister(data->bat_pdev);

	return ret;
}
static void gdium_laptop_battery_exit(struct gdium_laptop_data *data)
{
	if (!ec) {
		cancel_rearming_delayed_workqueue(data->workqueue, &data->work);
		destroy_workqueue(data->workqueue);
	}
	power_supply_unregister(&data->gdium_battery);
	power_supply_unregister(&data->gdium_ac);
	platform_device_unregister(data->bat_pdev);
}

/* Debug fs */
static int gdium_laptop_regs_show(struct seq_file *s, void *p)
{
	struct gdium_laptop_data *data = s->private;
	struct i2c_client *client = data->client;

	mutex_lock(&data->mutex);
	seq_printf(s, "Version    : 0x%02x\n", (unsigned char)ec_read_version(client));
	seq_printf(s, "Status     : 0x%02x\n", (unsigned char)ec_read_status(client));
	seq_printf(s, "Ctrl       : 0x%02x\n", (unsigned char)ec_read_ctrl(client));
	seq_printf(s, "Sign       : 0x%02x\n", (unsigned char)ec_read_sign(client));
	seq_printf(s, "Bat Lo     : 0x%02x\n", (unsigned char)i2c_smbus_read_byte_data(client, EC_BAT_LOW));
	seq_printf(s, "Bat Hi     : 0x%02x\n", (unsigned char)i2c_smbus_read_byte_data(client, EC_BAT_HIGH));
	seq_printf(s, "Battery    : %d uV\n",  (unsigned int)ec_read_battery(client) * 1000);
	seq_printf(s, "Charge cmd : %s %s\n", data->charge_cmd & 2 ? "C" : " ", data->charge_cmd & 1 ? "T" : " ");

	mutex_unlock(&data->mutex);
	return 0;
}

static int gdium_laptop_regs_open(struct inode *inode,
					 struct file *file)
{
	return single_open(file, gdium_laptop_regs_show, inode->i_private);
}

static const struct file_operations gdium_laptop_regs_fops = {
	.open		= gdium_laptop_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};


static int gdium_laptop_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gdium_laptop_data *data;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: no smbus_byte support !\n", __func__);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct gdium_laptop_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;
	mutex_init(&data->mutex);

	ret = ec_read_version(client);
	if (ret < 0)
		goto err_alloc;

	data->version = (unsigned char)ret;

	ret = gdium_laptop_input_init(data);
	if (ret)
		goto err_alloc;

	ret = gdium_laptop_battery_init(data);
	if (ret)
		goto err_input;


	if (!ec) {
		ret = ec_write_sign(client, EC_SIGN_OS);
		if (ret)
			goto err_sign;
	}

	if (gpio16) {
		ret = gpio_request(SM502_WLAN_ON, "wlan-on");
		if (ret < 0)
			goto err_sign;
		gpio_set_value(SM502_WLAN_ON, ec_wlan_status(client));
		gpio_direction_output(SM502_WLAN_ON, 1);
	}

	dev_info(&client->dev, "Found firmware 0x%02x\n", data->version);
	data->debugfs = debugfs_create_file("gdium_laptop", S_IFREG | S_IRUGO,
				NULL, data, &gdium_laptop_regs_fops);

	return 0;

err_sign:
	gdium_laptop_battery_exit(data);
err_input:
	gdium_laptop_input_exit(data);
err_alloc:
	kfree(data);
	return ret;
}

static int gdium_laptop_remove(struct i2c_client *client)
{
	struct gdium_laptop_data *data = i2c_get_clientdata(client);

	if (gpio16)
		gpio_free(SM502_WLAN_ON);
	ec_write_sign(client, EC_SIGN_EC);
	if (data->debugfs)
		debugfs_remove(data->debugfs);

	gdium_laptop_battery_exit(data);
	gdium_laptop_input_exit(data);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int gdium_laptop_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct gdium_laptop_data *data = i2c_get_clientdata(client);

	if (!ec)
		cancel_rearming_delayed_workqueue(data->workqueue, &data->work);
	return 0;
}

static int gdium_laptop_resume(struct i2c_client *client)
{
	struct gdium_laptop_data *data = i2c_get_clientdata(client);

	if (!ec)
		queue_delayed_work(data->workqueue, &data->work, msecs_to_jiffies(BAT_SCAN_INTERVAL));
	return 0;
}
#else
#define gdium_laptop_suspend NULL
#define gdium_laptop_resume NULL
#endif
static const struct i2c_device_id gdium_id[] = {
	{ "gdium-laptop" },
	{},
};
MODULE_DEVICE_TABLE(i2c, gdium_id);

static struct i2c_driver gdium_laptop_driver = {
	.driver = {
		.name = "gdium-laptop",
		.owner = THIS_MODULE,
	},
	.probe = gdium_laptop_probe,
	.remove = gdium_laptop_remove,
	.shutdown = gdium_laptop_remove,
	.suspend = gdium_laptop_suspend,
	.resume = gdium_laptop_resume,
	.id_table = gdium_id,
};

static int __init gdium_laptop_init(void)
{
	return i2c_add_driver(&gdium_laptop_driver);
}

static void __exit gdium_laptop_exit(void)
{
	i2c_del_driver(&gdium_laptop_driver);
}

module_init(gdium_laptop_init);
module_exit(gdium_laptop_exit);

MODULE_AUTHOR("Arnaud Patard <apatard@mandriva.com>");
MODULE_DESCRIPTION("Gdium laptop extras");
MODULE_LICENSE("GPL");
