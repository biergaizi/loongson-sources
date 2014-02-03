/*
 * SM501 PWM clock
 * Copyright (C) 2009-2010 Arnaud Patard <apatard@mandriva.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/sm501.h>
#include <linux/sm501-regs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static const char drv_name[] = "sm501-pwm";

#define INPUT_CLOCK		96 /* MHz */
#define PWM_COUNT		3

#define SM501PWM_HIGH_COUNTER	(1<<20)
#define SM501PWM_LOW_COUNTER	(1<<8)
#define SM501PWM_CLOCK_DIVIDE	(1>>4)
#define SM501PWM_IP		(1<<3)
#define SM501PWM_I		(1<<2)
#define SM501PWM_E		(1<<0)

struct pwm_device {
	struct list_head	node;
	struct device		*dev;
	void __iomem		*regs;
	int			duty_ns;
	int			period_ns;
	char			enabled;
	void			(*handler)(struct pwm_device *pwm);

	const char		*label;
	unsigned int		use_count;
	unsigned int		pwm_id;
};

struct sm501pwm_info {
	void __iomem	*regs;
	int		irq;
	struct resource *res;
	struct device	*dev;
	struct dentry	*debugfs;

	struct pwm_device pwm[3];
};

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned int high, low, divider;
	int divider1, divider2;
	unsigned long long delay;

	if (!pwm || !pwm->regs || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	/* Get delay
	 * We're loosing some precision but multiplying then dividing
	 * will overflow
	 */
	if (period_ns > 1000) {
		delay = period_ns / 1000;
		delay *= INPUT_CLOCK;
	} else {
		delay = period_ns * 96;
		delay /= 1000;
	}

	/* Get the number of clock low and high */
	high  = delay * duty_ns / period_ns;
	low = delay - high;

	/* Get divider to make 'low' and 'high' fit into 12 bits */
	/* No need to say that the divider must be >= 0 */
	divider1 = fls(low)-12;
	divider2 = fls(high)-12;

	if (divider1 < 0)
		divider1 = 0;
	if (divider2 < 0)
		divider2 = 0;

	divider = max(divider1, divider2);

	low >>= divider;
	high >>= divider;

	pwm->duty_ns = duty_ns;
	pwm->period_ns = period_ns;

	writel((high<<20)|(low<<8)|(divider<<4), pwm->regs);
	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	u32 reg;

	if (!pwm)
		return -EINVAL;

	switch (pwm->pwm_id) {
	case 0:
		sm501_configure_gpio(pwm->dev->parent, 29, 1);
		break;
	case 1:
		sm501_configure_gpio(pwm->dev->parent, 30, 1);
		break;
	case 2:
		sm501_configure_gpio(pwm->dev->parent, 31, 1);
		break;
	default:
		return -EINVAL;
	}

	reg = readl(pwm->regs);
	reg |= (SM501PWM_IP | SM501PWM_E);
	writel(reg, pwm->regs);
	pwm->enabled = 1;

	return 0;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	u32 reg;

	if (!pwm)
		return;

	reg = readl(pwm->regs);
	reg &= ~(SM501PWM_IP | SM501PWM_E);
	writel(reg, pwm->regs);

	switch (pwm->pwm_id) {
	case 0:
		sm501_configure_gpio(pwm->dev->parent, 29, 0);
		break;
	case 1:
		sm501_configure_gpio(pwm->dev->parent, 30, 0);
		break;
	case 2:
		sm501_configure_gpio(pwm->dev->parent, 31, 0);
		break;
	default:
		break;
	}
	pwm->enabled = 0;
}
EXPORT_SYMBOL(pwm_disable);

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int found = 0;

	mutex_lock(&pwm_lock);

	list_for_each_entry(pwm, &pwm_list, node) {
		if (pwm->pwm_id == pwm_id && pwm->use_count == 0) {
			pwm->use_count++;
			pwm->label = label;
			found = 1;
			break;
		}
	}

	mutex_unlock(&pwm_lock);

	return (found) ? pwm : NULL;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);

	if (pwm->use_count) {
		pwm->use_count--;
		pwm->label = NULL;
	} else
		dev_warn(pwm->dev, "PWM device already freed\n");

	mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(pwm_free);

int pwm_int_enable(struct pwm_device *pwm)
{
	unsigned long conf;

	if (!pwm || !pwm->regs || !pwm->handler)
		return -EINVAL;

	conf = readl(pwm->regs);
	conf |= SM501PWM_I;
	writel(conf, pwm->regs);
	return 0;
}
EXPORT_SYMBOL(pwm_int_enable);

int pwm_int_disable(struct pwm_device *pwm)
{
	unsigned long conf;

	if (!pwm || !pwm->regs || !pwm->handler)
		return -EINVAL;

	conf = readl(pwm->regs);
	conf &= ~SM501PWM_I;
	writel(conf, pwm->regs);
	return 0;
}
EXPORT_SYMBOL(pwm_int_disable);

int pwm_set_handler(struct pwm_device *pwm,
		    void (*handler)(struct pwm_device *pwm))
{
	if (!pwm || !handler)
		return -EINVAL;
	pwm->handler = handler;
	return 0;
}
EXPORT_SYMBOL(pwm_set_handler);

static irqreturn_t sm501pwm_irq(int irq, void *dev_id)
{
	unsigned long value;
	struct sm501pwm_info *info = (struct sm501pwm_info *)dev_id;
	struct pwm_device *pwm;
	int i;

	value = sm501_modify_reg(info->dev->parent, SM501_IRQ_STATUS, 0, 0);

	/* Check is the interrupt is for us */
	if (value & (1<<22)) {
		for (i = 0 ; i < PWM_COUNT ; i++) {
			/*
			 * Find which pwm triggered the interrupt
			 * and ack
			 */
			value = readl(info->regs + i*4);
			if (value & SM501PWM_IP)
				writel(value | SM501PWM_IP, info->regs + i*4);

			pwm = &info->pwm[i];
			if (pwm->handler)
				pwm->handler(pwm);
		}
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void add_pwm(int id, struct sm501pwm_info *info)
{
	struct pwm_device *pwm = &info->pwm[id];

	pwm->use_count	= 0;
	pwm->pwm_id	= id;
	pwm->dev	= info->dev;
	pwm->regs	= info->regs + id * 4;

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);
}

static void del_pwm(int id, struct sm501pwm_info *info)
{
	struct pwm_device *pwm = &info->pwm[id];

	pwm->use_count  = 0;
	pwm->pwm_id     = -1;
	mutex_lock(&pwm_lock);
	list_del(&pwm->node);
	mutex_unlock(&pwm_lock);
}

/* Debug fs */
static int sm501pwm_show(struct seq_file *s, void *p)
{
	struct pwm_device *pwm;

	mutex_lock(&pwm_lock);
	list_for_each_entry(pwm, &pwm_list, node) {
		if (pwm->use_count) {
			seq_printf(s, "pwm-%d (%12s) %d %d %s\n",
					pwm->pwm_id, pwm->label,
					pwm->duty_ns, pwm->period_ns,
					pwm->enabled ? "on" : "off");
			seq_printf(s, "       %08x\n", readl(pwm->regs));
		}
	}
	mutex_unlock(&pwm_lock);

	return 0;
}

static int sm501pwm_open(struct inode *inode, struct file *file)
{
	return single_open(file, sm501pwm_show, inode->i_private);
}

static const struct file_operations sm501pwm_fops = {
	.open		= sm501pwm_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int __init sm501pwm_probe(struct platform_device *pdev)
{
	struct sm501pwm_info *info;
	struct device   *dev = &pdev->dev;
	struct resource *res;
	int ret = 0;
	int res_len;
	int i;

	info = kzalloc(sizeof(struct sm501pwm_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "Allocation failure\n");
		ret = -ENOMEM;
		goto err;
	}
	info->dev = dev;
	platform_set_drvdata(pdev, info);

	/* Get irq number */
	info->irq = platform_get_irq(pdev, 0);
	if (!info->irq) {
		dev_err(dev, "no irq found\n");
		ret = -ENODEV;
		goto err_alloc;
	}

	/* Get regs address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "No memory resource found\n");
		ret = -ENODEV;
		goto err_alloc;
	}
	info->res = res;
	res_len = (res->end - res->start)+1;

	if (!request_mem_region(res->start, res_len, drv_name)) {
		dev_err(dev, "Can't request iomem resource\n");
		ret = -EBUSY;
		goto err_alloc;
	}

	info->regs = ioremap(res->start, res_len);
	if (!info->regs) {
		dev_err(dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_mem;
	}

	ret = request_irq(info->irq, sm501pwm_irq, IRQF_SHARED, drv_name, info);
	if (ret != 0) {
		dev_err(dev, "can't get irq\n");
		goto err_map;
	}


	sm501_unit_power(info->dev->parent, SM501_GATE_GPIO, 1);

	for (i = 0; i < 3; i++)
		add_pwm(i, info);

	dev_info(dev, "SM501 PWM Found at %lx irq %d\n",
		 (unsigned long)info->res->start, info->irq);

	info->debugfs = debugfs_create_file("pwm", S_IFREG | S_IRUGO,
				NULL, info, &sm501pwm_fops);


	return 0;

err_map:
	iounmap(info->regs);

err_mem:
	release_mem_region(res->start, res_len);

err_alloc:
	kfree(info);
	platform_set_drvdata(pdev, NULL);
err:
	return ret;
}

static int sm501pwm_remove(struct platform_device *pdev)
{
	struct sm501pwm_info *info = platform_get_drvdata(pdev);
	int i;

	if (info->debugfs)
		debugfs_remove(info->debugfs);

	for (i = 0; i < 3; i++) {
		pwm_disable(&info->pwm[i]);
		del_pwm(i, info);
	}

	sm501_unit_power(info->dev->parent, SM501_GATE_GPIO, 0);
	sm501_modify_reg(info->dev->parent, SM501_IRQ_STATUS, 0, 1<<22);

	free_irq(info->irq, info);
	iounmap(info->regs);
	release_mem_region(info->res->start,
			   (info->res->end - info->res->start)+1);
	kfree(info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver sm501pwm_driver = {
	.probe		= sm501pwm_probe,
	.remove		= sm501pwm_remove,
	.driver		= {
		.name	= drv_name,
		.owner	= THIS_MODULE,
	},
};

static int __devinit sm501pwm_init(void)
{
	return platform_driver_register(&sm501pwm_driver);
}

static void __exit sm501pwm_cleanup(void)
{
	platform_driver_unregister(&sm501pwm_driver);
}

module_init(sm501pwm_init);
module_exit(sm501pwm_cleanup);

MODULE_AUTHOR("Arnaud Patard <apatard@mandriva.com>");
MODULE_DESCRIPTION("SM501 PWM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sm501-pwm");
