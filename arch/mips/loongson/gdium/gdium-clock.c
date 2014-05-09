/*
 * Doesn't work really well. When used, the clocksource is producing
 * bad timings and the clockevent can't be used (don't have one shot feature
 * thus can't switch on the fly and the pwm is initialised too late to be able
 * to use it at boot time).
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pwm.h>
#include <linux/clocksource.h>
#include <linux/debugfs.h>
#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/mips-boards/bonito64.h>
#include <asm/time.h>

#include <loongson.h>

#define CLOCK_PWM		1
#define CLOCK_PWM_FREQ		1500000				/* Freq in Hz */
#define CLOCK_LATCH		((CLOCK_PWM_FREQ + HZ/2) / HZ)
#define CLOCK_PWM_PERIOD	(1000000000/CLOCK_PWM_FREQ)	/* period ns  */
#define CLOCK_PWM_DUTY		50
#define CLOCK_PWM_IRQ		(MIPS_CPU_IRQ_BASE + 4)

static const char drv_name[] = "gdium-clock";

static struct pwm_device *clock_pwm;

static DEFINE_SPINLOCK(clock_pwm_lock);
static uint64_t clock_tick;

static irqreturn_t gdium_pwm_clock_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *cd = dev_id;
	unsigned long flag;

	spin_lock_irqsave(&clock_pwm_lock, flag);
	clock_tick++;
	/* wait intn2 to finish */
	do {
		LOONGSON_INTENCLR = (1 << 13);
	} while (LOONGSON_INTISR & (1 << 13));
	spin_unlock_irqrestore(&clock_pwm_lock, flag);

	if (cd && cd->event_handler)
		cd->event_handler(cd);

	return IRQ_HANDLED;
}

static cycle_t gdium_pwm_clock_read(struct clocksource *cs)
{
	unsigned long flag;
	uint32_t jifs;
	uint64_t ticks;

	spin_lock_irqsave(&clock_pwm_lock, flag);
	jifs = jiffies;
	ticks = clock_tick;
	spin_unlock_irqrestore(&clock_pwm_lock, flag);
	/* return (cycle_t)ticks; */
	return (cycle_t)(CLOCK_LATCH * jifs);
}

static struct clocksource gdium_pwm_clock_clocksource = {
	.name   = "gdium_csrc",
	.read   = gdium_pwm_clock_read,
	.mask   = CLOCKSOURCE_MASK(64),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS | CLOCK_SOURCE_MUST_VERIFY,
	.shift	= 20,
};

/* Debug fs */
static int gdium_pwm_clock_show(struct seq_file *s, void *p)
{
	unsigned long flag;
	uint64_t ticks;

	spin_lock_irqsave(&clock_pwm_lock, flag);
	ticks = clock_tick;
	spin_unlock_irqrestore(&clock_pwm_lock, flag);
	seq_printf(s, "%lld\n", ticks);
	return 0;
}

static int gdium_pwm_clock_open(struct inode *inode, struct file *file)
{
	return single_open(file, gdium_pwm_clock_show, inode->i_private);
}

static const struct file_operations gdium_pwm_clock_fops = {
	.open		= gdium_pwm_clock_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};
static struct dentry   *debugfs_file;

static void gdium_pwm_clock_set_mode(enum clock_event_mode mode,
		struct clock_event_device *evt)
{
	/* Nothing to do ...  */
}

static struct clock_event_device gdium_pwm_clock_cevt = {
	.name           = "gdium_cevt",
	.features       = CLOCK_EVT_FEAT_PERIODIC,
	/* .mult, .shift, .max_delta_ns and .min_delta_ns left uninitialized */
	.rating         = 299,
	.irq            = CLOCK_PWM_IRQ,
	.set_mode       = gdium_pwm_clock_set_mode,
};

static struct platform_device_id platform_device_ids[] = {
	{
		.name = "gdium-pwmclk",
	},
	{}
};
MODULE_DEVICE_TABLE(platform, platform_device_ids);

static struct platform_driver gdium_pwm_clock_driver = {
	.driver		= {
		.name   = drv_name,
		.owner  = THIS_MODULE,
	},
	.id_table = platform_device_ids,
};

static int gdium_pwm_clock_drvinit(void)
{
	int ret;
	struct clocksource *cs = &gdium_pwm_clock_clocksource;
	struct clock_event_device *cd = &gdium_pwm_clock_cevt;
	unsigned int cpu = smp_processor_id();

	clock_tick = 0;

	clock_pwm = pwm_request(CLOCK_PWM, drv_name);
	if (clock_pwm == NULL) {
		pr_err("unable to request PWM for Gdium clock\n");
		return -EBUSY;
	}
	ret = pwm_config(clock_pwm, CLOCK_PWM_DUTY, CLOCK_PWM_PERIOD);
	if (ret) {
		pr_err("unable to configure PWM for Gdium clock\n");
		goto err_pwm_request;
	}
	ret = pwm_enable(clock_pwm);
	if (ret) {
		pr_err("unable to enable PWM for Gdium clock\n");
		goto err_pwm_request;
	}

	cd->cpumask = cpumask_of(cpu);

	cd->shift = 22;
	cd->mult  = div_sc(CLOCK_PWM_FREQ, NSEC_PER_SEC, cd->shift);
	cd->max_delta_ns = clockevent_delta2ns(0x7FFF, cd);
	cd->min_delta_ns = clockevent_delta2ns(0xF, cd);
	clockevents_register_device(&gdium_pwm_clock_cevt);

	/* SM501 PWM1 connected to intn2 <->ip4 */
	LOONGSON_INTPOL = (1 << 13);
	LOONGSON_INTEDGE &= ~(1 << 13);
	ret = request_irq(CLOCK_PWM_IRQ, gdium_pwm_clock_interrupt, IRQF_DISABLED, drv_name, &gdium_pwm_clock_cevt);
	if (ret) {
		pr_err("Can't claim irq\n");
		goto err_pwm_disable;
	}

	cs->rating = 200;
	cs->mult = clocksource_hz2mult(CLOCK_PWM_FREQ, cs->shift);
	ret = clocksource_register(&gdium_pwm_clock_clocksource);
	if (ret) {
		pr_err("Can't register clocksource\n");
		goto err_irq;
	}
	pr_info("Clocksource registered with shift %d and mult %d\n",
			cs->shift, cs->mult);

	debugfs_file = debugfs_create_file(drv_name, S_IFREG | S_IRUGO,
			NULL, NULL, &gdium_pwm_clock_fops);

	return 0;

err_irq:
	free_irq(CLOCK_PWM_IRQ, &gdium_pwm_clock_cevt);
err_pwm_disable:
	pwm_disable(clock_pwm);
err_pwm_request:
	pwm_free(clock_pwm);
	return ret;
}

static void gdium_pwm_clock_drvexit(void)
{
	free_irq(CLOCK_PWM_IRQ, &gdium_pwm_clock_cevt);
	pwm_disable(clock_pwm);
	pwm_free(clock_pwm);
}


static int __devinit gdium_pwm_clock_init(void)
{
	int ret = gdium_pwm_clock_drvinit();

	if (ret) {
		pr_err("Fail to register gdium clock driver\n");
		return ret;
	}

	return platform_driver_register(&gdium_pwm_clock_driver);
}

static void __exit gdium_pwm_clock_cleanup(void)
{
	gdium_pwm_clock_drvexit();
	platform_driver_unregister(&gdium_pwm_clock_driver);
}

module_init(gdium_pwm_clock_init);
module_exit(gdium_pwm_clock_cleanup);

MODULE_AUTHOR("Arnaud Patard <apatard@mandriva.com>");
MODULE_DESCRIPTION("Gdium PWM clock driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gdium-pwmclk");
