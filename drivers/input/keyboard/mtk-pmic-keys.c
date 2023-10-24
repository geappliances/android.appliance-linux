// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 MediaTek, Inc.
 *
 * Author: Chen Zhong <chen.zhong@mediatek.com>
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/mt6323/registers.h>
#include <linux/mfd/mt6357/registers.h>
#include <linux/mfd/mt6358/registers.h>
#include <linux/mfd/mt6392/registers.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mfd/mt6397/registers.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define HOME_DUMP_PROCESSES 1

#ifdef HOME_DUMP_PROCESSES
#include <../kernel/sched/sched.h>
#include <asm-generic/bug.h>
#include <linux/cpumask.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/sched/signal.h>
#include "linux/slab.h"
#include <linux/timer.h>
#endif

#define MTK_PMIC_PWRKEY_RST_EN_MASK	0x1
#define MTK_PMIC_PWRKEY_RST_EN_SHIFT	6
#define MTK_PMIC_HOMEKEY_RST_EN_MASK	0x1
#define MTK_PMIC_HOMEKEY_RST_EN_SHIFT	5
#define MTK_PMIC_RST_DU_MASK		0x3
#define MTK_PMIC_RST_DU_SHIFT		8

#define MTK_PMIC_PWRKEY_RST		\
	(MTK_PMIC_PWRKEY_RST_EN_MASK << MTK_PMIC_PWRKEY_RST_EN_SHIFT)
#define MTK_PMIC_HOMEKEY_RST		\
	(MTK_PMIC_HOMEKEY_RST_EN_MASK << MTK_PMIC_HOMEKEY_RST_EN_SHIFT)

#define MTK_PMIC_PWRKEY_INDEX	0
#define MTK_PMIC_HOMEKEY_INDEX	1
#define MTK_PMIC_MAX_KEY_COUNT	2

struct mtk_pmic_keys_regs {
	u32 deb_reg;
	u32 deb_mask;
	u32 intsel_reg;
	u32 intsel_mask;
};

#define MTK_PMIC_KEYS_REGS(_deb_reg, _deb_mask,		\
	_intsel_reg, _intsel_mask)			\
{							\
	.deb_reg		= _deb_reg,		\
	.deb_mask		= _deb_mask,		\
	.intsel_reg		= _intsel_reg,		\
	.intsel_mask		= _intsel_mask,		\
}

struct mtk_pmic_regs {
	const struct mtk_pmic_keys_regs keys_regs[MTK_PMIC_MAX_KEY_COUNT];
	u32 pmic_rst_reg;
};

static const struct mtk_pmic_regs mt6397_regs = {
	.keys_regs[MTK_PMIC_PWRKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6397_CHRSTATUS,
		0x8, MT6397_INT_RSV, 0x10),
	.keys_regs[MTK_PMIC_HOMEKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6397_OCSTATUS2,
		0x10, MT6397_INT_RSV, 0x8),
	.pmic_rst_reg = MT6397_TOP_RST_MISC,
};

static const struct mtk_pmic_regs mt6323_regs = {
	.keys_regs[MTK_PMIC_PWRKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6323_CHRSTATUS,
		0x2, MT6323_INT_MISC_CON, 0x10),
	.keys_regs[MTK_PMIC_HOMEKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6323_CHRSTATUS,
		0x4, MT6323_INT_MISC_CON, 0x8),
	.pmic_rst_reg = MT6323_TOP_RST_MISC,
};

static const struct mtk_pmic_regs mt6357_regs = {
	.keys_regs[MTK_PMIC_PWRKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6357_TOPSTATUS,
		0x2, MT6357_PSC_TOP_INT_CON0, 0x5),
	.keys_regs[MTK_PMIC_HOMEKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6357_TOPSTATUS,
		0x8, MT6357_PSC_TOP_INT_CON0, 0xa),
	.pmic_rst_reg = MT6357_TOP_RST_MISC,
};

static const struct mtk_pmic_regs mt6358_regs = {
	.keys_regs[MTK_PMIC_PWRKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6358_TOPSTATUS,
		0x2, MT6358_PSC_TOP_INT_CON0, 0x5),
	.keys_regs[MTK_PMIC_HOMEKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6358_TOPSTATUS,
		0x8, MT6358_PSC_TOP_INT_CON0, 0xa),
	.pmic_rst_reg = MT6358_TOP_RST_MISC,
};

static const struct mtk_pmic_regs mt6392_regs = {
	.keys_regs[MTK_PMIC_PWRKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6392_CHRSTATUS,
		0x2, MT6392_INT_MISC_CON, 0x10),
	.keys_regs[MTK_PMIC_HOMEKEY_INDEX] =
		MTK_PMIC_KEYS_REGS(MT6392_CHRSTATUS,
		0x4, MT6392_INT_MISC_CON, 0x8),
	.pmic_rst_reg = MT6392_TOP_RST_MISC,
};

struct mtk_pmic_keys_info {
	struct mtk_pmic_keys *keys;
	const struct mtk_pmic_keys_regs *regs;
	unsigned int keycode;
	int irq;
	int irq_r; /* optional: release irq if different */
	bool wakeup:1;
};

struct mtk_pmic_keys {
	struct input_dev *input_dev;
	struct device *dev;
	struct regmap *regmap;
	struct mtk_pmic_keys_info keys[MTK_PMIC_MAX_KEY_COUNT];
};

enum mtk_pmic_keys_lp_mode {
	LP_DISABLE,
	LP_ONEKEY,
	LP_TWOKEY,
};

#ifdef HOME_DUMP_PROCESSES
#define INIT_TIMEOUT    5000	/* Init timeout 5 seconds */
#define REPEAT_TIMEOUT  2000	/* Repeat timeout 2 seconds */

static bool  mtk_pmic_is_active = 0;

static struct mtk_pmic_keys*  poll_mtk_pmic_keys = NULL;

static struct timer_list  poll_timer;

static u32 prev_dump_process_key_pressed = 0;
static u32 dump_process_info = 0;

static int process_cpu(const struct task_struct *p)
{
	unsigned int cpu = task_cpu(p);
	if (cpu > num_possible_cpus())
		cpu = 0;
	return cpu;
}

static char task_state_char (const struct task_struct *p)
{
	int cpu;
	char state;
	unsigned long tmp;

	if (!p || probe_kernel_read(&tmp, (char *)p, sizeof(unsigned long)))
		return 'E';

	cpu = process_cpu(p);
	state = (p->state == 0) ? 'R' :
		(p->state < 0) ? 'U' :
		(p->state & TASK_UNINTERRUPTIBLE) ? 'D' :
		(p->state & TASK_STOPPED) ? 'T' :
		(p->state & TASK_TRACED) ? 'C' :
		(p->exit_state & EXIT_ZOMBIE) ? 'Z' :
		(p->exit_state & EXIT_DEAD) ? 'E' :
		(p->state & TASK_INTERRUPTIBLE) ? 'S' : '?';
	if (is_idle_task(p)) {
		state = 'I';	/* idle task */
	} else if (!p->mm && state == 'S') {
		state = 'M';	/* sleeping system daemon */
	}
	return state;
}

static void dump_task_info (struct task_struct* p)
{
	printk(KERN_EMERG "Stack traceback for pid %d\n", p->pid);
	printk(KERN_EMERG "0x%px %8d %8d  %d %4d   %c  0x%px %s\n",
		   (void *)p, p->pid, p->parent->pid,
		   task_curr(p), process_cpu(p),
		   task_state_char(p),
		   (void *)(&p->thread),
		   p->comm);

	show_stack (p, NULL);
}

static void dump_all_task_info (void)
{
	unsigned long cpu;
	struct task_struct* g;
	struct task_struct* p;

	printk(KERN_EMERG CUT_HERE);
	for_each_online_cpu (cpu) {
		p = cpu_curr(cpu);
		dump_task_info(p);
		printk(KERN_EMERG CUT_HERE);
	}

	for_each_process_thread(g, p) {
		if (task_curr(p))
			continue;
		dump_task_info(p);
		printk(KERN_EMERG CUT_HERE);
	}

	printk(KERN_EMERG "\n");
}

static void poll_timer_fn (struct timer_list*  timer)
{
	u32 key_deb;
	u32 pressed;


	if (poll_mtk_pmic_keys != NULL) {
		if (!mtk_pmic_is_active) {

#if 0
			regmap_read(poll_mtk_pmic_keys->regmap, poll_mtk_pmic_keys->keys [MTK_PMIC_HOMEKEY_INDEX].regs->deb_reg, &key_deb);

			key_deb &= poll_mtk_pmic_keys->keys [MTK_PMIC_HOMEKEY_INDEX].regs->deb_mask;
#endif

			regmap_read(poll_mtk_pmic_keys->regmap, poll_mtk_pmic_keys->keys [MTK_PMIC_PWRKEY_INDEX].regs->deb_reg, &key_deb);

			key_deb &= poll_mtk_pmic_keys->keys [MTK_PMIC_PWRKEY_INDEX].regs->deb_mask;


			pressed = !key_deb;

			if (!pressed) {
				if (prev_dump_process_key_pressed) {
					dump_process_info = true;
				}
			}

			prev_dump_process_key_pressed = pressed;
		}

		if (dump_process_info) {
			dump_process_info = false;
			dump_all_task_info ();
		}
	}

	mod_timer (&poll_timer, jiffies + msecs_to_jiffies (REPEAT_TIMEOUT));
}
#endif

static void mtk_pmic_keys_lp_reset_setup(struct mtk_pmic_keys *keys,
		u32 pmic_rst_reg)
{
	int ret;
	u32 long_press_mode, long_press_debounce;

	ret = of_property_read_u32(keys->dev->of_node,
		"power-off-time-sec", &long_press_debounce);
	if (ret)
		long_press_debounce = 0;

	regmap_update_bits(keys->regmap, pmic_rst_reg,
			   MTK_PMIC_RST_DU_MASK << MTK_PMIC_RST_DU_SHIFT,
			   long_press_debounce << MTK_PMIC_RST_DU_SHIFT);

	ret = of_property_read_u32(keys->dev->of_node,
		"mediatek,long-press-mode", &long_press_mode);
	if (ret)
		long_press_mode = LP_DISABLE;

	switch (long_press_mode) {
	case LP_ONEKEY:
		regmap_update_bits(keys->regmap, pmic_rst_reg,
				   MTK_PMIC_PWRKEY_RST,
				   MTK_PMIC_PWRKEY_RST);
		regmap_update_bits(keys->regmap, pmic_rst_reg,
				   MTK_PMIC_HOMEKEY_RST,
				   0);
		break;
	case LP_TWOKEY:
		regmap_update_bits(keys->regmap, pmic_rst_reg,
				   MTK_PMIC_PWRKEY_RST,
				   MTK_PMIC_PWRKEY_RST);
		regmap_update_bits(keys->regmap, pmic_rst_reg,
				   MTK_PMIC_HOMEKEY_RST,
				   MTK_PMIC_HOMEKEY_RST);
		break;
	case LP_DISABLE:
		regmap_update_bits(keys->regmap, pmic_rst_reg,
				   MTK_PMIC_PWRKEY_RST,
				   0);
		regmap_update_bits(keys->regmap, pmic_rst_reg,
				   MTK_PMIC_HOMEKEY_RST,
				   0);
		break;
	default:
		break;
	}
}

static irqreturn_t mtk_pmic_keys_irq_handler_thread(int irq, void *data)
{
	struct mtk_pmic_keys_info *info = data;
	u32 key_deb, pressed;

	regmap_read(info->keys->regmap, info->regs->deb_reg, &key_deb);

	key_deb &= info->regs->deb_mask;

	pressed = !key_deb;

#ifdef HOME_DUMP_PROCESSES

	// For dump-trace, filter out the power-key, so it does not get reported to upper layer
	if (info->keycode != KEY_POWER )
	{
		input_report_key(info->keys->input_dev, info->keycode, pressed);
		input_sync(info->keys->input_dev);
	}

#else
	input_report_key(info->keys->input_dev, info->keycode, pressed);
	input_sync(info->keys->input_dev);
#endif

#ifdef HOME_DUMP_PROCESSES

#if 0
	if (info->keycode == KEY_HOME) {
		if (!pressed) {
			if (prev_dump_process_key_pressed) {
				dump_process_info = true;
			}
		}

		prev_dump_process_key_pressed = pressed;
	}
#endif

	// Use power key to activate dump-trace
	if (info->keycode == KEY_POWER) {
		if (!pressed) {
			if (prev_dump_process_key_pressed) {
				dump_process_info = true;
			}
		}

		prev_dump_process_key_pressed = pressed;
	}
#endif

	dev_dbg(info->keys->dev, "(%s) key =%d using PMIC\n",
		 pressed ? "pressed" : "released", info->keycode);

	return IRQ_HANDLED;
}

static int mtk_pmic_key_setup(struct mtk_pmic_keys *keys,
		struct mtk_pmic_keys_info *info)
{
	int ret;

	info->keys = keys;

	ret = regmap_update_bits(keys->regmap, info->regs->intsel_reg,
				 info->regs->intsel_mask,
				 info->regs->intsel_mask);
	if (ret < 0)
		return ret;

	ret = devm_request_threaded_irq(keys->dev, info->irq, NULL,
					mtk_pmic_keys_irq_handler_thread,
					IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
					"mtk-pmic-keys", info);
	if (ret) {
		dev_err(keys->dev, "Failed to request IRQ: %d: %d\n",
			info->irq, ret);
		return ret;
	}

	if (info->irq_r > 0) {
		ret = devm_request_threaded_irq(
			keys->dev, info->irq_r, NULL,
			mtk_pmic_keys_irq_handler_thread,
			IRQF_ONESHOT | IRQF_TRIGGER_HIGH, "mtk-pmic-keys",
			info);
		if (ret) {
			dev_err(keys->dev, "Failed to request IRQ: %d: %d\n",
				info->irq, ret);
			return ret;
		}
	}

	input_set_capability(keys->input_dev, EV_KEY, info->keycode);

	return 0;
}

static int __maybe_unused mtk_pmic_keys_suspend(struct device *dev)
{
	struct mtk_pmic_keys *keys = dev_get_drvdata(dev);
	int index;

	for (index = 0; index < MTK_PMIC_MAX_KEY_COUNT; index++) {
		if (keys->keys[index].wakeup) {
			enable_irq_wake(keys->keys[index].irq);
			if (keys->keys[index].irq_r > 0)
				enable_irq_wake(keys->keys[index].irq_r);
		}
	}

	return 0;
}

static int __maybe_unused mtk_pmic_keys_resume(struct device *dev)
{
	struct mtk_pmic_keys *keys = dev_get_drvdata(dev);
	int index;

	for (index = 0; index < MTK_PMIC_MAX_KEY_COUNT; index++) {
		if (keys->keys[index].wakeup) {
			disable_irq_wake(keys->keys[index].irq);
			if (keys->keys[index].irq_r > 0)
				disable_irq_wake(keys->keys[index].irq_r);
		}
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(mtk_pmic_keys_pm_ops, mtk_pmic_keys_suspend,
			mtk_pmic_keys_resume);

static const struct of_device_id of_mtk_pmic_keys_match_tbl[] = {
	{
		.compatible = "mediatek,mt6397-keys",
		.data = &mt6397_regs,
	}, {
		.compatible = "mediatek,mt6323-keys",
		.data = &mt6323_regs,
	}, {
		.compatible = "mediatek,mt6357-keys",
		.data = &mt6357_regs,
	}, {
		.compatible = "mediatek,mt6358-keys",
		.data = &mt6358_regs,
	}, {
		.compatible = "mediatek,mt6392-keys",
		.data = &mt6392_regs,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, of_mtk_pmic_keys_match_tbl);

static int mtk_pmic_keys_probe(struct platform_device *pdev)
{
	int error, index = 0;
	unsigned int keycount;
	struct mt6397_chip *pmic_chip = dev_get_drvdata(pdev->dev.parent);
	struct device_node *node = pdev->dev.of_node, *child;
	static const char *const irqnames[] = { "powerkey", "homekey" };
	static const char *const irqnames_r[] = { "powerkey_r", "homekey_r" };
	struct mtk_pmic_keys *keys;
	const struct mtk_pmic_regs *mtk_pmic_regs;
	struct input_dev *input_dev;
	const struct of_device_id *of_id =
		of_match_device(of_mtk_pmic_keys_match_tbl, &pdev->dev);

#ifdef HOME_DUMP_PROCESSES
	keys = kzalloc (sizeof(*keys), GFP_KERNEL);
	poll_mtk_pmic_keys = keys;
#else
	keys = devm_kzalloc(&pdev->dev, sizeof(*keys), GFP_KERNEL);
#endif

	if (!keys)
		return -ENOMEM;

	keys->dev = &pdev->dev;
	keys->regmap = pmic_chip->regmap;
	mtk_pmic_regs = of_id->data;

	keys->input_dev = input_dev = devm_input_allocate_device(keys->dev);
	if (!input_dev) {
		dev_err(keys->dev, "input allocate device fail.\n");
		return -ENOMEM;
	}

	input_dev->name = "mtk-pmic-keys";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;

	keycount = of_get_available_child_count(node);
	if (keycount > MTK_PMIC_MAX_KEY_COUNT ||
	    keycount > ARRAY_SIZE(irqnames)) {
		dev_err(keys->dev, "too many keys defined (%d)\n", keycount);
		return -EINVAL;
	}

	for_each_child_of_node(node, child) {
		keys->keys[index].regs = &mtk_pmic_regs->keys_regs[index];

		keys->keys[index].irq =
			platform_get_irq_byname(pdev, irqnames[index]);
		if (keys->keys[index].irq < 0) {
			of_node_put(child);
			return keys->keys[index].irq;
		}

		/* optional: release irq might be separate */
		keys->keys[index].irq_r =
			platform_get_irq_byname_optional(pdev, irqnames_r[index]);

		error = of_property_read_u32(child,
			"linux,keycodes", &keys->keys[index].keycode);
		if (error) {
			dev_err(keys->dev,
				"failed to read key:%d linux,keycode property: %d\n",
				index, error);
			of_node_put(child);
			return error;
		}

		if (of_property_read_bool(child, "wakeup-source"))
			keys->keys[index].wakeup = true;

		error = mtk_pmic_key_setup(keys, &keys->keys[index]);
		if (error) {
			of_node_put(child);
			return error;
		}

		index++;
	}

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev,
			"register input device failed (%d)\n", error);
		return error;
	}

	mtk_pmic_keys_lp_reset_setup(keys, mtk_pmic_regs->pmic_rst_reg);

	platform_set_drvdata(pdev, keys);

	return 0;
}

static struct platform_driver pmic_keys_pdrv = {
	.probe = mtk_pmic_keys_probe,
	.driver = {
		   .name = "mtk-pmic-keys",
		   .of_match_table = of_mtk_pmic_keys_match_tbl,
		   .pm = &mtk_pmic_keys_pm_ops,
	},
};

#if HOME_DUMP_PROCESSES
static int __init pmic_keys_pdrv_init(void)
{
	mtk_pmic_is_active = true;

	timer_setup (&poll_timer, poll_timer_fn, 0);
	mod_timer (&poll_timer, jiffies + msecs_to_jiffies (INIT_TIMEOUT));

	return platform_driver_register(&pmic_keys_pdrv);
}

module_init(pmic_keys_pdrv_init);

static void __exit pmic_keys_pdrv_exit(void)
{
	platform_driver_unregister(&pmic_keys_pdrv);

	mtk_pmic_is_active = false;
}

module_exit(pmic_keys_pdrv_exit);
#else
module_platform_driver(pmic_keys_pdrv);
#endif

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Chen Zhong <chen.zhong@mediatek.com>");
MODULE_DESCRIPTION("MTK pmic-keys driver v0.1");
