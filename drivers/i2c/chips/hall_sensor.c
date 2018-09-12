
/* drivers/input/misc/hall_sensor.c - Ak8789 Hall sensor driver
 *
 * Copyright (C) 2013 HTC Corporation.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include <linux/async.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/ak8789.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/hall_sensor.h>
#include <linux/xlog.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <mach/mt_gpio.h>

#define DRIVER_NAME "HL"

struct ak_hall_data {
	struct input_dev *input_dev;
	uint32_t gpio_att:16;
	uint32_t gpio_att_s:16;
	uint8_t  debug_level;
	uint8_t  hall_enable;
	uint8_t  irq_enable;
	uint8_t  att_used;
	struct wake_lock wake_lock;
	uint32_t irq_n:16;
	uint32_t irq_s:16;

	struct workqueue_struct *wq_npole;
	struct workqueue_struct *wq_spole;
	struct work_struct work_npole_irq;
	struct work_struct work_spole_irq;
};

static struct ak_hall_data *g_hl;
static int prev_val_n = 0;
static int prev_val_s = 0;
static int first_boot_s = 1;
static int first_boot_n = 1;

static void report_cover_event(int pole, int irq, struct ak_hall_data *hl);
static ssize_t debug_level_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ak_hall_data *hl = g_hl;
	if (buf[0] == '0' || buf[0] == '1') {
		hl->debug_level = buf[0] - '0';
		pr_info("[HL] debug_level = %d\b", hl->debug_level);
	} else
		pr_info("[HL] Parameter Error\n");
	return count;
}

static ssize_t debug_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ak_hall_data *hl = g_hl;
	return snprintf(buf, PAGE_SIZE, "[HL] debug_level = %d\n", hl->debug_level);
}
DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO), debug_level_show, debug_level_set);

static ssize_t read_att(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0, pos = 0;
	char string[100] = {0};
	struct ak_hall_data *hl = g_hl;

	ret = mt_get_gpio_in(hl->gpio_att);
	if(first_boot_n && ret == 0)
	{
		prev_val_n = 1;
		report_cover_event(0, hl->irq_n, hl);
	}
	first_boot_n = 0;
	HL_LOG("[HL]ATT(%d):GPIO_ATT=%d", hl->att_used, ret);
	pos += snprintf(string+pos, sizeof(string), "ATT(%d):ATT_N=%d", hl->att_used, ret);

	if (hl->att_used > 1) {
		ret = mt_get_gpio_in(hl->gpio_att_s);
		if(first_boot_s && ret == 0)
		{
			prev_val_s = 1;
			report_cover_event(1, hl->irq_s, hl);
		}
		first_boot_s = 0;
		HL_LOG("[HL]GPIO_ATT_S=%d", ret);
		pos += snprintf(string+pos, sizeof(string)-pos,  ", ATT_S=%d", ret);
	}

	pos = snprintf(buf, pos + 2, "%s", string);
	return pos;
}

static ssize_t write_att(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ak_hall_data *hl = g_hl;
	if (buf[0] == '0' || buf[0] == '1') {
		if(hl->hall_enable == 1 && buf[0] == '0')
		{
			if (hl->irq_enable) {
				HL_LOG("Disable hall sensor interrupts\n");
				disable_irq_nosync(hl->irq_n);
				disable_irq_nosync(hl->irq_s);
				hl->irq_enable = 0;
			}
			hl->hall_enable = 0;
			irq_set_irq_wake(hl->irq_n, 0);
			irq_set_irq_wake(hl->irq_s, 0);
			HL_LOG("irq_set_irq_wake: 0\n");
		}
		else if(hl->hall_enable == 0 && buf[0] == '1')
		{
			if (!hl->irq_enable) {
				HL_LOG("Enable hall sensor interrupts\n");
				report_cover_event(0, hl->irq_n, hl);
				report_cover_event(1, hl->irq_s, hl);
				enable_irq(hl->irq_n);
				enable_irq(hl->irq_s);
				hl->irq_enable = 1;
			}
			hl->hall_enable = 1;
			irq_set_irq_wake(hl->irq_s, 1);
			irq_set_irq_wake(hl->irq_n, 1);
			HL_LOG("irq_set_irq_wake: 1\n");
		}
		else
			HL_LOG("[HL]Invalid paramater(0:Disable 1:Enable) hall enable = %d\n", hl->hall_enable);
	} else
		pr_info("[HL] Parameter Error\n");

	return count;
}

static DEVICE_ATTR(read_att, S_IRUGO | S_IWUSR , read_att, write_att);

static struct kobject *android_cover_kobj;

static int hall_cover_sysfs_init(void)
{
	int ret = 0;
	android_cover_kobj = kobject_create_and_add("android_cover", NULL);
	if (android_cover_kobj == NULL) {
		HL_ERR("[HL]%s:subsystem_register_failed", __func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(android_cover_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		HL_ERR("[HL]%s: sysfs_create_file debug_level failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_cover_kobj, &dev_attr_read_att.attr);
	if (ret) {
		HL_ERR("[HL]%s: sysfs_create_file read_att failed\n", __func__);
		return ret;
	}
	HL_LOG("[HL]attribute file register Done");
	return 0;
}

#ifdef CONFIG_OF
static int hall_sensor_dt_parser(struct device_node *dt, struct hall_platform_data *pdata)
{
	struct property *prop;
	int ret = 0;
	const char *parser_st[] = {"hall,att_used", "hall,att_gpio", "hall,att_gpio_s"};
	uint32_t gpio_att = 0;
	u32 buf = 0;
	ret = of_property_read_u32(dt, parser_st[0], (u32 *)&pdata->att_used);
	if (ret < 0) {
		HL_LOG("[HL]DT:%s parser err, ret=%d", parser_st[0], ret);
		goto parser_failed;
	} else
		HL_LOG("[HL]DT:%s=%d", parser_st[0], pdata->att_used);

	prop = of_find_property(dt, parser_st[1], NULL);
	if(!prop){
		HL_ERR("[HL]DT: %s parser failue, ret=%d", parser_st[1], gpio_att);
		ret = gpio_att;
		goto parser_failed;
	} else {
		of_property_read_u32(dt, parser_st[1], &buf);
		pdata->gpio_att = buf;
		HL_LOG("[HL]DT:%s[%d] read", parser_st[1], pdata->gpio_att);
	}
	if (pdata->att_used > 1) {
		prop = of_find_property(dt, parser_st[1], NULL);
		if(!prop){
			HL_ERR("[HL]DT: %s parser failue, ret=%d", parser_st[2], gpio_att);
			ret = gpio_att;
			goto parser_failed;
		} else {
			of_property_read_u32(dt, parser_st[2], &buf);
			pdata->gpio_att_s = buf;
			HL_LOG("[HL]DT:%s[%d] read", parser_st[2], pdata->gpio_att_s);
		}
	}

	return 0;
parser_failed:
	return ret;
}
#endif


static int hall_input_register(struct ak_hall_data *hl)
{
	int ret = 0;
	hl->input_dev = input_allocate_device();
	if (!hl->input_dev) {
		ret = -ENOMEM;
		HL_ERR("[HL]%s: Failed to allocate input_device", __func__);
		return ret;
	}
	input_set_drvdata(hl->input_dev, hl);
	hl->input_dev->name = HL_INPUTDEV_NAME;

	set_bit(EV_SYN, hl->input_dev->evbit);
	set_bit(EV_KEY, hl->input_dev->evbit);

	input_set_capability(hl->input_dev, EV_KEY, HALL_N_POLE);
	input_set_capability(hl->input_dev, EV_KEY, HALL_S_POLE);

	HL_LOG("[HL]%s\n", __func__);
	return input_register_device(hl->input_dev);
}

#ifdef CONFIG_MTK_HX8527
extern int tpd_enable_high_Sensitivity(int enable);
#endif
static void report_cover_event(int pole, int irq, struct ak_hall_data *hl)
{
	uint8_t val_n = 0, val_s = 0;
	if (pole == HALL_POLE_N) {		// N-pole
		val_n = mt_get_gpio_in(hl->gpio_att);
		irq_set_irq_type(irq, val_n?IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		wake_lock_timeout(&hl->wake_lock, (2 * HZ));

		if (prev_val_n != val_n) {
			input_report_key(hl->input_dev, HALL_N_POLE, !val_n);
			input_sync(hl->input_dev);
			prev_val_n = val_n;
			HL_LOG("att_n[%s]", val_n ? "Far" : "Near");
			hallsensor_notifier_call_chain((HALL_POLE_N << HALL_POLE_BIT) |(!val_n), NULL);
#ifdef CONFIG_MTK_HX8527
			tpd_enable_high_Sensitivity(!val_n);
#endif
		}
	} else if (pole == HALL_POLE_S) {	//S-pole
		val_s = mt_get_gpio_in(hl->gpio_att_s);
		irq_set_irq_type(irq, val_s?IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		wake_lock_timeout(&hl->wake_lock, (2 * HZ));

		if (prev_val_s != val_s) {
			input_report_key(hl->input_dev, HALL_S_POLE, !val_s);
			input_sync(hl->input_dev);
			prev_val_s = val_s;
			HL_LOG("att_s[%s]", val_s ? "Far" : "Near");
			hallsensor_notifier_call_chain((HALL_POLE_S << HALL_POLE_BIT) |(!val_s), NULL);
#ifdef CONFIG_MTK_HX8527
			tpd_enable_high_Sensitivity(!val_s);
#endif
		}
	}
}

static void hall_npole_work(struct work_struct *work)
{
	struct ak_hall_data *hl = container_of(work, struct ak_hall_data, work_npole_irq);
	report_cover_event(HALL_POLE_N, hl->irq_n, hl);
}

static void hall_spole_work(struct work_struct *work)
{
	struct ak_hall_data *hl = container_of(work, struct ak_hall_data, work_spole_irq);
	report_cover_event(HALL_POLE_S, hl->irq_s, hl);
}

static irqreturn_t hall_npole_irq_thread(int irq, void *ptr)
{
	struct ak_hall_data *hl = ptr;
	HL_LOG_TIME("N-pole interrupt trigger");
	queue_work(hl->wq_npole, &hl->work_npole_irq);
	return IRQ_HANDLED;
}

static irqreturn_t hall_spole_irq_thread(int irq, void *ptr)
{
	struct ak_hall_data *hl = ptr;
	HL_LOG_TIME("S-pole interrupt trigger");
	queue_work(hl->wq_spole, &hl->work_spole_irq);
	return IRQ_HANDLED;
}

static int hall_sensor_probe(struct platform_device *pdev)
{
	struct device_node *node = NULL;
	unsigned int irq;
	int ret = 0;
	struct ak_hall_data * hl;
	struct hall_platform_data *pdata;
	printk("%s:\n", __func__);

	hl = kzalloc(sizeof(*hl), GFP_KERNEL);
	if (hl == NULL) {
		ret = -ENOMEM;
		goto err_alloc_mem_failed;
		printk("%s:hl = kzalloc success\n", __func__);
	}

	HL_LOG("[HL]++++++++++++++++++");
	hl->hall_enable = 1;
	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
		{
			HL_ERR("[HL]platform_data alloc memory fail");
			goto err_alloc_mem_failed;
		}

		ret = hall_sensor_dt_parser(pdev->dev.of_node, pdata);
		if (ret < 0) {
			ret = -ENOMEM;
			goto err_alloc_pdata_mem_failed;
		}
		if (pdata) {
			hl->att_used   = pdata->att_used;
			hl->gpio_att   = pdata->gpio_att;
			hl->gpio_att_s = pdata->gpio_att_s;
		}
	}
	hl->irq_enable = 1;

	ret = hall_input_register(hl);
	if (ret)
		goto err_input_register_device_failed;

	if (gpio_is_valid(pdata->gpio_att)) {
		mt_set_gpio_dir(hl->gpio_att, 0);//in
		mt_set_gpio_mode(hl->gpio_att, 0);//gpio
	}
	if (hl->att_used > 1){
		if (gpio_is_valid(pdata->gpio_att_s)){
			mt_set_gpio_dir(hl->gpio_att_s, 0);//in
			mt_set_gpio_mode(hl->gpio_att_s, 0);//gpio
		}
	}
	platform_set_drvdata(pdev, hl);
	wake_lock_init(&hl->wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	prev_val_n = mt_get_gpio_in(hl->gpio_att);
	HL_LOG("[HL]mt get gipo dir = %d att_n = %d\n",hl->gpio_att , prev_val_n);

	/* Create singlethread workqueue */
	hl->wq_npole = create_singlethread_workqueue("hall_wq_npole");
	if (hl->wq_npole == NULL) {
		pr_err("Not able to create N-pole workqueue\n");
		ret = -ENOMEM;
		goto err_device_init_wq_pole;
	}
	INIT_WORK(&hl->work_npole_irq, hall_npole_work);

	hl->wq_spole = create_singlethread_workqueue("hall_wq_spole");
	if (hl->wq_spole == NULL) {
		pr_err("Not able to create S-pole workqueue\n");
		ret = -ENOMEM;
		goto err_device_init_wq_pole;
	}
	INIT_WORK(&hl->work_spole_irq, hall_spole_work);

	node = of_find_compatible_node(NULL, NULL, "mediatek, HALL_2-eint");
	if(node){
		irq = irq_of_parse_and_map(node, 0);

		if(prev_val_n == 1){
			ret = request_irq(irq, hall_npole_irq_thread,
			IRQF_TRIGGER_FALLING, "HALL_2-eint", hl);
		}
		else if(prev_val_n == 0){
			ret = request_irq(irq, hall_npole_irq_thread,
			IRQF_TRIGGER_RISING, "HALL_2-eint", hl);
		}
		if (ret == 0)
		{
			irq_set_irq_wake(irq, 1);
			hl->irq_n = irq;
			HL_LOG("[HL]Operate in [Interrupt] mode, irq[%d]", irq);
		}
		else
		{
			HL_ERR("[HL]Request IRQ failed, ret=%d, gpio=%d", ret, hl->gpio_att);
			goto err_request_irq_failed;
		}
		if (hl->att_used > 1) {
			prev_val_s = mt_get_gpio_in(hl->gpio_att_s);
			HL_LOG("[HL]mt get value dir = %d att_s = %d\n",hl->gpio_att_s , prev_val_s );
			node = of_find_compatible_node(NULL, NULL, "mediatek, HALL_1-eint");
			if(node){
				irq = irq_of_parse_and_map(node, 0);

				if(prev_val_s == 1){
					ret = request_irq(irq, hall_spole_irq_thread,
					IRQF_TRIGGER_FALLING, "HALL_1-eint", hl);
				}
				else if(prev_val_s == 0){
					ret = request_irq(irq, hall_spole_irq_thread,
					IRQF_TRIGGER_RISING, "HALL_1-eint", hl);
				}
				if (ret == 0)
				{
					irq_set_irq_wake(irq, 1);
					hl->irq_s = irq;
					HL_LOG("[HL]Operate in [Interrupt] mode, irq[%d]", irq);
				}
				else
				{
					free_irq(irq, hl);
					HL_ERR("[HL]Request IRQ failed, ret=%d, gpio=%d", ret, hl->gpio_att_s);
					goto err_request_irq_failed;
				}
			}

		}
	}
	hall_cover_sysfs_init();
	g_hl = hl;

	HL_LOG("[HL]------------------");
	kfree(pdata);
	return 0;
err_request_irq_failed:
err_device_init_wq_pole:
	wake_lock_destroy(&hl->wake_lock);

err_request_gpio_att_or_gpio_att_s_irq_failed:

err_input_register_device_failed:
err_alloc_pdata_mem_failed:
	if (pdev->dev.of_node)
		kfree(pdata);
err_alloc_mem_failed:
	kfree(hl);
	printk("%s:fail-----\n", __func__);
	return ret;
}

static int hall_sensor_remove(struct platform_device *pdev)
{
	struct ak_hall_data *hl = platform_get_drvdata(pdev);

	if (hl->input_dev) {
		input_unregister_device(hl->input_dev);
		input_free_device(hl->input_dev);
	}
	wake_lock_destroy(&hl->wake_lock);
	kfree(g_hl);
	return 0;
}

#ifdef CONFIG_PM
static int hall_sensor_suspend(struct device *dev)
{
	struct ak_hall_data *hl = dev_get_drvdata(dev);

	HL_LOG("enter");
	if (hl->hall_enable && hl->irq_enable) {
		if (gpio_is_valid(hl->gpio_att)) {
			disable_irq(hl->irq_n);
		}
		if ((hl->att_used > 1) && (gpio_is_valid(hl->gpio_att_s))) {
			disable_irq(hl->irq_s);
		}
		hl->irq_enable = 0;
		HL_LOG("disable_irq");
	}

	return 0;
}

static int hall_sensor_resume(struct device *dev)
{
	struct ak_hall_data *hl = dev_get_drvdata(dev);

	HL_LOG("enter");
	if (hl->hall_enable && !hl->irq_enable) {
		if (gpio_is_valid(hl->gpio_att)) {
			enable_irq(hl->irq_n);
		}
		if ((hl->att_used > 1) && (gpio_is_valid(hl->gpio_att_s))) {
			enable_irq(hl->irq_s);
		}
		hl->irq_enable = 1;
		HL_LOG("enable_irq");
	}

	return 0;
}

static const struct dev_pm_ops hall_sensor_pm_ops = {
	.suspend = hall_sensor_suspend,
	.resume  = hall_sensor_resume,
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id hall_sensor_mttable[] = {
	{ .compatible = "hall_sensor,ak8789"},
	{},
};
#else
#define hall_sensor_mttable NULL
#endif

static struct platform_driver hall_sensor_driver = {
	.probe  = hall_sensor_probe,
	.remove = hall_sensor_remove,
	.driver = {
		.name = "AK8789_HALL_SENSOR",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &hall_sensor_pm_ops,
#endif
		.of_match_table = hall_sensor_mttable,
	},
};

static void __init hall_sensor_init_async(void *unused, async_cookie_t cookie)
{
	printk("%s:\n", __func__);
	platform_driver_register(&hall_sensor_driver);
}

static int __init hall_sensor_init(void)
{
	printk("%s:\n", __func__);
	async_schedule(hall_sensor_init_async, NULL);
	return 0;
}

static void __exit hall_sensor_exit(void)
{
	platform_driver_unregister(&hall_sensor_driver);
}
module_init(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_DESCRIPTION("HTC Hall Sesnor driver");
MODULE_LICENSE("GPL");
