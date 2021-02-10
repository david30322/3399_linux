/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/of_gpio.h>

#define DEVICE_NAME				"buzzer_ctl"
#define DRIVER_NAME "buzzer_ctl"


//#define BUZZER_GPIO			100//IMX_GPIO_NR(1, 19)

uint32_t BUZZER_GPIO = 0;


static int itop6x_buzzer_open(struct inode *inode, struct file *file) {
		return 0;
}

static int itop6x_buzzer_close(struct inode *inode, struct file *file) {
	return 0;
}

static long itop6x_buzzer_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	printk("%s: cmd = %d\n", __FUNCTION__, cmd);
	switch(cmd) {
		case 0:
			gpio_set_value(BUZZER_GPIO, 0);
			break;
		case 1:
			gpio_set_value(BUZZER_GPIO, 1);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static ssize_t itop6x_buzzer_write(struct file *filp, char *buffer, size_t count, loff_t *ppos)
{
	char str[20];

	memset(str, 0, 20);

	if(copy_from_user(str, buffer, count))
	{
		printk("Error\n");

		return -EINVAL;
	}

	printk("%s", str);
#if 1
	if(!strncmp(str, "1", 1))
		gpio_set_value(BUZZER_GPIO, 1);
	else
		gpio_set_value(BUZZER_GPIO, 0);
#endif
	return count;
}

static struct file_operations itop6x_buzzer_ops = {
	.owner			= THIS_MODULE,
	.open			= itop6x_buzzer_open,
	.release		= itop6x_buzzer_close, 
	.unlocked_ioctl	= itop6x_buzzer_ioctl,
	.write			= itop6x_buzzer_write,
};

static struct miscdevice itop6x_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &itop6x_buzzer_ops,
};

static int itop6x_buzzer_probe(struct platform_device *pdev)
{
	int ret;

	struct device_node *np = pdev->dev.of_node;

	BUZZER_GPIO = of_get_gpio(np, 0);
	if (BUZZER_GPIO == -EPROBE_DEFER)
                return BUZZER_GPIO;
        if (BUZZER_GPIO < 0) {
                dev_err(&pdev->dev, "error acquiring buzzer gpio: %d\n", BUZZER_GPIO);
                return BUZZER_GPIO;
        }

        ret = devm_gpio_request_one(&pdev->dev, BUZZER_GPIO, 0, "buzzer-gpio");
        if(ret) {
                dev_err(&pdev->dev, "error requesting buzzer gpio: %d\n", ret);
                return ret;
        }

	gpio_direction_output(BUZZER_GPIO, 0);

	ret = misc_register(&itop6x_misc_dev);

	printk(DEVICE_NAME "\tinitialized\n");

	return 0;
}

static int itop6x_buzzer_remove (struct platform_device *pdev)
{
	misc_deregister(&itop6x_misc_dev);
	gpio_free(BUZZER_GPIO);	

	return 0;
}

static int itop6x_buzzer_suspend (struct platform_device *pdev, pm_message_t state)
{
	printk("led_ctl suspend:power off!\n");
	return 0;
}

static int itop6x_buzzer_resume (struct platform_device *pdev)
{
	printk("led_ctl resume:power on!\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id buzzer_of_match[] = {
        { .compatible = "itop_buzzer" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, buzzer_of_match);
#endif

static struct platform_driver itop6x_buzzer_driver = {
	.probe = itop6x_buzzer_probe,
	.remove = itop6x_buzzer_remove,
	.suspend = itop6x_buzzer_suspend,
	.resume = itop6x_buzzer_resume,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(buzzer_of_match),
	},
};

static int __init itop6x_buzzer_dev_init(void) {
	return platform_driver_register(&itop6x_buzzer_driver);
}

static void __exit itop6x_buzzer_dev_exit(void) {
	platform_driver_unregister(&itop6x_buzzer_driver);
}

module_init(itop6x_buzzer_dev_init);
module_exit(itop6x_buzzer_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("TOPEET Inc.");
MODULE_DESCRIPTION("I.MX6 BUZZER Driver");
