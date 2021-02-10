/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010 Focal tech Ltd.
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
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/regulator/consumer.h>

#include <linux/of_gpio.h>

#include "ft5x06_ts.h"

/* NOTE: support mulititouch only */
#ifdef CONFIG_ANDROID_PARANOID_NETWORK	//CONFIG_STAGING	//in linux Qt single report
#define CONFIG_FT5X0X_MULTITOUCH		1
#endif

int TOUCH_MAX_X = 1024;
int TOUCH_MAX_Y = 768;

unsigned int type = 1;

static int swap_xy = 0;
static int scal_xy = 0;

int touch_size = 0;	//0:1024*768 or 1:1280*800
/*---------------------------------------------------------
 * Chip I/O operations
 */

static int ft5x0x_i2c_rxdata(struct i2c_client *this_client, char *rxdata, int length) {
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("%s: i2c read error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_read_reg(struct i2c_client *this_client, u8 addr, u8 *pdata) {
	u8 buf[4] = { 0 };
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};
	int ret;

	buf[0] = addr;

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("read reg (0x%02x) error, %d\n", addr, ret);
	} else {
		*pdata = buf[0];
	}

	return ret;
}

static int ft5x0x_read_fw_ver(struct ft5x0x_ts_data *ts, unsigned char *val)
{
	int ret;

	*val = 0xff;
	ret = ft5x0x_read_reg(ts->client, FT5X0X_REG_FIRMID, val);
	if (*val == 0x06) {
#if 0
		swap_xy = 1;
		scal_xy = 1;
#endif
	} else {
		/* TODO: Add support for other version */
	}

	return ret;
}


/*---------------------------------------------------------
 * Touch core support
 */

static void ft5x0x_ts_report(struct ft5x0x_ts_data *ts) {
	struct ft5x0x_event *event = &ts->event;
	int x, y;
	int i = 0;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	for (i = 0; i < event->touch_point; i++) {
		if(touch_size != 1)
			event->x[i] = ts->screen_max_x - event->x[i];
		//event->y[i] = ts->screen_max_y - event->y[i];
#ifdef CONFIG_PRODUCT_SHENDAO
		event->y[i] = ts->screen_max_y - event->y[i];
#endif
		
		if (swap_xy) {
			x = event->y[i];
			y = event->x[i];
		} else {
			x = event->x[i];
			y = event->y[i];
		}

		if (scal_xy) {
			x = (x * ts->screen_max_x) / TOUCH_MAX_X;
			y = (y * ts->screen_max_y) / TOUCH_MAX_Y;
		}
	
		if(0 == touch_size)
		{
			x = ts->screen_max_x - x;
		}	

		//printk("x = %d, y = %d\n", x, y);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);

		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, event->pressure);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);

		input_mt_sync(ts->input_dev);
	}
#else
	if (event->touch_point == 1) {
		if (swap_xy) {
			x = event->y[i];
			y = event->x[i];
		} else {
			x = event->x[i];
			y = event->y[i];
		}

		if (scal_xy) {
			x = (x * ts->screen_max_x) / TOUCH_MAX_X;
			y = (y * ts->screen_max_y) / TOUCH_MAX_Y;
		}

		input_report_abs(ts->input_dev, ABS_X, x);
		input_report_abs(ts->input_dev, ABS_Y, y);
		input_report_abs(ts->input_dev, ABS_PRESSURE, event->pressure);
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif

	input_sync(ts->input_dev);
}

static void ft5x0x_ts_release(struct ft5x0x_ts_data *ts) {
#ifdef CONFIG_FT5X0X_MULTITOUCH
#if 0
	/* NOT needed for ICS */
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#endif
	input_mt_sync(ts->input_dev);
#else
	input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif

	input_sync(ts->input_dev);
}

static int ft5x0x_read_data(struct ft5x0x_ts_data *ts) {
	struct ft5x0x_event *event = &ts->event;
	u8 buf[64] = { 0 };
	int ret;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x0x_i2c_rxdata(ts->client, buf, 63);
#else
	ret = ft5x0x_i2c_rxdata(ts->client, buf, 7);
#endif
	if (ret < 0) {
		printk("%s: read touch data failed, %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ft5x0x_event));

	event->touch_point = buf[2] & 0x0F;

	if (!event->touch_point) {
		ft5x0x_ts_release(ts);
		return 1;
	}
	//printk("point = %d\n", event->touch_point);
#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
		case 10:
			event->x[9] = (s16)(buf[57] & 0x0F)<<8 | (s16)buf[58];
			event->y[9] = (s16)(buf[59] & 0x0F)<<8 | (s16)buf[60];
		case 9:
			event->x[8] = (s16)(buf[51] & 0x0F)<<8 | (s16)buf[52];
			event->y[8] = (s16)(buf[53] & 0x0F)<<8 | (s16)buf[54];
		case 8:
			event->x[7] = (s16)(buf[45] & 0x0F)<<8 | (s16)buf[46];
			event->y[7] = (s16)(buf[47] & 0x0F)<<8 | (s16)buf[48];
		case 7:
			event->x[6] = (s16)(buf[39] & 0x0F)<<8 | (s16)buf[40];
			event->y[6] = (s16)(buf[41] & 0x0F)<<8 | (s16)buf[42];
		case 6:
			event->x[5] = (s16)(buf[33] & 0x0F)<<8 | (s16)buf[34];
			event->y[5] = (s16)(buf[35] & 0x0F)<<8 | (s16)buf[36];
		case 5:
			event->x[4] = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y[4] = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
		case 4:
			event->x[3] = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y[3] = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
			//printk("x:%d, y:%d\n", event->x[3], event->y[3]);
		case 3:
			event->x[2] = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y[2] = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
			//printk("x:%d, y:%d\n", event->x[2], event->y[2]);
		case 2:
			event->x[1] = (s16)(buf[0x09] & 0x0F)<<8 | (s16)buf[0x0a];
			event->y[1] = (s16)(buf[0x0b] & 0x0F)<<8 | (s16)buf[0x0c];
			//printk("x:%d, y:%d\n", event->x[1], event->y[1]);
		case 1:
			event->x[0] = (s16)(buf[0x03] & 0x0F)<<8 | (s16)buf[0x04];
			event->y[0] = (s16)(buf[0x05] & 0x0F)<<8 | (s16)buf[0x06];
			//printk("x:%d, y:%d\n", event->x[0], event->y[0]);
			break;
		default:
			printk("%s: invalid touch data, %d\n", __func__, event->touch_point);
			return -1;
	}
#else
	if (event->touch_point == 1) {
		event->x[0] = (s16)(buf[0x03] & 0x0F)<<8 | (s16)buf[0x04];
		event->y[0] = (s16)(buf[0x05] & 0x0F)<<8 | (s16)buf[0x06];
	}
#endif

	event->pressure = 200;

	return 0;
}

static void ft5x0x_ts_pen_irq_work(struct work_struct *work) {
	struct ft5x0x_ts_data *ts = container_of(work, struct ft5x0x_ts_data, work);

	if (!ft5x0x_read_data(ts)) {
		ft5x0x_ts_report(ts);
	}

	enable_irq(ts->client->irq);
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id) {
	struct ft5x0x_ts_data *ts = dev_id;

        //printk("%s(%d)\n", __FUNCTION__, __LINE__);

	disable_irq_nosync(ts->client->irq);

	if (!work_pending(&ts->work)) {
		queue_work(ts->queue, &ts->work);
	}

	return IRQ_HANDLED;
}


/*---------------------------------------------------------
 * I2C client driver functions
 */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
#if 1
	struct ft5x0x_ts_data *ts;

	ts = container_of(handler, struct ft5x0x_ts_data, early_suspend);

	disable_irq(ts->client->irq);
	flush_workqueue(ts->queue);
	cancel_work_sync(&ts->work);
	//flush_workqueue(ts->queue);

	//ft5x0x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
#endif
#if 0
	if( IS_ERR_OR_NULL(dc33v_tp) )
	{
		printk( "error on dc33_tp regulator disable : tp_regulator is null\n");
		//return;
	}
	else
	{
		regulator_force_disable(dc33v_tp);
	}
#endif
	printk("ft5x0x_ts: suspended\n");
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts;

        ts = container_of(handler, struct ft5x0x_ts_data, early_suspend);
#if 1
	/* Wakeup: output_L --> 100ms --> output_H --> 100ms */
	enable_irq(ts->client->irq);
#endif

	printk("ft5x0x_ts: resumed\n");
}
#endif

static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct ft5x0x_i2c_platform_data *pdata;
	struct ft5x0x_ts_data *ts;
	struct input_dev *input_dev;
	unsigned char val;
	int err = -EINVAL;
	unsigned int value = 0;

	printk("**************** fun:%s, line = %d\n", __FUNCTION__, __LINE__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	//printk("****** ft5x: client irq = %d\n", client->irq);

	err = of_property_read_u32(np, "touch_type", &type);
        if (err < 0)
	{
		printk("****************** %s, line = %d(get touch_type failed!)\n", __FUNCTION__, __LINE__);
                return err;
	}
	else
	{
		printk("%s, line = %d(get touch_type success!)\n", __FUNCTION__, __LINE__);
	}

	if(0x00 == type)  //9.7
        {
                TOUCH_MAX_X = 1024;
                TOUCH_MAX_Y = 768;

#ifdef CONFIG_VT        //for Ubuntu
                touch_size = 1;
                scal_xy = 1;
#else
                touch_size = 1;//0;
                scal_xy = 1;
#endif
        }
        else if(0x01 == type) //7.0
        {
                TOUCH_MAX_X = 1280;
                TOUCH_MAX_Y = 800;
#ifdef CONFIG_VT        //for Ubuntu
                TOUCH_MAX_X = 800;//1280;
                TOUCH_MAX_Y = 1280;//800;

                scal_xy = 1;
                touch_size = 0;
#else
                touch_size = 0;//1;
#endif
        }

        if(1 == touch_size)
        {
                swap_xy = 1;
        }
        else
        {
                swap_xy = 0;
        }

	if(0x00 == type)  //9.7
	{
		ts->screen_max_x = 768;
		ts->screen_max_y = 1024;
		ts->pressure_max = 255;
	}
	else if(0x01 == type)//7.0
	{
		ts->screen_max_x = 800;
                ts->screen_max_y = 1280;
                ts->pressure_max = 255;
	}
	else if(0x03 == type)//1024x600
        {
                ts->screen_max_x = 1024;
                ts->screen_max_y = 600;
                ts->pressure_max = 255;

		client->irq = 111;
        }
	else if(0x04 == type)//5.0
        {
                ts->screen_max_x = 800;
                ts->screen_max_y = 480;
                ts->pressure_max = 255;

                client->irq = 111;
        }

	ts->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (ts->reset_gpio == -EPROBE_DEFER)
        	return ts->reset_gpio;
        if (ts->reset_gpio < 0) {
                dev_err(&client->dev, "error acquiring reset gpio: %d\n",
                         ts->reset_gpio);
                return ts->reset_gpio;
        }
	
	err = devm_gpio_request_one(&client->dev, ts->reset_gpio, 0,
                                       "reset-gpios");
        if(err) {
        	dev_err(&client->dev, "error requesting reset gpio: %d\n",err);
        	return err;
        }
	
	gpio_set_value(ts->reset_gpio, 0);
	mdelay(200);
	gpio_set_value(ts->reset_gpio, 1);
	msleep(300);

	INIT_WORK(&ts->work, ft5x0x_ts_pen_irq_work);

	ts->client = client;
	ts->gpio_irq = client->irq;
	i2c_set_clientdata(client, ts);

	ts->queue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ts->queue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ts->input_dev = input_dev;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, ts->pressure_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, FT5X0X_PT_MAX, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, ts->screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts->screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, ts->pressure_max, 0 , 0);
#endif

	input_dev->name = FT5X0X_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x12FA;
	input_dev->id.product = 0x2143;
	input_dev->id.version = 0x0100;

	err = input_register_device(input_dev);
	if (err) {
		input_free_device(input_dev);
		dev_err(&client->dev, "failed to register input device %s, %d\n",
				dev_name(&client->dev), err);
		goto exit_input_dev_alloc_failed;
	}

	msleep(3);

	unsigned int i = 0;
	printk("read fw ver %d...\n", i++);
	err = ft5x0x_read_fw_ver(ts, &val);

	if (err < 0) {
		dev_err(&client->dev, "chip not found\n");
		goto exit_irq_request_failed;
	}

	err = request_irq(client->irq, ft5x0x_ts_interrupt,
			IRQ_TYPE_EDGE_FALLING /*IRQF_TRIGGER_FALLING*/, "ft5x0x_ts", ts);
	if (err < 0) {
		dev_err(&client->dev, "Request IRQ %d failed, %d\n", client->irq, err);
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

	dev_info(&client->dev, "Firmware version 0x%02x\n", val);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;//EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ts->early_suspend.resume = ft5x0x_ts_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	enable_irq(client->irq);

	dev_info(&client->dev, "FocalTech ft5x0x TouchScreen initialized\n");
	return 0;

exit_irq_request_failed:
	input_unregister_device(input_dev);

exit_input_dev_alloc_failed:
	cancel_work_sync(&ts->work);
	destroy_workqueue(ts->queue);

exit_create_singlethread:
	i2c_set_clientdata(client, NULL);

exit_no_pdata:
	kfree(ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	dev_err(&client->dev, "probe ft5x0x TouchScreen failed, %d\n", err);

	return err;
}

static int ft5x0x_ts_remove(struct i2c_client *client) {
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

	if (client->irq) {
		free_irq(client->irq, ts);
	}

	cancel_work_sync(&ts->work);
	destroy_workqueue(ts->queue);

	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	if (ts->input_dev)
		kfree(ts->input_dev);

	kfree(ts);

	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id ft5x0x_of_match[] = {
        { .compatible = "edt,ft5x0x_ts" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ft5x0x_of_match);
#endif

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= ft5x0x_ts_remove,
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ft5x0x_of_match),
	},
};

static int __init ft5x0x_ts_init(void)
{
	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

late_initcall(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

#if 0
static int __init setup_width_height(char *str)
{
        if (strstr(str, "LDB-XGA")) {
                //printk("000000000000000000000000\n");

		type = 0;
        }
        else if(strstr(str, "VGA_8001280"))
        {
                //printk("1111111111111111111111111\n");

		type = 1;
        }
	else if(strstr(str, "VGA_1024600"))
        {
                //printk("1111111111111111111111111\n");

                type = 3;
        }
	else if(strstr(str, "VGA_800480"))
        {
                //printk("1111111111111111111111111\n");

                type = 4;
        }
	else
	{
		type = 1;
	}


        //printk("%s\n", __FUNCTION__);
}
early_param("video", setup_width_height);
#endif

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
