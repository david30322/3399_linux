/*
 *  ls_stk3x3x.c - Linux kernel modules for sensortek stk301x, stk321x and stk331x 
 *  proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2015 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include   <linux/fs.h>   
#include  <asm/uaccess.h> 
#include <linux/sensor-dev.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif
#include "linux/stk3x3x.h"

/* Driver Settings */
#define STK_POLL_ALS		/* ALS interrupt is valid only when STK_PS_INT_MODE = 1	or 4*/
// #define STK_IRS
#define STK_ALS_FIR
/*****************************************************************************/
#define ALS_DEFAULT_GAIN    128

#ifdef STK_ALS_FIR
	#define STK_FIR_LEN	8
	#define MAX_FIR_LEN 32
	
struct data_filter {
    uint32_t raw[MAX_FIR_LEN];
    int sum;
    int number;
    int idx;
}stk3x3x_fir;
#endif

static uint32_t last_als = 0;
static uint32_t last_data_c = 0; //Modify data type, Because "c_raw" may exceed 65535
static bool first_als = true;
uint8_t als_gain_level = 0;
uint16_t stk3x3x_als_gain = 128;

struct stk3x3x_data {
	uint16_t ir_code;
	uint16_t als_correct_factor;
	uint8_t alsctrl_reg;
	uint8_t psctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t state_reg;
	int		int_pin;
	uint8_t wait_reg;
	uint8_t int_reg;
#ifdef CONFIG_HAS_EARLYSUSPEND
	//struct early_suspend stk_early_suspend;
#endif	
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
#ifdef CALI_PS_EVERY_TIME	
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;
#endif	
	struct mutex io_lock;
	struct input_dev *ps_input_dev;
	int32_t ps_distance_last;
	bool ps_enabled;
	bool re_enable_ps;
	struct wake_lock ps_wakelock;	
#ifdef STK_POLL_PS		
	struct hrtimer ps_timer;	
    struct work_struct stk_ps_work;
	struct workqueue_struct *stk_ps_wq;
	struct wake_lock ps_nosuspend_wl;		
#endif
	struct input_dev *als_input_dev;
	int32_t als_lux_last;
	uint32_t als_transmittance;	
	bool als_enabled;
	bool re_enable_als;
	ktime_t ps_poll_delay;
	ktime_t als_poll_delay;
#ifdef STK_POLL_ALS		
    struct work_struct stk_als_work;
	struct hrtimer als_timer;	
	struct workqueue_struct *stk_als_wq;
#endif	
	bool first_boot;
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;	
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;
	ktime_t ps_tune0_delay;
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;
	int stk_max_min_diff;
	int stk_lt_n_ct;
	int stk_ht_n_ct;
#endif	
#ifdef STK_ALS_FIR
	struct data_filter      fir;
	atomic_t                firlength;	
#endif
	atomic_t	recv_reg;

#ifdef STK_GES		
	struct input_dev *ges_input_dev;
	int ges_enabled;
	int re_enable_ges;	
	atomic_t gesture2;
#endif	

#ifdef STK_QUALCOMM_POWER_CTRL
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
#endif	
	uint8_t pid;
	uint8_t	p_wv_r_bd_with_co;
	uint32_t als_code_last;
};

static struct stk3x3x_data *stk_als_data;

const int ALS_LEVEL[] = {100, 500, 1000, 1600, 2250, 3200, 6400, 12800, 20000, 26000};

/*****************************************************************************/

#ifdef STK_ALS_FIR
static void stk3x3x_als_bubble_sort(uint32_t* sort_array, uint8_t size_n)
{
	int i, j, tmp;

	for (i = 1; i < size_n; i++)
	{
		tmp = sort_array[i];
		j = i - 1;

		while (j >= 0 && sort_array[j] > tmp)
		{
			sort_array[j + 1] = sort_array[j];
			j = j - 1;
		}

		sort_array[j + 1] = tmp;
	}
}
#endif


#if 0
static int32_t stk3x3x_set_als_thd_l(struct i2c_client *client, uint16_t thd_l)
{
	unsigned char val[3];
	int ret;
	
	val[0] = STK_THDL1_ALS_REG;
	val[1] = (thd_l & 0xFF00) >> 8;
	val[2] = thd_l & 0x00FF;
	ret = sensor_tx_data(client, val, 3);
	
	// ret = sensor_write_reg(client, STK_THDL1_ALS_REG, );
	// if(ret)
		// printk("%s:fail to active sensor\n",__func__);

	return ret;		
}

static int32_t stk3x3x_set_als_thd_h(struct i2c_client *client, uint16_t thd_h)
{
	unsigned char val[2];
	int ret;
	
	val[0] = STK_THDH1_ALS_REG;
	val[1] = (thd_h & 0xFF00) >> 8;
	val[2] = thd_h & 0x00FF;
	ret = sensor_tx_data(client, val, 3);	
	// ret = sensor_write_reg(client, STK_THDL1_ALS_REG, );
	// if(ret)
		// printk("%s:fail to active sensor\n",__func__);	
	return ret;	
}
#endif

static int stk_light_sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);	
#ifndef STK_POLL_ALS
    if (enable)
	{				
        stk3x3x_set_als_thd_h(client, 0x0000);
        stk3x3x_set_als_thd_l(client, 0xFFFF);
	}	
#endif	
	sensor->ops->ctrl_data = (uint8_t)((sensor->ops->ctrl_data) & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK)));

	if(enable){
		sensor->ops->ctrl_data |= STK_STATE_EN_ALS_MASK;
#ifdef STK_ALS_FIR
	        memset(&stk_als_data->fir, 0x00, sizeof(stk_als_data->fir));  
       		atomic_set(&stk_als_data->firlength, STK_FIR_LEN);
	}   
#endif	
	else if (sensor->ops->ctrl_data & STK_STATE_EN_PS_MASK){		
		sensor->ops->ctrl_data |= STK_STATE_EN_WAIT_MASK;
	}		

	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);

	if(!enable)
	{
	    first_als = true;
		// sensor->ops->report(sensor->client);
	}
	stk_als_data->als_enabled = enable?true:false;
    printk("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",
        __func__,
        sensor->ops->ctrl_reg,
        sensor->ops->ctrl_data,
        enable);
	return result;
}

static int32_t stk3x3x_check_pid(struct i2c_client *client)
{
	char  reg_val;

    reg_val = sensor_read_reg(client, STK_PDT_ID_REG);
    if (reg_val < 0)
	{
		printk("stk %s PID error\n", __func__);
		return -1;
	}
    printk(KERN_INFO "%s: PID=0x%x\n", __func__, reg_val);

	return 0;
}

static int stk_light_sensor_init(struct i2c_client *client)
{
	int res = 0;
    int reg_num, i;
	
	printk("stk %s init ...\n", __func__);
	stk_als_data = kzalloc(sizeof(struct stk3x3x_data),GFP_KERNEL);
	if(!stk_als_data)
	{
		printk(KERN_ERR "%s: failed to allocate stk3x3x_data\n", __func__);
		return -ENOMEM;
	}	

	res = stk3x3x_check_pid(client);
	if(res < 0)
    {   
        printk(KERN_ERR "%s: stk3x3x_check_pid fail\n", __func__);
		goto EXIT_ERR;
    }

	res = sensor_write_reg(client, STK_SW_RESET_REG, 0x0);
	if(res < 0)
    {   
        printk(KERN_ERR "%s: stk3x3x SWR fail\n", __func__);
		goto EXIT_ERR;
    }
	
	usleep_range(13000, 15000);	
	
    reg_num = sizeof(stk3x3x_config_table)/sizeof(stk3x3x_config_table[0]);
    for(i=0;i<reg_num;i++)
    {
        res = sensor_write_reg(client, stk3x3x_config_table[i].address,stk3x3x_config_table[i].value);
        if(res < 0)
            {
                printk("stk %s sensor_write_reg err \n", __func__);	
		        goto EXIT_ERR;
            }
    }

#ifndef STK_POLL_ALS	
	value = STK_INT_REG;
	res = sensor_rx_data(client, value, 1);	
	if(res)
	{
		printk("%s:line=%d,error=%d\n",__func__,__LINE__, res);
		return res;
	}		

	value |= STK_INT_ALS;
	res = sensor_write_reg(client, STK_INT_REG, value);
	if(res <= 0)
		goto EXIT_ERR;	
#endif
	
	stk_als_data->als_code_last = 0;

#ifdef STK_ALS_FIR
        memset(&stk_als_data->fir, 0x00, sizeof(stk_als_data->fir));  
        atomic_set(&stk_als_data->firlength, STK_FIR_LEN);   
#endif

	
	printk("stk %s init successful \n", __func__);
	return 0;
	
EXIT_ERR:
	printk(KERN_ERR "stk init fail dev: %d\n", res);
	return res;

}



static int stk_light_report_abs_value(struct input_dev *input, int data)
{
            input_report_abs(input, ABS_MISC, data);
            input_sync(input);
            return data;

}

#if 0
static int stk_allreg(struct i2c_client *client)
{
	uint8_t ps_reg[0x22];
	int cnt = 0;	
	
	for(cnt=0;cnt<0x20;cnt++)
	{
		ps_reg[cnt] = sensor_read_reg(client, cnt);
		if(ps_reg[cnt] < 0)
		{
			printk("%s fail \n", __func__);	
			return -EINVAL;
		}
		printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
	}	
	return 0;
}
#endif

static int stk3x3x_als_cal(struct i2c_client *client, int als_data)
{
#ifdef STK_ALS_FIR
	int index;   
	int firlen = atomic_read(&stk_als_data->firlength);   
#endif	

    printk("%s: get_value %d\n",__func__, als_data);
	stk_als_data->als_code_last = als_data;	
#ifdef STK_ALS_FIR
	if(stk_als_data->fir.number < firlen)
	{                
		stk_als_data->fir.raw[stk_als_data->fir.number] = als_data;
		stk_als_data->fir.sum += als_data;
		stk_als_data->fir.number++;
		stk_als_data->fir.idx++;
	}
	else
	{
		index = stk_als_data->fir.idx % firlen;
		stk_als_data->fir.sum -= stk_als_data->fir.raw[index];
		stk_als_data->fir.raw[index] = als_data;
		stk_als_data->fir.sum += als_data;
		stk_als_data->fir.idx++;
		als_data = stk_als_data->fir.sum/firlen;
	}	
#endif		
	return als_data;
}

static uint32_t stk3x3x_als_fir(struct i2c_client *client, uint32_t als_raw_data)
{
#ifdef STK_ALS_FIR
        int fir_index = 0;   
        uint32_t mid_als = 0;
        uint32_t cpraw[STK_FIR_LEN] = { 0 };
#endif
        printk("%s: als_value %d\n",__func__, als_raw_data);
        //stk3a6x_als_data->als_code_last = als_raw_data;

#ifdef STK_ALS_FIR
    memset(cpraw, 0x00, sizeof(cpraw) / sizeof(cpraw[0]));
    if(stk3x3x_fir.number < STK_FIR_LEN)
    {                
        stk3x3x_fir.raw[stk3x3x_fir.number] = als_raw_data;
        stk3x3x_fir.number++;
        stk3x3x_fir.idx++;
    }
    else
    {
        fir_index = stk3x3x_fir.idx % stk3x3x_fir.number;
        stk3x3x_fir.raw[fir_index] = als_raw_data;
        stk3x3x_fir.idx++;
        
        //memcpy(cpraw, stk3x3x_fir.raw, sizeof(stk3x3x_fir.raw));
        memcpy(cpraw, stk3x3x_fir.raw,  sizeof(cpraw));

#ifdef STK_DEBUG_PRINTF
        printk("%s: :~~~~cpraw_CP1 = %u %u %u %u %u\n",__func__, cpraw[0], cpraw[1], cpraw[2], cpraw[3], cpraw[4]);  
        printk("%s: :~~~~sizeof(stk3x3x_fir.raw)=%ld  sizeof(cpraw)=%ld sizeof(cpraw[0])=%ld\n",
            __func__,sizeof(stk3x3x_fir.raw), sizeof(cpraw), sizeof(cpraw[0]));  
#endif
        stk3x3x_als_bubble_sort(cpraw, sizeof(cpraw) / sizeof(cpraw[0]));
        mid_als = cpraw[STK_FIR_LEN / 2];
        als_raw_data = mid_als;
#ifdef STK_DEBUG_PRINTF
        printk("%s: :~~~~cpraw_CP2 = %u %u %u %u %u\n",__func__, cpraw[0], cpraw[1], cpraw[2], cpraw[3], cpraw[4]);  
#endif
    }

#endif

    return als_raw_data;
}


//add auto gain david 20210514
static bool stk3x3x_set_als_gain(struct i2c_client *client, uint16_t level)
{
    int ret = 0;
    uint8_t alsctrl_reg, gainctrl_reg, gain;
    uint8_t rx_buf[1] = {0};

    //printk("%s: starting\n", __func__);
    //ret |= oppo_i2c_read_byte(ALS, STK_ALSCTRL_REG, rx_buf);
    rx_buf[0] = sensor_read_reg(client, STK_ALSCTRL_REG);
    alsctrl_reg = rx_buf[0];
    //ret |= oppo_i2c_read_byte(ALS, STK_GAINCTRL_REG, rx_buf);
    rx_buf[0] = sensor_read_reg(client, STK_GAINCTRL_REG);
    gainctrl_reg = rx_buf[0];

    if (level == 0) {
        // Highest gain for low light :  ALS Gain X128 & C Gain X128
        // Reg[0x02] don't care
        // Write Reg[0x4E] = 0x06, GAIN_ALS_DX128 = 1, GAIN_C_DX128 = 1
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN64;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x06; //als&c data gain x128
        gain = 128;
        //printk("%s level =%d\n", __func__, level);
    } else if (level == 1) {
        // Middle gain for middle light :  ALS Gain X16 & C Gain X16
        // Write Reg[0x02] = 0x21, ALS IT = 50ms, GAIN_ALS = 2' b10
        // Write Reg[0x4E] = 0x20, GAIN_ALS_DX128 = 0, GAIN_C_DX128 = 0, GAIN_C = 2' b10
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN16;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x20; //c data gain x16
        gain = 16;
        //printk("%s level =%d\n", __func__, level);
    } else if (level == 2) {
        //Lowest gain for high light :  ALS Gain X1 & C Gain X1
        //Write Reg[0x02] = 0x01, ALS IT = 50ms, GAIN_ALS = 2' b00
        //Write Reg[0x4E] = 0x00, GAIN_ALS_DX128 = 0, GAIN_C_DX128 = 0, GAIN_C = 2' b00
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN1;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x00; //c data gain x1
        gain = 1;
        //printk("%s level =%d\n", __func__, level);
    } else {
        printk("%s level =%d\n", __func__, level);
        return false;
    }
//sensor_write_reg(client, STK_INT_REG, value);
    ret = sensor_write_reg(client, STK_ALSCTRL_REG, alsctrl_reg);
    if(ret < 0)
    {
        printk("stk %s sensor_write_reg err \n", __func__);
    }
    ret = sensor_write_reg(client, STK_GAINCTRL_REG, gainctrl_reg);
    if(ret < 0)
    {
        printk("stk %s sensor_write_reg err \n", __func__);
    }
    rx_buf[0] = sensor_read_reg(client, 0x5F);
    ret = sensor_write_reg(client, 0x5F, (rx_buf[0] | 0x01));
    if(ret < 0)
    {
        printk("stk %s sensor_write_reg err \n", __func__);
    }

    stk3x3x_als_gain = gain;
    printk("%s level = %d 02 = 0x%x 4E = 0x%x %d\n", __func__, level, alsctrl_reg, gainctrl_reg , gain);

    return true;
}

static bool stk3x3x_als_auto_gain(struct i2c_client *client, uint16_t *als_data)
{
    bool result = false;

    //printk("%s: starting\n", __func__);
    if (((als_data[0]) > 64000 || (als_data[4] > 64000)) &&
        (als_gain_level < 2)) {
        // Reduce gain
        als_gain_level++;
        result = stk3x3x_set_als_gain(client, als_gain_level);
    } else if (((als_data[0] < 3000) && (als_data[4] < 3000)) &&
        (als_gain_level > 0)) {
        // Raise gain
        als_gain_level--;
        result = stk3x3x_set_als_gain(client, als_gain_level);
    }
    return result;
}
//add auto gain david 20210514 end

static int stk3x3x_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
    bool auto_gain = false;
	int flag_data = 0;
    uint8_t count = 0;
    uint16_t als_data[5] = {0};//als/r/g/b/c
	unsigned char buffer[10] = {0};
	char index = 0;
    int als = 0.0;
    uint32_t report_lux1;
    uint32_t c_raw = 0;
    uint32_t g_raw = 0;

	if(sensor->ops->read_len < 2)	//sensor->ops->read_len = 1
	{
		printk("%s:lenth is error,len=%d\n",__func__,sensor->ops->read_len);
		return -1;
	}
	flag_data = sensor_read_reg(client, STK_FLAG_REG);
	if(flag_data < 0)
	{
		printk("stk %s read STK_FLAG_REG, ret=%d\n", __func__, flag_data);
		return flag_data;
	}

	if(!(flag_data & STK_FLG_ALSDR_MASK))
		return 0;
		
	buffer[0] = STK_DATA1_ALS_REG;
    result = sensor_rx_data(client, buffer, 10);
	if (result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
    for(count = 0; count < (sizeof(als_data) / sizeof(als_data[0])); count++) {
        *(als_data + count) = (*(buffer + (2 * count)) << 8 | (* (buffer + (2 * count + 1))));
    }
    
    if (!first_als) {
        if ((flag_data & STK_FLG_ALSDR_MASK)) {
            auto_gain = stk3x3x_als_auto_gain(client, als_data);
        }
    } else {
        printk("%s, first als data\n", __func__);
        first_als = false;
    }

    
    if (auto_gain) {
        // last data
        als = last_als;
        c_raw = last_data_c;
    } else {
        als = als_data[0] * 128 / stk3x3x_als_gain; // als data
        g_raw = als_data[2] * 128 / stk3x3x_als_gain; // als data
        c_raw = als_data[4] * 128 / stk3x3x_als_gain;// c data
        //todo
        //als = stk3x3x_get_als_lux(als, c_raw);
    }

    last_als = als;
    last_data_c = c_raw;

    report_lux1 = stk3x3x_als_cal(client, als);
#ifdef STK_FIR
    report_lux2 = stk3x3x_als_fir(client, als);
#endif
    //printk("%s: lux_average=%u lux_mid=%u\n",__func__, report_lux1, report_lux2);
	printk("%s: lux als raw[0]-[4] gain= %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
	    __func__, report_lux1, als, als_data[0], als_data[1], als_data[2], als_data[3], als_data[4], stk3x3x_als_gain);

    //index = stk_light_report_abs_value(sensor->input_dev, als);	
	index = stk_light_report_abs_value(sensor->input_dev, report_lux1);	

	return result;
}

struct sensor_operate light_stk3x3x_ops = {
	.name				= "ls_stk3x3x",
	.type				= SENSOR_TYPE_LIGHT,	//sensor type and it should be correct
	.id_i2c				= LIGHT_ID_STK3X3X,		//i2c id number
	.read_reg			= STK_DATA1_ALS_REG,			//read data
	.read_len			= 2,				//data length
	.id_reg				= 0x3E,//SENSOR_UNKNOW_DATA,		//read device id from this register
	.id_data 			= 0x51,//SENSOR_UNKNOW_DATA,		//device id
	.precision			= 16,				//16 bits
	.ctrl_reg 			= STK_STATE_REG,			//enable or disable 
	.int_status_reg 	= SENSOR_UNKNOW_DATA,			//intterupt status register
	.range				= {2,65535},		//range
	.brightness         ={5,255},     //brightness	
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,		
	.active				= stk_light_sensor_active,	
	.init				= stk_light_sensor_init,
	.report				= stk3x3x_report_value,
};

static struct sensor_operate *light_get_ops(void)
{
	return &light_stk3x3x_ops;
}

static int __init light_stk3x3x_init(void)
{
	struct sensor_operate *ops = light_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, light_get_ops);
	return result;
}

static void __exit light_stk3x3x_exit(void)
{
	struct sensor_operate *ops = light_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, light_get_ops);
}


module_init(light_stk3x3x_init);
module_exit(light_stk3x3x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x3x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

