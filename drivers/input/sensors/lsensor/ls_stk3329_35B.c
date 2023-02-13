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
#include "linux/stk3329_35B.h"

#define DRIVER_VERSION  "20220318_V0.1"

/* Driver Settings */
#define STK_POLL_ALS		/* ALS interrupt is valid only when STK_PS_INT_MODE = 1	or 4*/
// #define STK_IRS
//#define STK_DEBUG_PRINTF
#define STK_ALS_FIR
#define PROXIMITY_ID_I2C 2

/*****************************************************************************/
/* Define Register Map */
#define STK_STATE_REG 			0x00
#define STK_ALSCTRL_REG 		0x02
#define STK_INT_REG 			0x04
#define STK_WAIT_REG 			0x05
#define STK_THDH1_PS_REG 		0x06
#define STK_THDH2_PS_REG 		0x07
#define STK_THDL1_PS_REG 		0x08
#define STK_THDL2_PS_REG 		0x09
#define STK_THDH1_ALS_REG 		0x0A
#define STK_THDH2_ALS_REG 		0x0B
#define STK_THDL1_ALS_REG 		0x0C
#define STK_THDL2_ALS_REG 		0x0D
#define STK_FLAG_REG 			0x10
#define STK_DATA1_PS_REG	 	0x11
#define STK_DATA2_PS_REG 		0x12
#define STK_DATA1_ALS_REG 		0x13
#define STK_DATA2_ALS_REG 		0x14
#define STK_DATA1_IR_REG 		0x17
#define STK_DATA2_IR_REG 		0x18
#define STK_PDT_ID_REG 			0x3E
#define STK_RSRVD_REG 			0x3F
#define STK_SW_RESET_REG		0x80	

#define STK_STATE_EN_IRS_MASK	0x80
#define STK_STATE_EN_AK_MASK	0x40
#define STK_STATE_EN_ASO_MASK	0x20
#define STK_STATE_EN_IRO_MASK	0x10
#define STK_STATE_EN_WAIT_MASK	0x04
#define STK_STATE_EN_ALS_MASK	0x02

#define STK_FLG_ALSDR_MASK		0x80
#define STK_FLG_ALSINT_MASK		0x20
#define STK_FLG_OUI_MASK			0x04
#define STK_FLG_IR_RDY_MASK		0x02
#define STK_FLG_NF_MASK			0x01

#define STK_INT_ALS				0x08

#define STK_IRC_MAX_ALS_CODE	20000
#define STK_IRC_MIN_ALS_CODE	25
#define STK_IRC_MIN_IR_CODE		50
#define STK_IRC_ALS_DENOMI		2		
#define STK_IRC_ALS_NUMERA		5
#define STK_IRC_ALS_CORREC		850

#define STK_IRS_IT_REDUCE			2
#define STK_ALS_READ_IRS_IT_REDUCE	5
#define STK_ALS_THRESHOLD			30

/*****************************************************************************/
#define STK3310SA_PID		0x17
#define STK3311SA_PID		0x1E
#define STK3329_35B_PID     0x15
/*****************************************************************************/

#ifdef STK_ALS_FIR
	#define STK_FIR_LEN	5
	#define MAX_FIR_LEN 32
	
struct data_filter {
    uint32_t raw[MAX_FIR_LEN];
    uint32_t sum;
    int number;
    int idx;
};
#endif


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
#ifdef STK_IRS
	int als_data_index;
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

const int ALS_LEVEL[] = {100, 1600, 2250, 3200, 6400, 12800, 26000};

static struct stk3x3x_platform_data stk3x3x_pfdata={ 
  .state_reg = 0x0,    /* disable all */ 
  .psctrl_reg = 0x33,    /* ps_persistance=1, ps_gain=64X, PS_IT=0.391ms */ 
  .alsctrl_reg = 0x32, 	/* als_persistance=1, als_gain=64X, ALS_IT=100ms */
  .ledctrl_reg = 0xC0,   /* 100mA IRDR, 64/64 LED duty */ 
  .wait_reg = 0x1F,    /* 50 ms */   
  .ps_thd_h = 5, 
  .ps_thd_l = 0, 
  //.int_pin = sprd_3rdparty_gpio_pls_irq,  
  .transmittance = 500, 
  .stk_max_min_diff = 30,
  .stk_lt_n_ct = 50,
  .stk_ht_n_ct = 60,  
};

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

static int light_sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	// int status = 0;
	// char buffer[3] = {0};
	// int high = 0x80, low = 0x60;
	//int ret = 0;
	
	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);	
#ifndef STK_POLL_ALS
    if (enable)
	{				
        stk3x3x_set_als_thd_h(client, 0x0000);
        stk3x3x_set_als_thd_l(client, 0xFFFF);
	}	
	#ifdef STK_IRS
	if(enable && !(sensor->ops->ctrl_data & STK_STATE_EN_PS_MASK))
	{
		ret = stk3x3x_get_ir_reading(stk_als_data, STK_IRS_IT_REDUCE);
		if(ret > 0)
			stk_als_data->ir_code = ret;
	}
	#endif	
#endif	
	sensor->ops->ctrl_data = (uint8_t)((sensor->ops->ctrl_data) & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK))); 

	if(enable)
		sensor->ops->ctrl_data |= STK_STATE_EN_ALS_MASK;	
	else if (sensor->ops->ctrl_data & STK_STATE_EN_PS_MASK)		
		sensor->ops->ctrl_data |= STK_STATE_EN_WAIT_MASK;		
	printk("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);

	if(enable)
	{
#ifdef STK_IRS
		stk_als_data->als_data_index = 0;
#endif		
		// sensor->ops->report(sensor->client);
	}
	stk_als_data->als_enabled = enable?true:false;
    
#ifdef STK_ALS_FIR
        memset(&stk_als_data->fir, 0x00, sizeof(stk_als_data->fir));  
        atomic_set(&stk_als_data->firlength, STK_FIR_LEN);   
#endif
    
	return result;
}

static int32_t stk3x3x_check_pid(struct i2c_client *client)
{
	char value[2] = {0}, pid_msb;
	int result;
	
	stk_als_data->p_wv_r_bd_with_co = 0;
	
	value[0] = STK_PDT_ID_REG;
	result = sensor_rx_data(client, value, 2);	
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}	
	
	printk(KERN_INFO "%s: PID=0x%x, RID=0x%x\n", __func__, value[0], value[1]);
	stk_als_data->pid = value[0];
	
	if(value[0] == STK3311WV_PID)
		stk_als_data->p_wv_r_bd_with_co |= 0b100;
	if(value[1] == 0xC3)
		stk_als_data->p_wv_r_bd_with_co |= 0b010;
		
	// if(stk3x3x_read_otp25(stk_als_data) == 1)
	// {
		// stk_als_data->p_wv_r_bd_with_co |= 0b001;
	// }
	printk(KERN_INFO "%s: p_wv_r_bd_with_co = 0x%x\n", __func__, stk_als_data->p_wv_r_bd_with_co);	
	
	pid_msb = value[0] & 0xF0;
	switch(pid_msb)
	{
	case 0x10:
	case 0x20:
	case 0x30:
	case 0x50:	
		return 0;
	default:
		printk(KERN_ERR "%s: invalid PID(%#x)\n", __func__, value[0]);	
		return -1;
	}
	return 0;
}

static int light_sensor_init(struct i2c_client *client)
{
	int res = 0;
	
	printk("stk %s init ...\n", __func__);
	stk_als_data = kzalloc(sizeof(struct stk3x3x_data),GFP_KERNEL);
	if(!stk_als_data)
	{
		printk(KERN_ERR "%s: failed to allocate stk3x3x_data\n", __func__);
		return -ENOMEM;
	}	
	
	res = sensor_write_reg(client, STK_WAIT_REG, 0x1F);
	if(res < 0)
		goto EXIT_ERR;

	res = sensor_read_reg(client, STK_WAIT_REG);
	if(res != 0x1F)
	{
		printk("stk %s i2c test error\n", __func__);		
		goto EXIT_ERR;
	}
	
	res = sensor_write_reg(client, STK_SW_RESET_REG, 0x0);
	if(res < 0)
		goto EXIT_ERR;
	
	usleep_range(13000, 15000);	
	
	res = stk3x3x_check_pid(client);
	if(res < 0)
		goto EXIT_ERR;
	
	res = sensor_write_reg(client, STK_STATE_REG, stk3x3x_pfdata.state_reg);
	if(res < 0)
		goto EXIT_ERR;

	res = sensor_write_reg(client, STK_PSCTRL_REG, stk3x3x_pfdata.psctrl_reg);
	if(res < 0)
		goto EXIT_ERR;

	res = sensor_write_reg(client, STK_ALSCTRL_REG, stk3x3x_pfdata.alsctrl_reg);
	if(res < 0)
		goto EXIT_ERR;
	
	if(stk_als_data->pid == STK3310SA_PID || stk_als_data->pid == STK3311SA_PID)
		stk3x3x_pfdata.ledctrl_reg &= 0x3F;	
	res = sensor_write_reg(client, STK_LEDCTRL_REG, stk3x3x_pfdata.ledctrl_reg);
	if(res < 0)
		goto EXIT_ERR;

	res = sensor_write_reg(client, STK_WAIT_REG, stk3x3x_pfdata.wait_reg);
	if(res < 0)
		goto EXIT_ERR;	

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
	
	printk("stk %s init successful \n", __func__);
	return 0;
	
EXIT_ERR:
	printk(KERN_ERR "stk init fail dev: %d\n", res);
	return res;

}



static void light_report_abs_value(struct input_dev *input, uint32_t data)
{
            input_report_abs(input, ABS_MISC, data);
            input_sync(input);
            //return data;
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

static int32_t stk3x3x_set_irs_it_slp(struct i2c_client *client, uint16_t *slp_time, int32_t ials_it_reduce)
{
	int irs_alsctrl;
	int32_t ret;
	
	irs_alsctrl = (stk3x3x_pfdata.alsctrl_reg & 0x0F) - ials_it_reduce;
	switch(irs_alsctrl)
	{
		case 2:
			*slp_time = 1;
			break;			
		case 3:
			*slp_time = 2;
			break;	
		case 4:
			*slp_time = 3;
			break;	
		case 5:
			*slp_time = 6;
			break;
		case 6:
			*slp_time = 12;
			break;
		case 7:
			*slp_time = 24;			
			break;
		case 8:
			*slp_time = 48;			
			break;
		case 9:
			*slp_time = 96;			
			break;				
		case 10:
			*slp_time = 192;
			break;				
		default:
			printk(KERN_ERR "%s: unknown ALS IT=0x%x\n", __func__, irs_alsctrl);
			ret = -EINVAL;	
			return ret;
	}
	irs_alsctrl |= (stk3x3x_pfdata.alsctrl_reg & 0xF0);
	ret = sensor_write_reg(client, STK_ALSCTRL_REG, irs_alsctrl);
	if(ret <= 0)
		return ret;
	return 0;
}

static int stk3x3x_get_ir_reading(struct i2c_client *client, int32_t als_it_reduce)
{
	int res = 0;	
    int32_t word_data, ret;
	int w_reg, retry = 0;	
	uint16_t irs_slp_time = 100;
	char buffer[2] = {0};
	
	ret = stk3x3x_set_irs_it_slp(client, &irs_slp_time, als_it_reduce);
	if(ret < 0)
		return ret;
	
	w_reg = sensor_read_reg(client, STK_STATE_REG);
	if(w_reg <= 0)
	{
		printk("stk %s i2c error(%d)\n", __func__, w_reg);
		return ret;
	}	

	w_reg |= STK_STATE_EN_IRS_MASK;		
	res = sensor_write_reg(client, STK_STATE_REG, w_reg);
	if(res <= 0)
		return res;
	
	msleep(irs_slp_time);		
	do
	{
		usleep_range(3000, 4000);
		w_reg = sensor_read_reg(client, STK_FLAG_REG);
		if(w_reg <= 0)
			return w_reg;
		retry++;
	}while(retry < 10 && ((w_reg & STK_FLG_IR_RDY_MASK) == 0));
	
	if(retry == 10)
	{
		printk(KERN_ERR "%s: ir data is not ready for a long time\n", __func__);
		return -EINVAL;
	}

	w_reg &= (~STK_FLG_IR_RDY_MASK);
	res = sensor_write_reg(client, STK_FLAG_REG, w_reg);
	if(res <= 0)
		return res;
	
	buffer[0] = STK_DATA1_IR_REG;
	res = sensor_rx_data(client, buffer, 2);	
	if(res)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return res;
	}
	word_data = ((buffer[0]<<8) | buffer[1]);	
	printk(KERN_INFO "%s: ir=%d\n", __func__, word_data);
	
	res = sensor_write_reg(client, STK_ALSCTRL_REG, stk3x3x_pfdata.alsctrl_reg);
	if(res <= 0)
		return res;
	
	return word_data;
}
#ifdef STK_IRS	
static int stk_als_ir_skip_als(struct i2c_client *client, struct sensor_private_data *sensor)
{
	int ret;
	unsigned char buffer[2] = {0};	
	
	if(stk_als_data->als_data_index < 60000)
		stk_als_data->als_data_index++;
	else
		stk_als_data->als_data_index = 0;
	
	if(	stk_als_data->als_data_index % 10 == 1)
	{
		buffer[0] = STK_DATA1_ALS_REG;
		ret = sensor_rx_data(client, buffer, 2);	
		if(ret)
		{
			printk("%s:line=%d,error=%d\n",__func__,__LINE__, ret);
			return ret;
		}
		return 1;
	}
	return 0;
}
#endif
static uint32_t stk3x3x_als_cal(struct i2c_client *client, uint16_t als_data)
{
#ifdef STK_ALS_FIR
	int index;   
	//int firlen = atomic_read(&stk_als_data->firlength);   
#endif	

    printk("%s: get_value %d\n",__func__, als_data);
	stk_als_data->als_code_last = als_data;	
#ifdef STK_ALS_FIR
	if(stk_als_data->fir.number < STK_FIR_LEN)
	{                
		stk_als_data->fir.raw[stk_als_data->fir.number] = als_data;
		stk_als_data->fir.sum += als_data;
		stk_als_data->fir.number++;
		stk_als_data->fir.idx++;
	}
	else
	{
		index = stk_als_data->fir.idx % STK_FIR_LEN;
		stk_als_data->fir.sum -= stk_als_data->fir.raw[index];
		stk_als_data->fir.raw[index] = als_data;
		stk_als_data->fir.sum += als_data;
		stk_als_data->fir.idx++;
		als_data = stk_als_data->fir.sum/STK_FIR_LEN;
	}	
#endif		
	return als_data;
}


#ifdef STK_IRS
static void stk_als_ir_get_corr(int32_t als)
{
	int32_t als_comperator;
	
	if(stk_als_data->ir_code)
	{
		stk_als_data->als_correct_factor = 1000;
		if(als < STK_IRC_MAX_ALS_CODE && als > STK_IRC_MIN_ALS_CODE && 
			stk_als_data->ir_code > STK_IRC_MIN_IR_CODE)
		{
			als_comperator = als * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(stk_als_data->ir_code > als_comperator)
				stk_als_data->als_correct_factor = STK_IRC_ALS_CORREC;
		}
#ifdef STK_DEBUG_PRINTF				
		printk(KERN_INFO "%s: als=%d, ir=%d, als_correct_factor=%d", __func__, 
						als, stk_als_data->ir_code, stk_als_data->als_correct_factor);
#endif		
		stk_als_data->ir_code = 0;
	}	
	return;
}

static int stk_als_ir_run(struct i2c_client *client)
{
	int ret;
	
	if(stk_als_data->als_data_index % 10 == 0)
	{
		if(stk_als_data->ps_distance_last != 0 && stk_als_data->ir_code == 0)
		{
			ret = stk3x3x_get_ir_reading(client, STK_IRS_IT_REDUCE);
			if(ret > 0)
				stk_als_data->ir_code = ret;
		}		
		return ret;
	}	
	return 0;
}
#endif

static int light_sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
    //bool auto_gain = false;
	int flag_data = 0;
    uint8_t count = 0;
    uint16_t als_data[5] = {0};//als/r/g/b/c
	unsigned char buffer[10] = {0};
	uint32_t index = 0;
    //int als = 0.0;
    uint32_t report_lux1;
    //uint32_t c_raw = 0;
    //uint32_t g_raw = 0;

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


	
#ifdef STK_IRS
	result = stk_als_ir_skip_als(client, sensor);
	if(result == 1)
		return 0;
#endif		
	
	buffer[0] = STK_DATA1_ALS_REG;
    result = sensor_rx_data(client, buffer, 10);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
    for(count = 0; count < (sizeof(als_data) / sizeof(als_data[0])); count++) {
        *(als_data + count) = (*(buffer + (2 * count)) << 8 | (* (buffer + (2 * count + 1))));
    }
    
//	index = light_report_abs_value(sensor->input_dev, value);	
	
	/*
	if(sensor->pdata->irq_enable)
	{
		if(sensor->ops->int_status_reg)
		{	
			value = sensor_read_reg(client, sensor->ops->int_status_reg);
		}
		
		if(value & STA_PS_INT)
		{
			value &= ~STA_PS_INT;
			result = sensor_write_reg(client, sensor->ops->int_status_reg,value);	//clear int
			if(result)
			{
				printk("%s:line=%d,error\n",__func__,__LINE__);
				return result;
			}
		}
	}
	*/
    report_lux1 = stk3x3x_als_cal(client, als_data[0]);
#ifdef STK_IRS
	stk_als_ir_run(client);
#endif	
	printk("%s: lux als raw[0]-[4] gain= %d\t%d\t%d\t%d\t%d\t%d\n",
	    __func__, report_lux1, als_data[0], als_data[1], als_data[2], als_data[3], als_data[4]);
	light_report_abs_value(sensor->input_dev, report_lux1);	
	return result;
}

struct sensor_operate light_stk3x3x_ops = {
	.name				= "ls_stk3x3x",
	.type				= SENSOR_TYPE_LIGHT,	//sensor type and it should be correct
	.id_i2c				= LIGHT_ID_STK3X3X,		//i2c id number
	.read_reg			= STK_DATA1_ALS_REG,			//read data
	.read_len			= 2,				//data length
	.id_reg				= SENSOR_UNKNOW_DATA,		//read device id from this register
	.id_data 			= SENSOR_UNKNOW_DATA,		//device id
	.precision			= 16,				//16 bits
	.ctrl_reg 			= STK_STATE_REG,			//enable or disable 
	.int_status_reg 	= SENSOR_UNKNOW_DATA,			//intterupt status register
	.range				= {100,65535},		//range
	.brightness         ={10,255},     //brightness	
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,		
	.active				= light_sensor_active,	
	.init				= light_sensor_init,
	.report				= light_sensor_report_value,
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

