/*
 *  ps_stk3x3x.c - Linux kernel modules for sensortek stk301x, stk321x and stk331x 
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
#include <linux/fs.h>   
#include <asm/uaccess.h> 
#include <linux/sensor-dev.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif

#define STK_POLL_PS
#define STK_TUNE0
#define STK_DEBUG_PRINTF

#include "linux/stk3x3x.h"

#define STK3A5X_TRACKING_QUANTI         3
#define STK3A5X_QUANTI_RANGE            30
#define STK3A5X_PS_DIFF                 100
#define STK3A5X_MAX_MIN_DIFF            200

enum PROX_STATE{
    PS_UNKNOWN = -1,
    PS_NEAR = 0,
    PS_FAR = 1,
};

bool stk_ps_swr = false;
extern bool stk_als_swr;

struct stk3x3x_data {
	int		int_pin;
	uint16_t stk_ps_thd_h;
	uint16_t stk_ps_thd_l;
#ifdef CALI_PS_EVERY_TIME	
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;
#endif	
	int32_t ps_distance_last;
    int32_t ps_report;
    bool ps_need_report;
	bool ps_enabled;
	// bool re_enable_ps;
	bool first_boot;
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;
    uint16_t ps_raw;
    bool ps_thd_update;
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;
	ktime_t ps_tune0_delay;
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;
    int ps_debug_count;
	int stk_max_min_diff;
	int stk_lt_n_ct;
	int stk_ht_n_ct;
    int stk_ps_pocket;
    int stk_h_hand;
    int stk_smg_h;
    int stk_smg_l;
    uint16_t pocket_threshold;
    uint32_t    ct_tracking_avg;
    uint16_t    ct_tracking_cnt;
    uint16_t    ct_tracking_min;
    uint16_t    ct_tracking_max;
#endif	
	atomic_t	recv_reg;
	uint8_t pid;
	uint8_t	p_wv_r_bd_with_co;
};

struct stk3x3x_data *stk3x3x_ps_data;

/*****************************************************************************/
char stk3x3x_reg_map[] =
{
    0x00,
    0x01,
    0x02,
    0x03,
    0x04,
    0x05,
    0x06,
    0x07,
    0x08,
    0x09,
    0x11,
    0x12,
    0x4E,
    0xDB,
    0xA1,
    0xA8,
    0xA9,
    0xAA,
};

static struct stk3x3x_platform_data stk3x3x_platform_data={ 
//  .state_reg = 0x0,    /* disable all */ 
//  .psctrl_reg = 0xB2,    /* ps_persistance=1, ps_gain=64X, PS_IT=0.391ms */ 
//  .alsctrl_reg = 0x32, 	/* als_persistance=1, als_gain=64X, ALS_IT=100ms */
//  .ledctrl_reg = 0xC0,   /* 100mA IRDR, 64/64 LED duty */ 
//  .wait_reg = 0x1F,    /* 50 ms */ 
//  .pd_choice_reg =0x0F,
//  .Again_reg = 0x14,  
  .ps_thd_h = 2000,
  .ps_thd_l = 1800,
  .ps_pocket = 4800,

  //.int_pin = sprd_3rdparty_gpio_pls_irq,  
  .transmittance = 500,
  .ps_max_min_diff = STK3A5X_MAX_MIN_DIFF,
  .ps_ht_n_ct = STK_HT_N_CT,
  .ps_lt_n_ct = STK_LT_N_CT,
  .ps_hand_h = STK_PS_THD_HAND,
  .ps_smg_h = STK_PS_SMG_H,
  .ps_smg_l = STK_PS_SMG_L,
};


static int32_t stk3x3x_check_pid(struct i2c_client *client);
static int stk3x3x_ps_tune_zero_init(struct i2c_client *client);


/*****************************************************************************/

void stk3x3x_dump_reg(struct i2c_client *client)
{
    int i = 0;
    unsigned char reg_val;
    int n = sizeof(stk3x3x_reg_map) / sizeof(stk3x3x_reg_map[0]);

    printk("%s: ", __func__);
    for (i = 0; i < n; i++)
    {
        reg_val = sensor_read_reg(client, stk3x3x_reg_map[i]);
        printk("reg[0x%X]:0x%X ", stk3x3x_reg_map[i], reg_val);
        if (i == n-1)
            printk("\n");
    }
}

static void stk3x3x_get_ps_thd(struct i2c_client *client)
{
    int ret;
    uint16_t ps_thdh, ps_thdl;
    uint8_t tx_buf[4] ={0};

    tx_buf[0] = STK_THDH1_PS_REG;
    ret = sensor_rx_data(client, tx_buf, 4);

    ps_thdh = tx_buf[0] << 8 | tx_buf[1];
    ps_thdl = tx_buf[2] << 8 | tx_buf[3];
    printk("%s:ps_thdh=%d, ps_thdl=%d!!\n",__func__, ps_thdh, ps_thdl);
}

static int32_t stk3x3x_set_ps_thd(struct i2c_client *client, uint16_t thd_h, uint16_t thd_l)
{
    unsigned char val[5];
    int ret;
    val[0] = STK_THDH1_PS_REG;
    val[1] = (thd_h & 0xFF00) >> 8;
    val[2] = thd_h & 0x00FF;
    val[3] = (thd_l & 0xFF00) >> 8;
    val[4] = thd_l & 0x00FF;

    ret = sensor_tx_data(client, val, 5);

    if(ret < 0){
        printk("%s: fail, ret=%d\n", __func__, ret);	
    }

    return ret;
}

static void stk3x3x_ps_ct_tracking_rst(void)
{
    printk(KERN_INFO "%s in\n", __func__);
    if(stk3x3x_ps_data->ct_tracking_cnt != 0) {
        stk3x3x_ps_data->ct_tracking_avg = 0;
        stk3x3x_ps_data->ct_tracking_max = 0;
        stk3x3x_ps_data->ct_tracking_min = 0xFFFF;
        stk3x3x_ps_data->ct_tracking_cnt = 0;
    }
}

static int stk3x3x_ps_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
    char reg_buf = 0;
    
	// u16 ps_code;
#ifdef STK_DEBUG_PRINTF	
	printk("%s init proc = %d\n", __func__, (stk3x3x_ps_data->tune_zero_init_proc ? 1 : 0));
#endif	
	reg_buf = sensor_read_reg(client, STK_STATE_REG);
    reg_buf = (reg_buf & (~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK)));
	
	if(enable)
	{
		reg_buf |= STK_STATE_EN_PS_MASK;
		if(!(reg_buf & STK_STATE_EN_ALS_MASK))
			reg_buf |= STK_STATE_EN_WAIT_MASK;
	}
	printk("%s:reg_ctrl=0x%x,enable=%d\n",__func__, reg_buf, enable);
	result = sensor_write_reg(client, STK_STATE_REG, reg_buf);
	if(result)
		printk("%s:fail to active sensor\n",__func__);


	if(enable)
	{
		usleep_range(4000, 5000);
		//sensor->ops->report(sensor->client);//donot report when eable by david
		
		// stk3x3x_get_ps_reading(client, &ps_code);
		// stk3x3x_set_stk_ps_thd_h(client, ps_code + STK_HT_N_CT);
		// stk3x3x_set_stk_ps_thd_l(client, ps_code + STK_LT_N_CT);
		stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_platform_data.ps_thd_h;
        stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_platform_data.ps_thd_l;
		result = stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
        if (result < 0){
            printk("%s:fail set thd\n",__func__);
        }
        sensor->ops->report(sensor->client);//first report when eable by david
#ifdef STK_DEBUG_PRINTF				
        printk("%s: enable set thdh:%d, thdl:%d\n", __func__, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
#endif	
	    stk3x3x_ps_data->ps_distance_last = PS_UNKNOWN;
	} else {
        stk3x3x_ps_data->ps_distance_last = PS_FAR;
    }
	stk3x3x_ps_data->ps_enabled = enable?true:false;

    //stk3x3x_ps_data->tune_zero_init_proc = false; //david??

	stk3x3x_ps_data->psa = 0x0;
	stk3x3x_ps_data->psi = 0xFFFF;	
	stk3x3x_ps_data->psi_set = 0;
    stk3x3x_ps_data->pocket_threshold = 3000;
    stk3x3x_ps_ct_tracking_rst();

	stk3x3x_ps_data->stk_max_min_diff = stk3x3x_platform_data.ps_max_min_diff;
	stk3x3x_ps_data->stk_lt_n_ct = stk3x3x_platform_data.ps_lt_n_ct;
	stk3x3x_ps_data->stk_ht_n_ct = stk3x3x_platform_data.ps_ht_n_ct;

#ifdef STK_DEBUG_PRINTF
	printk(KERN_INFO "%s: ht:%d lt:%d max diff:%d\n",
	__func__,
	stk3x3x_ps_data->stk_ps_thd_h,
	stk3x3x_ps_data->stk_ps_thd_l,
	stk3x3x_ps_data->stk_max_min_diff);
    stk3x3x_dump_reg(client);
#endif	
	return result;

}

static void stk3x3x_ps_ct_tracking(uint16_t ps_raw_data)
{
    printk("%s in\n", __func__);
    if (ps_raw_data > stk3x3x_ps_data->ct_tracking_max)
        stk3x3x_ps_data->ct_tracking_max = ps_raw_data;//max

    if (ps_raw_data < stk3x3x_ps_data->ct_tracking_min)
        stk3x3x_ps_data->ct_tracking_min = ps_raw_data;//min
}

static void stk3x3x_ps_tune_fae(struct i2c_client *client )
{
    int ret = 0;
    uint16_t ct_value;

    if ((stk3x3x_ps_data->ps_debug_count % 12) == 9)
        printk(KERN_INFO "stk fae:psi_set=%d, ps=%d, dis=%d\n",
            stk3x3x_ps_data->psi_set, stk3x3x_ps_data->ps_raw, stk3x3x_ps_data->ps_distance_last);

    if (stk3x3x_ps_data->psi_set != 0) {
        if (stk3x3x_ps_data->ps_distance_last == PS_NEAR) {
            if ((stk3x3x_ps_data->ps_raw - stk3x3x_ps_data->psi) > stk3x3x_ps_data->stk_h_hand) {
                if (stk3x3x_ps_data->stk_ps_thd_h != (stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_smg_h)) {
                    stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_smg_h;
                    stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_smg_l;
                    //stk3x3x_ps_data->psi_set = stk3x3x_ps_data->psi;

                    ret = stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                    if (ret < 0)
                        printk(KERN_INFO "%s stk fae set1 fail\n", __func__);
                    printk(KERN_INFO "stk fae:near set ht=%d, lt=%d\n",
                        stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                    stk3x3x_ps_data->ps_thd_update = true;
                }
            }
            stk3x3x_ps_ct_tracking_rst();
        } else { //far away
            if (stk3x3x_ps_data->ps_thd_update) { //reset thd
                stk3x3x_ps_data->ps_thd_update = false;
                if ((stk3x3x_ps_data->ps_raw + stk3x3x_ps_data->stk_ht_n_ct) < stk3x3x_ps_data->stk_ps_thd_h) {
                    stk3x3x_ps_data->psi = stk3x3x_ps_data->ps_raw;
                    stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_ht_n_ct;
                    stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_lt_n_ct;
                    ret = stk3x3x_set_ps_thd(client,stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                    if (ret < 0)
                        printk(KERN_INFO "%s stk fae set2 fail\n", __func__);
                    printk(KERN_INFO "stk fae:far update1 ht=%d, lt=%d\n",
                        stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                }else {
                    printk(KERN_INFO "stk far update = 1 no set \n"); 
                }
            } else { //Tracking
                printk("%s ct_traking in\n", __func__);
                stk3x3x_ps_data->ct_tracking_avg += stk3x3x_ps_data->ps_raw;
                stk3x3x_ps_ct_tracking(stk3x3x_ps_data->ps_raw);
                stk3x3x_ps_data->ct_tracking_cnt ++;
                printk("%s ct_traking raw= %d min = %d, max = %d, avg = %d cnt = %d\n",
                    __func__,
                    stk3x3x_ps_data->ps_raw,
                    stk3x3x_ps_data->ct_tracking_min,
                    stk3x3x_ps_data->ct_tracking_max,
                    stk3x3x_ps_data->ct_tracking_avg,
                    stk3x3x_ps_data->ct_tracking_cnt);
                if(stk3x3x_ps_data->ct_tracking_cnt == STK3A5X_TRACKING_QUANTI) {
                    stk3x3x_ps_data->ct_tracking_avg /= stk3x3x_ps_data->ct_tracking_cnt;
                    ct_value = stk3x3x_ps_data->stk_ps_thd_h - stk3x3x_ps_data->stk_ht_n_ct;
                    printk(KERN_INFO "%s ct traking ct_value = %d, word_data = %d\n", __func__, ct_value, stk3x3x_ps_data->ps_raw);
                    if ((stk3x3x_ps_data->ct_tracking_avg < ct_value) &&
                        ((ct_value - stk3x3x_ps_data->ct_tracking_avg) >= 5) &&
                        ((stk3x3x_ps_data->ct_tracking_max - stk3x3x_ps_data->ct_tracking_min) <= STK3A5X_QUANTI_RANGE)) {
                        stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->ct_tracking_avg + stk3x3x_ps_data->stk_ht_n_ct;
                        stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->ct_tracking_avg + stk3x3x_ps_data->stk_lt_n_ct;
                        stk3x3x_ps_data->psi = stk3x3x_ps_data->ct_tracking_avg;
                        ret = stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                        if (ret < 0)
                            printk(KERN_INFO "%s fae set3 fail\n", __func__);
                        printk(KERN_INFO "stk fae:far update2 ht=%d, lt=%d, ps variation = %d, avg=%d\n",
                            stk3x3x_ps_data->stk_ps_thd_h,
                            stk3x3x_ps_data->stk_ps_thd_l,
                            (stk3x3x_ps_data->ct_tracking_max - stk3x3x_ps_data->ct_tracking_min),
                            stk3x3x_ps_data->ct_tracking_avg);
                    }
                    stk3x3x_ps_ct_tracking_rst();
                }
            }
        }
    } else { //if donot update THHD, update psa/psi
        if (stk3x3x_ps_data->ps_raw != 0) {
            if (stk3x3x_ps_data->ps_raw > stk3x3x_ps_data->psa) {
                stk3x3x_ps_data->psa = stk3x3x_ps_data->ps_raw;
                printk(KERN_INFO "stk fae:update psa, psa=%d,psi=%d\n", stk3x3x_ps_data->psa, stk3x3x_ps_data->psi);
            }
            if (stk3x3x_ps_data->ps_raw < stk3x3x_ps_data->psi) {
                stk3x3x_ps_data->psi = stk3x3x_ps_data->ps_raw; 
                printk(KERN_INFO "stk fae:update psi, psa=%d,psi=%d\n", stk3x3x_ps_data->psa, stk3x3x_ps_data->psi);
            }
        }

        if (stk3x3x_ps_data->psa > stk3x3x_ps_data->psi) {
            if (stk3x3x_ps_data->psa - stk3x3x_ps_data->psi > STK3A5X_PS_DIFF ) {
                if ((stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_ht_n_ct) > stk3x3x_ps_data->pocket_threshold) {
                    printk(KERN_INFO "stk fae:in pocket ps=%d, thd=%d\n",
                        stk3x3x_ps_data->ps_raw, stk3x3x_ps_data->pocket_threshold);
                } else {
                    stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_ht_n_ct;
                    stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_lt_n_ct;
                    stk3x3x_ps_data->psi_set = stk3x3x_ps_data->psi;
                    ret = stk3x3x_set_ps_thd(client,stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                    if (ret < 0)
                        printk(KERN_INFO "%s fae set4 fail\n", __func__);
                    printk(KERN_INFO "stk fae:set HT=%d, LT=%d\n",
                        stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                }
            }
        }
    }
}

static int32_t stk3x3x_check_pid(struct i2c_client *client)
{
    char  reg_val;

    reg_val = sensor_read_reg(client, STK_PDT_ID_REG);
    if (reg_val != STK335XX_PID)
	{
		printk("%s PID error\n", __func__);
		return -1;
	}
    printk(KERN_INFO "%s: PID=0x%x\n", __func__, reg_val);

	return 0;
}

#ifdef STK_TUNE0	
static int stk3x3x_ps_val(struct i2c_client *client)
{
	int32_t word_data1, word_data2, word_data3, word_data4;	
	unsigned char value[8];
	int ret;
	
	value[0] = STK_PSOFF_REG;
	ret = sensor_rx_data(client, value, 8);	
	if(ret)
	{
		printk("%s:line=%d,error=%d\n",__func__,__LINE__, ret);
		return ret;
	}

    word_data1 = ((value[0]<<8) | value[1]);
    word_data2 = ((value[2]<<8) | value[3]);
    word_data3 = ((value[4]<<8) | value[5]);
    word_data4 = ((value[6]<<8) | value[7]);

	if((word_data1 > STK_THD_SL) || (word_data2 > STK_THD_SL) || (word_data3 > STK_THD_SL) || (word_data4 > STK_THD_SL))
	{
		printk(KERN_INFO "%s: word_data=%d %d %d %d, lii=%d\n",
            __func__,
            word_data1,
            word_data2,
            word_data3,
            word_data4,
            STK_THD_SL); 
		return 0xFFFF;	
	}
	return 0;
}	

static int stk3x3x_ps_tune_zero_final(struct i2c_client *client)
{
	int ret;
	int value;
	
    printk(KERN_INFO "%s: in\n", __func__);

	value = 0;
#ifndef STK_POLL_PS	
	value |= 0x01;		
#endif
#ifndef STK_POLL_ALS
	value |= STK_INT_ALS;
#endif
	ret = sensor_write_reg(client, STK_INT_REG, value);
	if(ret < 0)
    {
        printk(KERN_INFO "%s: write_reg fail\n", __func__);   
        return ret;	
    }

	value = sensor_read_reg(client, STK_STATE_REG);

	if(!(value & STK_STATE_EN_ALS_MASK))
		value |= STK_STATE_EN_WAIT_MASK;
	if(stk3x3x_ps_data->ps_enabled)
		value |= STK_STATE_EN_PS_MASK;
	
	ret = sensor_write_reg(client, STK_STATE_REG, value);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	
	if(stk3x3x_ps_data->data_count == -1)
	{
		printk(KERN_INFO "%s: exceed limit\n", __func__);
        stk3x3x_ps_data->tune_zero_init_proc = false;
		return 0;
	}
	
	stk3x3x_ps_data->psa = stk3x3x_ps_data->ps_stat_data[0];
	stk3x3x_ps_data->psi = stk3x3x_ps_data->ps_stat_data[2];	
	stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->ps_stat_data[1] + stk3x3x_ps_data->stk_ht_n_ct;
	stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->ps_stat_data[1] + stk3x3x_ps_data->stk_lt_n_ct;
    stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
	printk(KERN_INFO "%s: set HT=%d,LT=%d\n", __func__, stk3x3x_ps_data->stk_ps_thd_h,  stk3x3x_ps_data->stk_ps_thd_l);
    
	return 0;
}
	
static int32_t stk3x3x_tune_zero_get_stk3x3x_ps_data(struct i2c_client *client, int ps_adc)
{
	int ret;
	
	ret = stk3x3x_ps_val(client);	
	if(ret == 0xFFFF)
	{
		stk3x3x_ps_data->data_count = -1;
		stk3x3x_ps_tune_zero_final(client);
		return 0;
	}
	printk(KERN_INFO "%s: ps_adc #%d=%d\n", __func__, stk3x3x_ps_data->data_count, ps_adc);
	stk3x3x_ps_data->ps_stat_data[1] +=  ps_adc;			
	if(ps_adc > stk3x3x_ps_data->ps_stat_data[0])
		stk3x3x_ps_data->ps_stat_data[0] = ps_adc;
	if(ps_adc < stk3x3x_ps_data->ps_stat_data[2])
		stk3x3x_ps_data->ps_stat_data[2] = ps_adc;						
	stk3x3x_ps_data->data_count++;	
	
	if(stk3x3x_ps_data->data_count == 5)
	{
		stk3x3x_ps_data->ps_stat_data[1]  /= stk3x3x_ps_data->data_count;			
		stk3x3x_ps_tune_zero_final(client);
        stk3x3x_ps_data->tune_zero_init_proc = false;
	}		
	
	return 0;
}

static int stk3x3x_ps_tune_zero_init(struct i2c_client *client)
{
	stk3x3x_ps_data->psa = 0x0;
	stk3x3x_ps_data->psi = 0xFFFF;	
	stk3x3x_ps_data->psi_set = 0;	
	stk3x3x_ps_data->ps_stat_data[0] = 0;
	stk3x3x_ps_data->ps_stat_data[2] = 0xFFFF;
	stk3x3x_ps_data->ps_stat_data[1] = 0;
	stk3x3x_ps_data->data_count = 0;
	stk3x3x_ps_data->ps_debug_count = 0;
	stk3x3x_ps_data->tune_zero_init_proc = true;

	/*
	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	sensor->ops->ctrl_data &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK); 
	sensor->ops->ctrl_data |= STK_STATE_EN_PS_MASK;	
	if(!(sensor->ops->ctrl_data & STK_STATE_EN_ALS_MASK))
		sensor->ops->ctrl_data |= STK_STATE_EN_WAIT_MASK;			
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
#ifdef STK_DEBUG_PRINTF		
	printk("%s:reg=0x%x,reg_ctrl=0x%x\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
#endif
	result = sensor_write_reg(client, STK_INT_REG, 0);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	*/
	return 0;	
}

static void stk3x3x_ps_get_min_max(struct i2c_client *client, int word_data)
{
    if(word_data > stk3x3x_ps_data->psa)
    {
        stk3x3x_ps_data->psa = word_data;
        printk("%s: update psa: psa=%d,psi=%d\n", __func__, stk3x3x_ps_data->psa, stk3x3x_ps_data->psi);
    }
    if(word_data < stk3x3x_ps_data->psi)
    {
        stk3x3x_ps_data->psi = word_data;	
        printk("%s: update psi: psa=%d,psi=%d\n", __func__, stk3x3x_ps_data->psa, stk3x3x_ps_data->psi);	
    }
}
static int stk3x3x_ps_tune_zero_func_fae(struct i2c_client *client, int raw_data)
{
	int ret, ps_diff;

    stk3x3x_ps_data->ps_debug_count ++;
    if(stk3x3x_ps_data->ps_debug_count > 50000)
	    stk3x3x_ps_data->ps_debug_count = 0;
    
//	if(stk3x3x_ps_data->psi_set || !(stk3x3x_ps_data->ps_enabled))
//		return 0;
	
	ret = stk3x3x_ps_val(client);	
	if(ret == 0)
	{
		if(raw_data == 0)
		{
			//printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		stk3x3x_ps_get_min_max(client, raw_data);
	}	
	ps_diff = stk3x3x_ps_data->psa - stk3x3x_ps_data->psi;
#ifdef STK_DEBUG_PRINTF				
    if(stk3x3x_ps_data->ps_debug_count % 10 == 1)
        stk3x3x_dump_reg(client);

#endif
    if (stk3x3x_ps_data->psi_set != 0) {
        if(stk3x3x_ps_data->ps_distance_last == PS_NEAR) {
            if((raw_data - stk3x3x_ps_data->psi) > stk3x3x_ps_data->stk_h_hand) {
                if(stk3x3x_ps_data->stk_ps_thd_h != (stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_smg_h)){
                    stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_smg_h;
                    stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_smg_l;
                    ret = stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                    if (ret) {
                        printk("%s:line=%d,ps set thd error\n", __func__, __LINE__);
                    }
                    printk("%s:ps NEAR update thd H=%d L=%d\n", __func__, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                    stk3x3x_ps_data->ps_thd_update = true;
                }
            }
        } else { //ps far
            if(stk3x3x_ps_data->ps_thd_update == true) {
                stk3x3x_ps_data->ps_thd_update = false;
                if((raw_data + stk3x3x_ps_data->stk_ht_n_ct) < stk3x3x_ps_data->stk_ps_thd_h){
                    stk3x3x_ps_data->psi = raw_data;
                    stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_ht_n_ct;
                    stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_lt_n_ct;
                    ret = stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                    if(ret){
                        printk("%s:line=%d,ps set thd1 error\n", __func__, __LINE__);
                    }
                    printk("%s:,ps set thd1 H:%d L:%d psi:%d\n",
                        __func__, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l, stk3x3x_ps_data->psi);
                } else {
                    printk("%s:stk3a5x far update = 1 no set\n", __func__);
                }
            } else { //tracking
                if((raw_data > 0) && (raw_data < (stk3x3x_ps_data->stk_ps_thd_h - stk3x3x_ps_data->stk_ps_thd_h - 5))){
                    stk3x3x_ps_data->psi = raw_data;
                    stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_ht_n_ct;
                    stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_lt_n_ct;
                    ret = stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
                    if(ret){
                        printk("%s:line=%d,ps set thd2 error\n", __func__, __LINE__);
                    }
                    printk("%s:,ps set thd2 H:%d L:%d psi:%d\n",
                        __func__, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l, stk3x3x_ps_data->psi);
                }
            }
        }
    } else { //stk3x3x_ps_data->psi_set !== 0
        if (ps_diff > stk3x3x_ps_data->stk_max_min_diff) {
            if (ps_diff > stk3x3x_ps_data->stk_ps_pocket) {
                printk("%s: is pocket mode!raw = %d diff = %d PK = %d\n",
                    __func__,raw_data, ps_diff, stk3x3x_ps_data->stk_ps_pocket);
            }
            stk3x3x_ps_data->psi_set = stk3x3x_ps_data->psi;
            stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_ht_n_ct;
            stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_ps_data->psi + stk3x3x_ps_data->stk_lt_n_ct;
            ret = stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
            if(ret){
                printk("%s:line=%d,ps set thd2 error\n", __func__, __LINE__);
            }
            printk("%s:,ps set thd2 H:%d L:%d\n", __func__, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
        }
    }
    printk("%s:raw:%d H:%d L:%d psi:%d\n",
        __func__, raw_data, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l, stk3x3x_ps_data->psi);
	return 0;
}	
#endif

/*
static int stk3x3x_ps_distance_last(struct i2c_client *client, int ps)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	char buffer[2] = {0};	
	int reg_flag = 0;

	buffer[0] = STK_FLAG_REG;
	result = sensor_rx_data(client, buffer, 1);	
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	reg_flag = buffer[0];
	reg_flag = (reg_flag & 0x1);

	stk3x3x_ps_data->ps_distance_last = reg_flag ? PS_FAR:PS_NEAR;


	if(ps > stk3x3x_ps_data->stk_ps_thd_h){
		stk3x3x_ps_data->ps_distance_last = PS_NEAR;
	}else if(ps < stk3x3x_ps_data->stk_ps_thd_l){
		stk3x3x_ps_data->ps_distance_last = PS_FAR;
	}

	printk("%s:ps=%d,NF=%d\n",__func__, ps, stk3x3x_ps_data->ps_report);
    if (stk3x3x_ps_data->ps_need_report) {
    	input_report_abs(sensor->input_dev, ABS_DISTANCE, stk3x3x_ps_data->ps_report);
    	input_sync(sensor->input_dev);
    }
	return 0;
}
*/

static int stk3x3x_ps_init(struct i2c_client *client)
{
	int res = 0, i, reg_num;
	 
	printk("%s init ...\n", __func__);

	stk3x3x_ps_data = kzalloc(sizeof(struct stk3x3x_data),GFP_KERNEL);
	if(!stk3x3x_ps_data)
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
        if(!stk_als_swr) {
            res = sensor_write_reg(client, STK_SW_RESET_REG, 0x0);
            if(res < 0)
            {   
                printk(KERN_ERR "%s: stk3x3x SWR fail\n", __func__);
                goto EXIT_ERR;
            }
            
            usleep_range(13000, 15000); 
            stk_ps_swr = true;
            stk_ps_swr = true;
        }
        reg_num = sizeof(stk3x3x_ps_config_table)/sizeof(stk3x3x_ps_config_table[0]);
        for(i=0;i<reg_num;i++)
        {
            res = sensor_write_reg(client, stk3x3x_ps_config_table[i].address,stk3x3x_ps_config_table[i].value);
            //printk("%s init write_reg 0x%x 0x%x %d\n", __func__, stk3x3x_config_table[i].address, stk3x3x_config_table[i].value, res);
            if(res < 0)
                {
                    printk("%s sensor_write_reg err \n", __func__); 
                    goto EXIT_ERR;
                }
        }

    stk3x3x_dump_reg(client);
		
#ifdef STK_TUNE0
	stk3x3x_ps_tune_zero_init(client);
#endif	

	atomic_set(&stk3x3x_ps_data->recv_reg, 0);
    stk3x3x_ps_data->ps_thd_update = false;
	stk3x3x_ps_data->ps_enabled = false;
	stk3x3x_ps_data->ps_distance_last = PS_FAR;
	stk3x3x_ps_data->stk_max_min_diff = stk3x3x_platform_data.ps_max_min_diff;
	stk3x3x_ps_data->stk_lt_n_ct = stk3x3x_platform_data.ps_lt_n_ct;
	stk3x3x_ps_data->stk_ht_n_ct = stk3x3x_platform_data.ps_ht_n_ct;
	stk3x3x_ps_data->stk_ps_thd_h = stk3x3x_platform_data.ps_thd_h;
	stk3x3x_ps_data->stk_ps_thd_l = stk3x3x_platform_data.ps_thd_l;
    stk3x3x_ps_data->stk_ps_pocket = stk3x3x_platform_data.ps_pocket;
    stk3x3x_ps_data->stk_h_hand = stk3x3x_platform_data.ps_hand_h;
    stk3x3x_ps_data->stk_smg_h = stk3x3x_platform_data.ps_smg_h;
    stk3x3x_ps_data->stk_smg_l = stk3x3x_platform_data.ps_smg_l;
    stk3x3x_set_ps_thd(client, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
    printk("%s init set stk_ps_thd_h=%d stk_ps_thd_l=%d\n", __func__, stk3x3x_ps_data->stk_ps_thd_h, stk3x3x_ps_data->stk_ps_thd_l);
	printk("%s init successful \n", __func__);
	return 0;
	
EXIT_ERR:
	printk(KERN_ERR "stk init fail dev: %d\n", res);
	return res;
}

static int stk3x3x_clr_int(struct i2c_client *client)
{
    int ret = 0;
    uint8_t flag;

    printk("%s in\n", __func__);
    flag = sensor_read_reg(client, STK_FLAG_REG);
    if (flag < 0)
        goto err_out;

    flag = flag & ((~STK_FLG_PSINT_MASK) & (~STK_FLG_ALSINT_MASK));
    ret = sensor_write_reg(client, STK_FLAG_REG, flag);
    if (ret < 0)
        goto err_out;

    return ret;
err_out:
    printk("%s fail\n", __func__);
    return ret;
}

static void stk3x3x_get_ps_status(struct i2c_client *client)
{
    int ret;
    int ps_flag;
    uint8_t tx_buf[3] = {0};

    tx_buf[0] = STK_FLAG_REG;
    ret = sensor_rx_data(client, tx_buf, 3);
    if(ret)
    {
        printk("%s:line=%d, get data error!!\n",__func__,__LINE__);
    }
    
    ps_flag = tx_buf[0] & 0x01;
	stk3x3x_ps_data->ps_raw = tx_buf[1] << 8 | tx_buf[2];

    if(stk3x3x_ps_data->ps_debug_count % 10 == 1){
        stk3x3x_get_ps_thd(client);
    }

    if (ps_flag == 1) {
        if (stk3x3x_ps_data->ps_distance_last != PS_FAR){
            stk3x3x_ps_data->ps_need_report = true;
        }
        stk3x3x_ps_data->ps_report = PS_FAR;
    } else if (ps_flag == 0) {
        if (stk3x3x_ps_data->ps_distance_last != PS_NEAR){
            stk3x3x_ps_data->ps_need_report = true;
        }
        stk3x3x_ps_data->ps_report = PS_NEAR;
    } else {
        printk( "stk get_ps_status error status: %d\n", tx_buf[0]);	
    }
    printk("%s stk cur psdata=%d, flag=0x%x, dis=%d\n",
        __func__, stk3x3x_ps_data->ps_raw, tx_buf[0], stk3x3x_ps_data->ps_report);

/*
//clr int 
    if(stk3x3x_ps_data->ps_need_report == true) {
        ret = stk3x3x_clr_int(client);
        if (ret < 0)
           printk( "stk clr int error: \n"); 
    }
*/
}


static int stk3x3x_ps_distance_last_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;

	//printk("%s in\n", __func__);
	stk3x3x_ps_data->ps_debug_count++;
    if(stk3x3x_ps_data->ps_debug_count > 50000)
	    stk3x3x_ps_data->ps_debug_count = 0;
	if(sensor->ops->read_len < 2)	//sensor->ops->read_len = 1
	{
		printk("%s:lenth is error,len=%d\n",__func__,sensor->ops->read_len);
		return -1;
	}

    stk3x3x_get_ps_status(client);
    stk3x3x_ps_data->ps_distance_last = stk3x3x_ps_data->ps_report;
    if(stk3x3x_ps_data->ps_debug_count % 20 == 1) {
        stk3x3x_dump_reg(client);
    }
	if(stk3x3x_ps_data->ps_raw < 0) {
		printk("%s: ps_rawdata < 0 == %d\n",__func__, stk3x3x_ps_data->ps_raw);
		return -1;
	}
#ifdef STK_TUNE0	
	if(stk3x3x_ps_data->tune_zero_init_proc)
		result = stk3x3x_tune_zero_get_stk3x3x_ps_data(client, stk3x3x_ps_data->ps_raw);
	else
        stk3x3x_ps_tune_fae(client);
		//stk3x3x_ps_tune_zero_func_fae(client, ps_raw);	
#endif	
	//result = stk3x3x_ps_distance_last(client, stk3x3x_ps_data->ps_raw);
    if (stk3x3x_ps_data->ps_need_report) {
    	input_report_abs(sensor->input_dev, ABS_DISTANCE, stk3x3x_ps_data->ps_report);
    	input_sync(sensor->input_dev);
    }

    return result;
}

struct sensor_operate proximity_stk3x3x_ops = {
	.name				= "ps_stk3x3x",
	.type				= SENSOR_TYPE_PROXIMITY,	//sensor type and it should be correct
	.id_i2c				= PROXIMITY_ID_STK3X3X,		//i2c id number
	.read_reg			= STK_DATA1_PS_REG,			//read data
//	.addr               =0x47,
	.read_len			= 2,				        //data length
	.id_reg				= 0x3E,//SENSOR_UNKNOW_DATA,		//read device id from this register
	.id_data 			= 0x51,//SENSOR_UNKNOW_DATA,		//device id
	.precision			= 16,				         //16 bits
	.ctrl_reg 			= STK_STATE_REG,			//enable or disable 
	.int_status_reg 	= SENSOR_UNKNOW_DATA,			//intterupt status register
	.range				= {0,1},			//range
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,		
	.active				= stk3x3x_ps_active,	
	.init				= stk3x3x_ps_init,
	.report				= stk3x3x_ps_distance_last_value,
	// int 	brightness[2];//backlight min_brightness max_brightness 
	// int int_ctrl_reg;
	// int (*suspend)(struct i2c_client *client);
	// int (*resume)(struct i2c_client *client);
	// struct miscdevice *misc_dev;	
};

static struct sensor_operate *proximity_get_ops(void)
{
	return &proximity_stk3x3x_ops;
}

static int __init stk3x3x_init(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, proximity_get_ops);
	return result;
}

static void __exit stk3x3x_exit(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, proximity_get_ops);
}


module_init(stk3x3x_init);
module_exit(stk3x3x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x3x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
