/*
 *  ls_stk3x8xx.c - Linux kernel modules for sensortek stk301x, stk321x and stk331x 
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

#include <../../fs/btrfs/math.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif
#include "linux/stk3x8xx.h"

#define DRIVER_VERSION  "3.10.0_0429"

/* Driver Settings */
#define STK_POLL_ALS        /* ALS interrupt is valid only when STK_PS_INT_MODE = 1    or 4*/
#define STK_DEBUG_PRINTF
#define STK3X8XX_DEBUG
//#define STK_ALS_FIR

#define PROXIMITY_ID_I2C    2

#define STK3X8XX_AGC_THDH     62000
#define STK3X8XX_AGC_THDL     3000
#define ALS_COEF            1
#define C_COEF              1

/*****************************************************************************/
/* Define Register Map */
#define STK_STATE_REG             0x00
#define STK_PSCTRL_REG             0x01
#define STK_ALSCTRL_REG         0x02
#define STK_LEDCTRL_REG         0x03
#define STK_INT_REG             0x04
#define STK_WAIT_REG             0x05
#define STK_THDH1_PS_REG         0x06
#define STK_THDH2_PS_REG         0x07
#define STK_THDL1_PS_REG         0x08
#define STK_THDL2_PS_REG         0x09
#define STK_THDH1_ALS_REG         0x0A
#define STK_THDH2_ALS_REG         0x0B
#define STK_THDL1_ALS_REG         0x0C
#define STK_THDL2_ALS_REG         0x0D
#define STK_FLAG_REG             0x10
#define STK_DATA1_PS_REG         0x11
#define STK_DATA2_PS_REG         0x12
#define STK_DATA1_ALS_REG         0x13
#define STK_DATA2_ALS_REG         0x14
#define STK_DATA1_OFFSET_REG     0x15
#define STK_DATA2_OFFSET_REG     0x16
#define STK_DATA1_IR_REG         0x17
#define STK_DATA2_IR_REG         0x18
#define STK_PDT_ID_REG             0x3E
#define STK_RSRVD_REG             0x3F
#define STK_GAINCTRL_REG        0x4E
#define STK_AGAIN_REG	        0xDB
#define STK_FIFOCTRL1_REG               0x60

#define STK_THD1_FIFO_FCNT_REG          0x61
#define STK_THD2_FIFO_FCNT_REG          0x62
#define STK_FIFOCTRL2_REG               0x63
#define STK_FIFOFCNT1_REG               0x64
#define STK_FIFOFCNT2_REG               0x65
#define STK_FIFO_OUT_REG                0x66
#define STK_FIFO_FLAG_REG               0x67
#define STK_ALSCTRL2_REG                0x6F
#define STK_SW_RESET_REG                0x80

#define STK_FIFO_OFF                    0x00
#define STK_FIFO_BPM                    0x10
#define STK_FIFO_FFM                    0x20
#define STK_FIFO_STRM                   0x30
#define STK_FIFO_DATA_ALS               0x00
#define STK_FIFO_DATA_C                 0x01
#define STK_FIFO_DATA_ALSC              0x02
#define STK_FIFOCTRL1_VAL               (STK_FIFO_STRM|STK_FIFO_DATA_ALSC)

#define STK_SHORT_IT_EN                 0x80
#define STK_SHORT_IT_192US              0x00
#define STK_SHORT_IT_288US              0x01
#define STK_SHORT_IT_384US              0x02
#define STK_SHORT_IT_576US              0x03
#define STK_SHORT_IT_672US              0x04
#define STK_SHORT_IT_768US              0x05
#define STK_SHORT_IT_864US              0x06
#define STK_SHORT_IT_960US              0x07
#define STK_SHORT_IT_2112US             0x14


#define STK_STATE_EN_IRS_MASK    0x80
#define STK_STATE_EN_AK_MASK    0x40
#define STK_STATE_EN_ASO_MASK    0x20
#define STK_STATE_EN_IRO_MASK    0x10
#define STK_STATE_EN_WAIT_MASK    0x04
#define STK_STATE_EN_ALS_MASK    0x02
#define STK_STATE_EN_PS_MASK    0x01

#define STK_FLG_ALSDR_MASK        0x80
#define STK_FLG_PSDR_MASK        0x40
#define STK_FLG_ALSINT_MASK        0x20
#define STK_FLG_PSINT_MASK        0x10
#define STK_FLG_OUI_MASK            0x04
#define STK_FLG_IR_RDY_MASK        0x02
#define STK_FLG_NF_MASK            0x01

#define STK_INT_ALS                0x08

#define STK_IRC_MAX_ALS_CODE        20000
#define STK_IRC_MIN_ALS_CODE        25
#define STK_IRC_MIN_IR_CODE        50
#define STK_IRC_ALS_DENOMI        2        
#define STK_IRC_ALS_NUMERA        5
#define STK_IRC_ALS_CORREC        850

#define STK_IRS_IT_REDUCE            2
#define STK_ALS_READ_IRS_IT_REDUCE    5
#define STK_ALS_THRESHOLD            30

#define STK_ALS_GAIN1                   0x00
#define STK_ALS_GAIN4                   0x10
#define STK_ALS_GAIN16                  0x20
#define STK_ALS_GAIN64                  0x30

/*****************************************************************************/
#define STK335XX_PID        0x51
#define STK3X8XX_PID        0x82

/*****************************************************************************/
#define STK_FIFO
#define FIFO_DATA_DEBUG
#define FIFO_DATA_PRT_LEN 10

#ifdef STK_FIFO
#define STK_FIFO_READ_TARGET            64// must lest than STK_FIFO_MAX_FRAME
#define STK_FIFO_MAX_FRAME              256//1024
#define STK_FIFO_MAX_LEN                1024
#endif

#define ALS_DF_GAIN    128

#ifdef STK_ALS_FIR
    #define STK_FIR_LEN    8
    #define MAX_FIR_LEN 32
    
struct data_filter {
    u16 raw[MAX_FIR_LEN];
    int sum;
    int number;
    int idx;
};
#endif

#ifdef STK_FIFO
enum {
    ALS,
    C,
    ALS_C,
};

struct stk3x8xx_fifo_frame {
        uint16_t als;
        uint16_t c;
}stk3x8xx_fifo_frame;

struct stk3x8xx_fifo {
        struct stk3x8xx_fifo_frame frame[STK_FIFO_MAX_FRAME];
        char data[STK_FIFO_MAX_LEN];
        uint16_t byte_per_frame;
        uint16_t frame_cnt;
        int fifo_frame_bytes;
        int fifo_data_sel;
        //bool first_fifo_read;
}stk3x8xx_fifo;
#endif

#define STK3X8XX_MN_LV        4 //min lv is 6
#define STK3X8XX_MX_LV        0 //Dgain * 128 ,Again * 4

//static uint32_t last_als = 0;
//static uint32_t last_data_c = 0; //Modify data type, Because "c_raw" may exceed 65535
static bool first_als = true;
uint8_t stk3x8xx_als_gain_level = STK3X8XX_MX_LV;
uint16_t stk3x8xx_als_dgain = 128;
uint16_t stk3x8xx_als_again = 4;

struct stk3x8xx_data {
    uint16_t als_cnt;
    uint16_t als_correct_factor;
    uint8_t alsctrl_reg;
    uint8_t psctrl_reg;
    uint8_t ledctrl_reg;
    uint8_t state_reg;
    int        int_pin;
    uint8_t wait_reg;
    uint8_t int_reg;
#ifdef CONFIG_HAS_EARLYSUSPEND
    //struct early_suspend stk_early_suspend;
#endif    

    struct input_dev *als_input_dev;
    int32_t als_lux_last;
    uint32_t als_transmittance;    
    bool als_enabled;
    bool re_enable_als;
    bool auto_gain;
    ktime_t als_poll_delay;
#ifdef STK_POLL_ALS        
    struct work_struct stk_als_work;
    struct hrtimer als_timer;    
    struct workqueue_struct *stk_als_wq;
#endif    
    bool first_boot;
#ifdef STK_ALS_FIR
    struct data_filter      fir;
    atomic_t                firlength;    
#endif

    uint8_t pid;
    uint8_t    p_wv_r_bd_with_co;
    uint32_t als_code_last;
};

static struct stk3x8xx_data *stk3x8xx_als;

//const int ALS_LEVEL[] = {100, 500, 1000, 1600, 2250, 3200, 6400, 12800, 20000, 26000};

/*****************************************************************************/
static struct stk3x8xx_register_table stk3x8xx_config_table[] =
{
    {0x00,  0x00},  
    {0x02,  0x00},
    {0x03,  0x00},  
    {0x04,  0x00},
    {0x05,  0x00},
    {0x4E,  0x36}, 
#ifdef STK_FIFO
    //{0x60,  0x30},//0xA2},
    {0x60,  STK_FIFOCTRL1_VAL},//0xA2},
    {0x6F,  STK_SHORT_IT_EN|STK_SHORT_IT_2112US},//0x14},
#else
    {0x60,  STK_FIFO_OFF},//0xA2},
    {0x6F,  0x00},//0x14},
#endif
    {0xA0,  0x10},
    {0xA1,  0x03},
    {0xA5,  0x00},
    {0xDB,  0x00},  
    {0xF6,  0x09}, //{0xF6,  0x14}, 
    {0xF1,  0x00},  
};

static int32_t stk3x8xx_set_als_thd_l(struct i2c_client *client, uint16_t thd_l)
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

static int32_t stk3x8xx_set_als_thd_h(struct i2c_client *client, uint16_t thd_h)
{
    unsigned char val[2];
    int ret;
    
    val[0] = STK_THDH1_ALS_REG;
    val[1] = (thd_h & 0xFF00) >> 8;
    val[2] = thd_h & 0x00FF;
    ret = sensor_tx_data(client, val, 3);    

    if(ret)
        printk("%s:fail to active sensor\n",__func__);    
    return ret;    
}

#ifdef STK3X8XX_DEBUG
void stk3x8xx_dump_reg(struct i2c_client *client)
{
    uint8_t i = 0;
//    int ret = 0;
    uint8_t stk3x8xx_debug_reg[30] = {0};
    uint8_t stk3x8xx_reg_map[] =
    {
        0x00, 0x02, 0x04, 0x05, 0x0A, 
        0x0B, 0x0C, 0x0D, 0x10,
        0x13, 0x14,
        0x15, 0x16, 0x4E, 0x60, 0x61, 0x62,
        0x63, 0x67, 0x64, 0x65, 0x6F, 0x80, 
        0xDB,0xA1
    };

    printk("%s: ", __func__);
    for (i = 0; i < sizeof(stk3x8xx_reg_map) / sizeof(stk3x8xx_reg_map[0]); i++)
    {
        stk3x8xx_debug_reg[i] = sensor_read_reg(client, stk3x8xx_reg_map[i]);
        printk("reg[0x%02X]=0x%02X ", stk3x8xx_reg_map[i], stk3x8xx_debug_reg[i]);
        if ((i + 1)%6 == 0)
            printk("\n%s: ", __func__);
    }
    printk("\n");
}
#else
void stk3x8xx_dump_reg(struct i2c_client *client)
{
    return;
}
#endif

#ifdef STK_FIFO
static void stk3x8xx_get_fifo_info(struct i2c_client *client)
{
    uint8_t buffer[2] = {0};
    int ret = 0;

    buffer[0] = STK_FIFOCTRL1_REG;
    ret = sensor_rx_data(client, buffer, 1);
    if (ret) {
        printk("%s:read fifo info ERR,ret = %d\n",__func__, ret);
    } else if ((buffer[0] | 0x00) == 0x00) {
        stk3x8xx_fifo.byte_per_frame = 2;
        stk3x8xx_fifo.fifo_data_sel = ALS;
    } else if ((buffer[0] & 0x01) == 0x01) {
        stk3x8xx_fifo.byte_per_frame = 2;
        stk3x8xx_fifo.fifo_data_sel = C;
    } else if ((buffer[0] & 0x02) == 0x02){
        stk3x8xx_fifo.byte_per_frame = 4;
        stk3x8xx_fifo.fifo_data_sel = ALS_C;
    } else {
        printk("%s:get fifo info ERR =%d get_reg_val=0x%x\n",__func__, stk3x8xx_fifo.byte_per_frame, buffer[0]);
    }

    printk("%s:get fifo_ctrl= 0x%x fifo_bpf =%d fifo_data_sel =%d\n",
        __func__,
        buffer[0],
        stk3x8xx_fifo.byte_per_frame,
        stk3x8xx_fifo.fifo_data_sel);
}


static void stk3x8xx_get_max_min(struct i2c_client *client, uint16_t *data_array, int data_len)
{
    int cnt, j, tmp;
    //uint32_t max, min;

    for(cnt= 1; cnt < data_len; cnt++){
        tmp = data_array[cnt];
        j = cnt - 1;
    
        while (j >= 0 && data_array[j] > tmp)
        {
            data_array[j + 1] = data_array[j];
            j = j - 1;
        }
        data_array[j + 1] = tmp;
    }
    printk("%s get min=%d max = %d \n", __func__, data_array[0], data_array[data_len - 1]);
}

static void stk3x8xx_fifo_data_tran(char *fifo_data, int data_mode, uint16_t tran_data_len)
{
    int i = 0;
    uint16_t data_len = stk3x8xx_fifo.byte_per_frame;

    switch(data_mode)
    {

    case ALS:
        for( i= 0; i < tran_data_len; i++)
            stk3x8xx_fifo.frame[i].als = (fifo_data[0 + i * data_len] << 8)|(fifo_data[1+ i * data_len]);
#ifdef FIFO_DATA_DEBUG
        printk("%s: ", __func__);
        for (i = 0; i < tran_data_len; i++)
        {
            printk("als[%02d]=%d ", i, stk3x8xx_fifo.frame[i].als);
            if ((i + 1)%FIFO_DATA_PRT_LEN == 0)
                printk("\n%s: ", __func__);
        }
        printk("\n");
#endif
        usleep_range(1200,1300);
        break;
    case C:
        for( i= 0; i < tran_data_len; i++)
            stk3x8xx_fifo.frame[i].c = (fifo_data[0 + i * data_len] << 8)|(fifo_data[1 + i * data_len]);
#ifdef FIFO_DATA_DEBUG
        printk("%s: ", __func__);
        for (i = 0; i < tran_data_len; i++)
        {
            printk("als[%02d]=%d ", i, stk3x8xx_fifo.frame[i].c);
            if ((i + 1)%FIFO_DATA_PRT_LEN == 0)
                printk("\n%s: ", __func__);
        }
        printk("\n");
#endif
        usleep_range(1200,1300);//for donot lost log
        break;
    case ALS_C:
        for( i= 0; i < tran_data_len; i++){
            stk3x8xx_fifo.frame[i].als = (fifo_data[0 + i * data_len] << 8)|(fifo_data[1 + i * data_len]);
            stk3x8xx_fifo.frame[i].c  = (fifo_data[2 + i * data_len] << 8)|(fifo_data[3 + i * data_len]);
        }
#ifdef FIFO_DATA_DEBUG
        printk("%s: ", __func__);
        for (i = 0; i < tran_data_len; i++)
        {
            printk("als_c[%02d]=%d %d  ", i, stk3x8xx_fifo.frame[i].als, stk3x8xx_fifo.frame[i].c);
            if ((i + 1)%FIFO_DATA_PRT_LEN == 0)
                printk("\n%s: ", __func__);
        }
        printk("\n");
#endif
        usleep_range(1200,1300);//for donot lost log
        break;
    default:
        break;
    }
}

static int stk3x8xx_get_fifo_data(struct i2c_client *client, uint16_t *data)
{
    int cnt = 0, min_index = 0, max_index = 0, ret = 0;
    char buffer[2] = {0};

    //printk("%s in als_enabled=%d\n", __func__, stk3x8xx_als_data->als_enabled);

    memset((void *)stk3x8xx_fifo.frame, 0, sizeof(struct stk3x8xx_fifo_frame) * STK_FIFO_MAX_FRAME);
    memset((void *)stk3x8xx_fifo.data, 0, sizeof(uint8_t) * STK_FIFO_MAX_LEN);

    buffer[0] = STK_FIFOFCNT1_REG;
    ret = sensor_rx_data(client, buffer, 2);
    if(ret){
        printk("%s:read fifo cnt ERR\n", __func__);
    }
    stk3x8xx_fifo.frame_cnt = ((buffer[0] & 0x03) << 8) | buffer[1];
    if(stk3x8xx_fifo.frame_cnt > STK_FIFO_READ_TARGET){
        stk3x8xx_fifo.frame_cnt = STK_FIFO_READ_TARGET;
    }
    stk3x8xx_fifo.fifo_frame_bytes = stk3x8xx_fifo.frame_cnt * stk3x8xx_fifo.byte_per_frame; // 1 Frame = ALS+C = 4 bytes
    printk("%s: byte_per_frame=%d frame_cnt =%d fifo_frame_bytes = %d buf0=0x%x buf1=0x%x\n",
        __func__,
        stk3x8xx_fifo.byte_per_frame,
        stk3x8xx_fifo.frame_cnt,
        stk3x8xx_fifo.fifo_frame_bytes,
        buffer[0],
        buffer[1]);

    if (stk3x8xx_fifo.frame_cnt != 0) {
        /*read fifo data*/
        stk3x8xx_fifo.data[0] = STK_FIFO_OUT_REG;
        ret = sensor_rx_data(client, stk3x8xx_fifo.data, stk3x8xx_fifo.fifo_frame_bytes);
        if(ret){
            printk("%s:read fifo cnt ERR\n", __func__);
        }
        //combine data

        stk3x8xx_fifo_data_tran(stk3x8xx_fifo.data, stk3x8xx_fifo.fifo_data_sel, stk3x8xx_fifo.frame_cnt);
        /*get min max index*/
        for(cnt = 1; cnt < stk3x8xx_fifo.frame_cnt; cnt++)
            if(stk3x8xx_fifo.frame[cnt].als < stk3x8xx_fifo.frame[min_index].als)
                min_index = cnt;
        for(cnt= 1; cnt < stk3x8xx_fifo.frame_cnt; cnt++)
            if(stk3x8xx_fifo.frame[cnt].als > stk3x8xx_fifo.frame[max_index].als)
                max_index = cnt;

        data[0] = stk3x8xx_fifo.frame[min_index].als;//als data
        data[1] = stk3x8xx_fifo.frame[min_index].c;//gdata 550
        data[2] = stk3x8xx_fifo.frame[max_index].als;//als data
        data[3] = stk3x8xx_fifo.frame[max_index].c;//gdata 550

        //printk("%s: min_als_c=\t%d\t%d\t%d\n", __func__, data[0], data[1], min_index);
        //printk("%s: max_als_c=\t%d\t%d\t%d\n", __func__, data[2], data[3], max_index);
    }else{
        printk("%s:fifo frame_cnt ERR cnt =%d\n", __func__, stk3x8xx_fifo.frame_cnt);
    }
    //auto_gain = check_auto_gain(scp_service,port_handle,als_raw_data);
    /*clear fifo*/
    usleep_range(1000, 1100); 
    buffer[0] = STK_FIFOCTRL1_REG;//0x60;
    buffer[1] = STK_FIFOCTRL1_VAL;
    ret = sensor_tx_data(client, buffer, 2);
    if(ret){
        printk("%s: clear fifo ERR, ret = %d\n", __func__, ret);
    }

    //stk3x8xx_als_data->last_als = data[0];
    //stk3x8xx_als_data->last_data_g = data[1];
    //stk3x8xx_als_data->last_data_c = data[2];
    
    return ret;
}
#endif

static int stk3x8xx_sensor_active(struct i2c_client *client, int enable, int rate)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);    
    int result = 0;
//    uint8_t reg0x2=0,reg0x4e=0,reg0xdb=0;
    
    sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);    
#ifndef STK_POLL_ALS
    if (enable)
    {                
        stk3x8xx_set_als_thd_h(client, 0x0000);
        stk3x8xx_set_als_thd_l(client, 0xFFFF);
    }    
#endif
    //clear als_en & wait_en
    sensor->ops->ctrl_data = (uint8_t)((sensor->ops->ctrl_data) & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK))); 

    if(enable)
    {
        sensor->ops->ctrl_data |= STK_STATE_EN_ALS_MASK;
        stk3x8xx_als->auto_gain = false;

#ifdef STK_FIFO
        stk3x8xx_get_fifo_info(client);
#endif
    }

    result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
    if(result)
        printk("%s:fail to active sensor\n",__func__);

    if(!enable)
    {
        first_als = true;
        // sensor->ops->report(sensor->client);
    }
    stk3x8xx_als->als_enabled = enable?true:false;
    printk("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n", __func__, sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
    return result;
}

static int32_t stk3x8xx_check_pid(struct i2c_client *client)
{
    char  reg_val;

    reg_val = sensor_read_reg(client, STK_PDT_ID_REG);
    if (reg_val < 0)
    {
        printk("%s PID error\n", __func__);
        return -1;
    }
    printk(KERN_INFO "%s: PID=0x%x\n", __func__, reg_val);

    return 0;
}


static int stk3a8xx_init_reg(struct i2c_client *client)
{
    int res = 0;
    int reg_num, i;

    reg_num = sizeof(stk3x8xx_config_table)/sizeof(stk3x8xx_register_table);
    for(i=0;i<reg_num;i++)
    {
        res = sensor_write_reg(client, stk3x8xx_config_table[i].address, stk3x8xx_config_table[i].value);
        if(res < 0)
            {
                printk("%s sensor_write_reg err \n", __func__);    
                return res;
            }
    }
    return 0;
}

static int stk3x8xx_sensor_init(struct i2c_client *client)
{
    int res = 0;
//    uint8_t reg0x2=0,reg0x4e=0,reg0xdb=0;

    printk("%s init ...\n", __func__);
    stk3x8xx_als = kzalloc(sizeof(struct stk3x8xx_data),GFP_KERNEL);
    if(!stk3x8xx_als)
    {
        printk(KERN_ERR "%s: failed to allocate stk3x8xx_data\n", __func__);
        return -ENOMEM;
    }    

    usleep_range(30000,35000);
    res = stk3x8xx_check_pid(client);
    if(res < 0)
    {   
        printk(KERN_ERR "%s: stk3x8xx_check_pid fail\n", __func__);
        goto EXIT_ERR;
    }

    res = sensor_write_reg(client, STK_SW_RESET_REG, 0x1);
    if(res < 0)
    {   
        printk(KERN_ERR "%s: stk3x8xx SWR fail\n", __func__);
        goto EXIT_ERR;
    }
    
    usleep_range(15000, 15000);    
    res = stk3a8xx_init_reg(client);
    if(res < 0)
        goto EXIT_ERR;
#ifndef STK_POLL_ALS    
    value = STK_INT_REG;
    res = sensor_rx_data(client, value, 1);    
    if(res <= 0)
    {
        printk("%s:line=%d,error=%d\n",__func__,__LINE__, res);
        return res;
    }        

    value |= STK_INT_ALS;
    res = sensor_write_reg(client, STK_INT_REG, value);
    if(res <= 0)
        goto EXIT_ERR;    
#endif
    
    stk3x8xx_als->als_code_last = 0;
    stk3x8xx_als->als_cnt = 0;
    stk3x8xx_fifo.byte_per_frame = 2;//default

#ifdef STK_ALS_FIR
        memset(&stk3x8xx_als->fir, 0x00, sizeof(stk3x8xx_als->fir));  
        atomic_set(&stk3x8xx_als->firlength, STK_FIR_LEN);   
#endif

    
    printk("%s init successful \n", __func__);
    return 0;
    
EXIT_ERR:
    printk(KERN_ERR "stk init fail dev: %d\n", res);
    return res;
}



static int stk3x8xx_report_abs_value(struct input_dev *input, int data)
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

#ifdef STK_ALS_FIR
static int stk_als_cal(struct i2c_client *client, int als_data)
{
    int index;   
    int firlen = atomic_read(&stk3x8xx_als->firlength);   

    printk("%s: als_value %d\n",__func__, als_data);
    stk3x8xx_als->als_code_last = als_data;    
    if(stk3x8xx_als->fir.number < firlen)
    {                
        stk3x8xx_als->fir.raw[stk3x8xx_als->fir.number] = als_data;
        stk3x8xx_als->fir.sum += als_data;
        stk3x8xx_als->fir.number++;
        stk3x8xx_als->fir.idx++;
    }
    else
    {
        index = stk3x8xx_als->fir.idx % firlen;
        stk3x8xx_als->fir.sum -= stk3x8xx_als->fir.raw[index];
        stk3x8xx_als->fir.raw[index] = als_data;
        stk3x8xx_als->fir.sum += als_data;
        stk3x8xx_als->fir.idx++;
        als_data = stk3x8xx_als->fir.sum/firlen;
    }    

    return als_data;
}
#endif
//add auto gain david 20210514
static bool stk3x8xx_set_als_gain(struct i2c_client *client, uint16_t level)
{
    int ret = 0;
    uint8_t alsctrl_reg, gainctrl_reg, againctrl_reg, dgain, again;
    uint8_t rx_buf[1] = {0};

    //printk("%s: set level: %d\n", __func__, level);
    rx_buf[0] = sensor_read_reg(client, STK_ALSCTRL_REG);
    alsctrl_reg = rx_buf[0];
    rx_buf[0] = sensor_read_reg(client, STK_GAINCTRL_REG);
    gainctrl_reg = rx_buf[0];
    rx_buf[0] = sensor_read_reg(client, STK_AGAIN_REG);
    againctrl_reg = rx_buf[0];

    if (level == 0) {
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN64; //ignore
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x06; //als&c data gain x128
        dgain = 128;
        again = 4;
    } else if (level == 1) {
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN64; //als gain 64
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x30; //c data gain x64
        dgain = 64;
        again = 4;
    } else if (level == 2) {
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN16;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x20; //c data gain x16
        dgain = 16;
        again = 4;
    } else if (level == 3) {
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN4;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x10; //c data gain x4
        dgain = 4;
        again = 4;
    } else if (level == 4) {//again * 2,dgain *1
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN1;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x00; //c data gain x1
        dgain = 1;
        again = 4;
    } else if (level == 5) {//again * 1,dgain *1
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN1;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x00; //c data gain x1
        againctrl_reg = (againctrl_reg & 0xC3) | 0x14; //again x1
        dgain = 1;
        again = 2;
    } else if (level == 6) {//again * 1,dgain *1
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK_ALS_GAIN1;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x00; //c data gain x1
        againctrl_reg = (againctrl_reg & 0xC3) | 0x28; //again x0.5
        dgain = 1;
        again = 1;
    } else {
        printk("%s level = %d, return!\n", __func__, level);
        return false;
    }

    ret |= sensor_write_reg(client, STK_ALSCTRL_REG, alsctrl_reg);
    ret |= sensor_write_reg(client, STK_GAINCTRL_REG, gainctrl_reg);
    ret |= sensor_write_reg(client, STK_AGAIN_REG, againctrl_reg);
    rx_buf[0] = sensor_read_reg(client, 0x5F);
    ret |= sensor_write_reg(client, 0x5F, (rx_buf[0] | 0x01));
    if(ret < 0)
    {
        printk("%s sensor_write_reg err \n", __func__);
    }

    stk3x8xx_als_dgain = dgain;
    stk3x8xx_als_again = again;
    printk("%s set level = %d %d %d\n", __func__, level, stk3x8xx_als_dgain, stk3x8xx_als_again);

    return true;
}

static bool stk3x8xx_als_auto_gain(struct i2c_client *client, uint16_t *als_data)
{
    bool result = false;

    if (((als_data[0]) > STK3X8XX_AGC_THDH || (als_data[1] > STK3X8XX_AGC_THDH)) &&
        (stk3x8xx_als_gain_level < STK3X8XX_MN_LV)) {
        // Reduce gain
        printk("%s: cur lev:%d %d %d, up\n", __func__, stk3x8xx_als_gain_level, stk3x8xx_als_dgain, stk3x8xx_als_again);
        stk3x8xx_als_gain_level++;
        result = stk3x8xx_set_als_gain(client, stk3x8xx_als_gain_level);
    } else if (((als_data[0] < STK3X8XX_AGC_THDL) && (als_data[1] < STK3X8XX_AGC_THDL)) &&
        (stk3x8xx_als_gain_level > STK3X8XX_MX_LV)) {
        // Raise gain
        printk("%s: cur lev:%d %d %d, dwn\n", __func__, stk3x8xx_als_gain_level, stk3x8xx_als_dgain, stk3x8xx_als_again);
        stk3x8xx_als_gain_level--;
        result = stk3x8xx_set_als_gain(client, stk3x8xx_als_gain_level);
    }
    return result;
}
//add auto gain david 20210514 end

// lux_calc
uint32_t als_ratio = 1.0; //als factory cali ratio,产线校准系数 
uint32_t c_ratio = 1.0; //c factory cali ratio
typedef enum {
    STK_ALS_DATA_ALS= 0,        // default
    STK_ALS_DATA_C,
    STK_ALS_DATA_SIZE,
} stk3a8x_als_data_position;

typedef struct light_param {
    uint8_t group_sel;
    uint8_t group_rule;
    uint32_t group_rule_mat;
    uint32_t group_rule_mat_2; 
    uint32_t param_ac_ratio;
    uint32_t param_a_scale;
    uint32_t param_c_scale;
    uint32_t param_a2_scale;
    uint32_t param_c2_scale;
    uint32_t param_limit;
    uint16_t param_lower_thd;
    uint8_t  param_lower_sel;
} light_param;
#define STK3A8X_SENSOR_PARE_NUM_2 2
#define STK3A8X_INT_SCL 1000

light_param lux_pare_sel[STK3A8X_SENSOR_PARE_NUM_2];

static uint32_t stk3a8x_als_compensation(uint32_t als_data_t, uint32_t c_data_t, uint8_t panel_sel)
{
    //To-do
    /*
        1. It has patent(basic patent) for ours.
        2. burden of proof for other user(filing of an application now).
        3. note: 
            als_data_t = als_raw_data * 128 /cur_gain;
            c_data_t = c_raw_data * 128 /cur_gain;
            panel_sel is diffrent panel if needed;
        
    */
    uint32_t lux_calc = 0;
    uint32_t div_ratio = 0;
    uint32_t f_calc_als_data[STK_ALS_DATA_SIZE] = {0};
    uint32_t calc_tmp[STK_ALS_DATA_SIZE]={0};
    uint32_t calc_tmp1=0.0;
    uint32_t calc_tmp2=0.0;
    bool  entry_flag = false;
    uint32_t calc_tmp_diff_ratio=0.0;
    uint8_t lux_calc_i = 0;
    uint8_t lux_calc_j = 0;
    uint8_t i = 0;
    uint8_t lux_size = STK3A8X_SENSOR_PARE_NUM_2;

    light_param lux_pare_temp[STK3A8X_SENSOR_PARE_NUM_2] = {
    {0, 0,  0,  1,    500,     2,   2, 2, 2, 1, 100, 0},
    {1, 0,  0,  1,    0,     1,   1, 1, 1, 1, 100, 0},
    };
    
    light_param lux_pare_temp_def[STK3A8X_SENSOR_PARE_NUM_2] = {
    {0, 0,  0,  1,    5,     1,   0.01, 0.05,0.05, 1.03, 100, 0},
    {1, 0,  0,  1,    0,     1,   0.01, 0.05,0.05, 1.03, 100, 0},
    };

    //chioce panel for project(2 or more)
    switch(panel_sel)
    {
        case 0:
            for (i = 0; i < STK3A8X_SENSOR_PARE_NUM_2; i++) 
                lux_pare_sel[i] = lux_pare_temp[i];
        break;
        default:
                for (i = 0; i < STK3A8X_SENSOR_PARE_NUM_2; i++) 
                lux_pare_sel[i] = lux_pare_temp_def[i];            
        break;
    }

    //note:
    //als_info.last_raw_data = raw_data * gain_ratio
    //f_calc_als_data[STK_ALS_DATA_ALS] = raw_data * gain_ratio * coef.

    f_calc_als_data[STK_ALS_DATA_ALS] = ((uint32_t)als_data_t * als_ratio);
    f_calc_als_data[STK_ALS_DATA_C] = ((uint32_t)c_data_t * c_ratio);

    calc_tmp1 = lux_pare_sel[0].group_rule_mat;
    calc_tmp2 = lux_pare_sel[0].group_rule_mat_2;

    div_ratio = f_calc_als_data[(uint8_t)calc_tmp1] * STK3A8X_INT_SCL; // *100 for not float type
    if(0 != f_calc_als_data[(uint8_t)calc_tmp2])
        div_ratio /= f_calc_als_data[(uint8_t)calc_tmp2];

    calc_tmp1 =0;
    calc_tmp2 =0;

       //calc lux
    for(lux_calc_i = 0; lux_calc_i<lux_size; lux_calc_i++) {
        if((div_ratio > lux_pare_sel[lux_calc_i].param_ac_ratio) || (lux_calc_i == (lux_size - 1))) {
            if((lux_pare_sel[0].param_lower_thd > als_data_t) && (lux_pare_sel[0].param_lower_thd > c_data_t)){
                    //default
                lux_calc_i = (lux_pare_sel[0].param_lower_sel);
                calc_tmp1 = f_calc_als_data[STK_ALS_DATA_ALS];
                calc_tmp1 *= (lux_pare_sel[lux_calc_i].param_a2_scale);

                calc_tmp2 = f_calc_als_data[STK_ALS_DATA_C];
                calc_tmp2 *= (lux_pare_sel[lux_calc_i].param_c2_scale);    

                lux_calc = calc_tmp1 + calc_tmp2;
                lux_calc_j = lux_calc_i;
                lux_calc_i = (lux_size + 1);

                if((lux_size + 1) == lux_calc_i) {
                    printk("%s: als_sel = %d\n", __func__, (uint16_t)(lux_pare_sel[lux_calc_j].group_sel));
                }
            } else {
                entry_flag = false; //initial flag
                if(0 == lux_pare_sel[lux_calc_i].group_rule){
                    if(div_ratio > lux_pare_sel[lux_calc_i].param_ac_ratio)
                        entry_flag = true;          
                }else{
                    if(div_ratio < lux_pare_sel[lux_calc_i].param_ac_ratio)
                        entry_flag = true;
                }

                if((true == entry_flag) || (lux_calc_i == (lux_size - 1))) {
                    calc_tmp[STK_ALS_DATA_ALS] = f_calc_als_data[STK_ALS_DATA_ALS];
                    calc_tmp[STK_ALS_DATA_ALS] *= (lux_pare_sel[lux_calc_i].param_a_scale); 
                    calc_tmp[STK_ALS_DATA_C] = f_calc_als_data[STK_ALS_DATA_C];
                    calc_tmp[STK_ALS_DATA_C] *= (lux_pare_sel[lux_calc_i].param_c_scale);

                    //normalize data
                    calc_tmp1 = 0;
                    calc_tmp2 = 0;
                    for(i = STK_ALS_DATA_ALS; i <= STK_ALS_DATA_C; i++){
                        if(0 < calc_tmp[i]){
                            calc_tmp1 +=calc_tmp[i]; 
                        }else{
                            calc_tmp2 +=calc_tmp[i];
                        }
                    }
                    calc_tmp_diff_ratio = calc_tmp1;

                    //need include math.h
                    if(0 != calc_tmp2)
                        calc_tmp_diff_ratio = calc_tmp_diff_ratio * STK3A8X_INT_SCL / abs((uint32_t)calc_tmp2); //*1000 for not flaot

                    if(calc_tmp_diff_ratio > (lux_pare_sel[lux_calc_i].param_limit)) {
                        lux_calc =  calc_tmp1 + calc_tmp2;
                    } else {
                        calc_tmp1 = f_calc_als_data[STK_ALS_DATA_ALS];
                        calc_tmp1 *= (lux_pare_sel[lux_calc_i].param_a2_scale);

                        calc_tmp2 = f_calc_als_data[STK_ALS_DATA_C];
                        calc_tmp2 *= (lux_pare_sel[lux_calc_i].param_c2_scale);    

                        lux_calc = calc_tmp1 + calc_tmp2;
                    }

                    lux_calc_j = lux_calc_i;
                    lux_calc_i = (lux_size + 1);

                    if((lux_size + 1) == lux_calc_i) {
                        printk("%s: als_sel = %d\n", __func__, (uint16_t)(lux_pare_sel[lux_calc_j].group_sel));
                    }
                }
            } 
        }
    }
    printk("%s: get lux = %d\n", __func__, (uint32_t)lux_calc);
       //end calc lux
    return lux_calc;
}


//

static int stk3x8xx_report_value(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int result = 0;
    int flag_data = 0;
    uint32_t lux = 0;
    uint8_t count = 0;
    uint16_t gain_ratio;
    uint16_t als_data[2] = {0};//f/c
    uint16_t fifo_data[4] = {0}; // f/c
    unsigned char buffer[4] = {0};
    char index = 0;
    uint32_t als = 0;
    uint32_t c_raw = 0;
    //uint8_t reg0x2=0,reg0x4e=0,reg0xdb=0;

    stk3x8xx_als->als_cnt++;
    if(stk3x8xx_als->als_cnt > 5){
        stk3x8xx_dump_reg(client);
        stk3x8xx_als->als_cnt = 0;
    }

    flag_data = sensor_read_reg(client, STK_FLAG_REG);
    if(flag_data < 0)
    {
        printk("%s read STK_FLAG_REG, ret=%d\n", __func__, flag_data);
        return flag_data;
    }

    if((!(flag_data & STK_FLG_ALSDR_MASK)) || stk3x8xx_als->auto_gain)
    {
        printk("%s skip frame flag=0x%02X auto:%d\n", __func__, flag_data, stk3x8xx_als->auto_gain);
        stk3x8xx_als->auto_gain = false;
        return 0;
    }
    buffer[0] = STK_DATA1_ALS_REG;
    result = sensor_rx_data(client, buffer, 4);
    if (result)
    {
        printk("%s:line=%d,error\n",__func__,__LINE__);
        return result;
    }
    for(count = 0; count < (sizeof(als_data) / sizeof(als_data[0])); count++) {
        *(als_data + count) = (*(buffer + (2 * count)) << 8 | (* (buffer + (2 * count + 1))));
    }

#ifdef STK_FIFO
            result = stk3x8xx_get_fifo_data(client, fifo_data);
            printk("%s, david get fifo data min:als-c:\t%d\t%d max:als-c:\t%d\t%d\n",
                __func__, fifo_data[0], fifo_data[1], fifo_data[2], fifo_data[3]);
#endif

    gain_ratio = 128 * 4 / (stk3x8xx_als_dgain * stk3x8xx_als_again);

    als = als_data[0] * gain_ratio; // als data
    c_raw = als_data[1] * gain_ratio;// c data

    //todo :: calc report_lux
    //report_lux = stk3x8xx_get_als_lux(als, c_raw);
    //lux = als * ALS_COEF + c_raw * C_COEF;
    lux = stk3a8x_als_compensation(als, c_raw, 0);
#ifdef STK_DEBUG_PRINTF
    printk("%s:lux als(raw)_c(raw)_gain= %d %d(%d)\t(%d)%d\t%d\t%d\n",
        __func__, (uint32_t)lux, als, als_data[0], c_raw, als_data[1], stk3x8xx_als_dgain, stk3x8xx_als_again);
#endif
    if (!first_als) {
        stk3x8xx_als->auto_gain = stk3x8xx_als_auto_gain(client, als_data);
    } else {
        printk("%s, first als data\n", __func__);
        first_als = false;
    }
#ifdef STK_ALS_FIR
    stk_als_cal(client, als_data[0]);
#endif
    //printk("%s: als_value %d\n",__func__, als_value);
    //index = stk3x8xx_report_abs_value(sensor->input_dev, als);    
    index = stk3x8xx_report_abs_value(sensor->input_dev, lux);

    return result;
}

struct sensor_operate stk3x8xx_ops = {
    .name                = "ls_stk3x8xx",
    .type                = SENSOR_TYPE_LIGHT,    //sensor type and it should be correct
    .id_i2c              = LIGHT_ID_STK3X8XX,        //i2c id number
    .read_reg            = STK_DATA1_ALS_REG,            //read data
    .read_len            = 2,                //data length
    .id_reg              = 0x3E,//SENSOR_UNKNOW_DATA,        //read device id from this register
    .id_data             = STK3X8XX_PID,//SENSOR_UNKNOW_DATA,        //device id
    .precision           = 16,                //16 bits
    .ctrl_reg            = STK_STATE_REG,            //enable or disable 
    .int_status_reg      = SENSOR_UNKNOW_DATA,            //intterupt status register
    .range               = {2,65535},        //range
    .brightness          ={5,255},     //brightness    
    .trig                = IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,        
    .active              = stk3x8xx_sensor_active,    
    .init                = stk3x8xx_sensor_init,
    .report              = stk3x8xx_report_value,
};

static struct sensor_operate *light_get_ops(void)
{
    return &stk3x8xx_ops;
}

static int __init stk3x8xx_init(void)
{
    struct sensor_operate *ops = light_get_ops();
    int result = 0;
    int type = ops->type;
    result = sensor_register_slave(type, NULL, NULL, light_get_ops);
    return result;
}

static void __exit stk3x8xx_exit(void)
{
    struct sensor_operate *ops = light_get_ops();
    int type = ops->type;
    sensor_unregister_slave(type, NULL, NULL, light_get_ops);
}


module_init(stk3x8xx_init);
module_exit(stk3x8xx_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x8xx Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

