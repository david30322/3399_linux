/*
 *  ls_stk3a6x.c - Linux kernel modules for sensortek stk301x, stk321x and stk331x 
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
#include <linux/stk3a6x.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif

/* Driver Settings */
#define STK_POLL_ALS        /* ALS interrupt is valid only when STK_PS_INT_MODE = 1    or 4*/
#define STK_DEBUG_PRINTF
//#define muse_debug

//#define STK_ALS_FIR
//#define A(x) ((((uint16_t)(((uint8_t *)(&(x)))[0]))<< 8) + ((uint16_t)(((uint8_t *)&(x)))[1])) //debug for  HW J
//#define FIFO_DATA_DEBUG

#define ALS_DEFAULT_GAIN    128
#define STK_MAX_ALS_THD     62000
#define STK_MIN_ALS_THD     3000

/*****************************************************************************/
#define STK3A6X_PID_LIST_NUM    8
char stk3a6x_pid_list[STK3A6X_PID_LIST_NUM] = {0x21, 0x22, 0x23, 0x24, 0x25,
                                                  0x26, 0x2E};
/*****************************************************************************/

#ifdef STK_ALS_FIR
    #define STK_FIR_LEN 5
    #define MAX_FIR_LEN 32
    
struct data_filter {
    uint32_t raw[STK_FIR_LEN];
    int number;
    int idx;
} data_filter;
#endif

static uint32_t last_als = 0;
static uint32_t last_data_c = 0; //Modify data type, Because "c_raw" may exceed 65535
static bool first_als = true;
uint8_t stk3a6x_als_gain_level = 0;
uint16_t stk3a6x_als_gain = 128;

#ifdef STK_FIFO
enum {
    PS_ONLY,
    ALS_C1,
    ALS_C2,
    ALS_C1_C2_PS,
};

struct stk3a6x_fifo_frame {
        uint16_t als;
        uint16_t c1;
        uint16_t c2;
        uint16_t ps;
}stk3a6x_fifo_frame;

struct stk3a6x_fifo {
        struct stk3a6x_fifo_frame frame[STK_FIFO_MAX_FRAME];
        char data[STK_FIFO_MAX_LEN];
        uint16_t byte_per_frame;
        uint16_t frame_cnt;
        int fifo_frame_bytes;
        int fifo_data_sel;
        //bool first_fifo_read;
}stk3a6x_fifo;
#endif

struct stk3a6x_data {
    uint8_t deviceId;
    uint16_t ir_code;
    uint16_t als_correct_factor;
    uint8_t alsctrl_reg;
    uint8_t ledctrl_reg;
    uint8_t state_reg;
    int int_pin;
    uint8_t wait_reg;
    uint8_t int_reg;
#ifdef CONFIG_HAS_EARLYSUSPEND
    //struct early_suspend stk_early_suspend;
#endif    

    struct mutex io_lock;

    struct input_dev *als_input_dev;
    int32_t als_lux_last;
    uint32_t als_transmittance;    
    bool als_enabled;
    bool re_enable_als;
    ktime_t als_poll_delay;
#ifdef STK_POLL_ALS        
    struct work_struct stk_als_work;
    struct hrtimer als_timer;    
    struct workqueue_struct *stk_als_wq;
#endif

    bool first_boot;
//bool first_als;

#ifdef STK_ALS_FIR
    struct data_filter      fir;
    atomic_t                firlength;    
#endif
    atomic_t    recv_reg;

#ifdef STK_QUALCOMM_POWER_CTRL
    struct regulator *vdd;
    struct regulator *vio;
    bool power_enabled;
#endif    
    uint8_t pid;
    uint32_t als_code_last;
    bool first_als;
    uint32_t last_als;
    uint32_t last_data_g;
    uint32_t last_data_c;

    uint32_t debug_cnt;
};

static struct stk3a6x_data *stk3a6x_als_data;

//const int ALS_LEVEL[] = {100, 500, 1000, 1600, 2250, 3200, 6400, 12800, 20000, 26000};

/*****************************************************************************/

static int32_t stk3a6x_set_als_thd_l(struct i2c_client *client, uint16_t thd_l)
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

static int32_t stk3a6x_set_als_thd_h(struct i2c_client *client, uint16_t thd_h)
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

#ifdef STK_FIFO
static void stk3a6x_get_fifo_info(struct i2c_client *client)
{
    uint8_t buffer[2] = {0};
    int ret = 0;

    buffer[0] = STK_FIFOCTRL1_REG;
    ret = sensor_rx_data(client, buffer, 2);
    if (ret) {
        printk("%s:read fifo info ERR\n",__func__);
    } else if ((buffer[0] & 0x30) != 0) {
        if (((buffer[0] << 4) | 0x00) == 0x30) {
            stk3a6x_fifo.byte_per_frame = 2;
            stk3a6x_fifo.fifo_data_sel = PS_ONLY;
        } else if (((buffer[0] << 4) & 0xF0) == 0x10) {
            stk3a6x_fifo.byte_per_frame = 4;
            stk3a6x_fifo.fifo_data_sel = ALS_C1;
        } else if (((buffer[0] << 4) & 0xF0) == 0x20){
            stk3a6x_fifo.byte_per_frame = 4;
            stk3a6x_fifo.fifo_data_sel = ALS_C2;
        } else if (((buffer[0] << 4) & 0xF0) == 0x40) {
            stk3a6x_fifo.byte_per_frame = 8;
            stk3a6x_fifo.fifo_data_sel = ALS_C1_C2_PS;
        } else {
            printk("%s:get fifo info ERR =%d\n",__func__, stk3a6x_fifo.byte_per_frame);
        }
    } else {
        printk("%s:get fifo reg = 0x%x\n",__func__, buffer[0]);
    }

    printk("%s:get fifo_ctrl= 0x%x fifo_bpf =%d fifo_data_sel =%d\n",
        __func__,
        buffer[0],
        stk3a6x_fifo.byte_per_frame,
        stk3a6x_fifo.fifo_data_sel);
}
#endif

static int stk3a6x_light_sensor_active(struct i2c_client *client, int enable, int rate)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);    
    int result = 0, fifo_cnt = 0;
    char buffer[2] = {0};
    
    sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg); //read 0x00   
#ifndef STK_POLL_ALS
    if (enable)
    {                
        stk3a6x_set_als_thd_h(client, 0x0000);
        stk3a6x_set_als_thd_l(client, 0xFFFF);
    }    
#endif    
    sensor->ops->ctrl_data = (uint8_t)((sensor->ops->ctrl_data) & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK))); 

    if(enable) {
        sensor->ops->ctrl_data |= STK_STATE_EN_ALS_MASK;
    } else if (sensor->ops->ctrl_data & STK_STATE_EN_PS_MASK) {
        sensor->ops->ctrl_data |= (~STK_STATE_EN_WAIT_MASK);
    }
    stk3a6x_als_data->debug_cnt = 0;

#ifdef STK_FIFO
    if(enable){
        buffer[0] = STK_FIFOFCNT1_REG;
        result = sensor_rx_data(client, buffer, 2);
        if(result) {
            printk("%s:david fail to read fifo cnt\n",__func__);
            return -1;
        }
        fifo_cnt = (buffer[0] << 8) | buffer[1];
        printk("%s:get enable fifo_cnt= %d(0x%x)\n", __func__, fifo_cnt, fifo_cnt);
        /*enable fifo*/
        result = sensor_write_reg(client, STK_FIFOCTRL1_REG, STK_FIFOCTRL1_VAL);
        if(result){
            printk("%s:fail to en fifo\n", __func__);
        }
    }else if(!enable){
        result = sensor_write_reg(client, STK_FIFOCTRL1_REG, 0x00);
        if(result){
            printk("%s:fail to clear fifo\n", __func__);
        }else{
            printk("%s:clear fifo before disable als\n", __func__);
        }
    }
    stk3a6x_get_fifo_info(client);
#endif

    result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
    if(result)
        printk("%s:fail to active sensor val= %d\n", __func__, sensor->ops->ctrl_data);

    if(!enable)
    {
        stk3a6x_als_data->first_als = true;
        first_als = true;
        sensor->ops->report(sensor->client);
    }
    stk3a6x_als_data->als_enabled = enable?true:false;
    printk("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d(%d)\n",
        __func__,
        sensor->ops->ctrl_reg,
        sensor->ops->ctrl_data,
        enable,
        stk3a6x_als_data->als_enabled);
    return result;
}

static int32_t stk3a6x_check_pid(struct i2c_client *client)
{
    char  reg_val;
    int pid_count;

    reg_val = sensor_read_reg(client, STK_PDT_ID_REG);
    stk3a6x_als_data->deviceId = reg_val;
    if (reg_val < 0)
    {
        printk("stk %s PID error\n", __func__);
        return -1;
    }
    for (pid_count = 0; pid_count < STK3A6X_PID_LIST_NUM; pid_count++)
    {
        if (stk3a6x_als_data->deviceId == stk3a6x_pid_list[pid_count])
        {
            printk(KERN_INFO "%s: PID=0x%x\n", __func__, stk3a6x_als_data->deviceId);
            return 0;
        } else {
            printk(KERN_INFO "%s: PID do not match!!! get pid = 0x%x\n", __func__, stk3a6x_als_data->deviceId);
            return -1;
        }
    }

    return 0;
}

void stk3a6x_dump_reg(struct i2c_client *client)
{
    uint8_t i = 0;
//    int ret = 0;
    uint8_t stk3a6x_debug_reg[20] = {0};
    uint8_t stk3a6x_reg_map[] =
    {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 
        0x4E, 0x81, 0xA0, 0xA1, 0xA2, 0xAA, 
        0xDB, 0x60, 0x63, 0x6F, 0x46, 0x47
    };

    printk("%s;", __func__);
    for (i = 0; i < sizeof(stk3a6x_reg_map) / sizeof(stk3a6x_reg_map[0]); i++)
    {
        stk3a6x_debug_reg[i] = sensor_read_reg(client, stk3a6x_reg_map[i]);
        printk("reg[0x%X]=0x%X  ", stk3a6x_reg_map[i], stk3a6x_debug_reg[i]);
        if (i == sizeof(stk3a6x_reg_map) / sizeof(stk3a6x_reg_map[0]) - 1)
            printk("\n");
    }
}

static int stk3a6x_light_sensor_init(struct i2c_client *client)
{
    int res = 0;
    int reg_num, i;
    
    printk("stk %s init ...\n", __func__);
    stk3a6x_als_data = kzalloc(sizeof(struct stk3a6x_data),GFP_KERNEL);
    if(!stk3a6x_als_data)
    {
        printk(KERN_ERR "%s: failed to allocate stk3a6x_data\n", __func__);
        return -ENOMEM;
    }    

    res = stk3a6x_check_pid(client);
    if(res < 0)
    {   
        printk(KERN_ERR "%s: stk3a6x_check_pid fail\n", __func__);
        goto EXIT_ERR;
    }

    res = sensor_write_reg(client, STK_SW_RESET_REG, 0x0);
    if(res < 0)
    {   
        printk(KERN_ERR "%s: stk3a6x SWR fail\n", __func__);
        goto EXIT_ERR;
    }
    
    usleep_range(13000, 15000);    
    
    reg_num = sizeof(stk3a6x_config_table)/sizeof(stk3a6x_config_table[0]);
    for(i=0;i<reg_num;i++)
    {
        res = sensor_write_reg(client, stk3a6x_config_table[i].address,stk3a6x_config_table[i].value);
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
    
    stk3a6x_als_data->als_code_last = 0;
    stk3a6x_als_data->debug_cnt = 0;

#ifdef STK_ALS_FIR
        memset(&stk3a6x_als_data->fir, 0x00, sizeof(stk3a6x_als_data->fir));  
#endif

    printk("%s init successful \n", __func__);
    return 0;
    
EXIT_ERR:
    printk(KERN_ERR "stk init fail dev: %d\n", res);
    return res;
}



static int stk3a6x_light_report_abs_value(struct input_dev *input, int data)
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
static void stk3a6x_als_bubble_sort(uint32_t* sort_array, uint8_t size_n)
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


static uint32_t stk3a6x_als_fir(struct i2c_client *client, uint32_t als_raw_data)
{
#ifdef STK_ALS_FIR
        int fir_index = 0;   
        uint32_t mid_als = 0;
        uint32_t cpraw[STK_FIR_LEN] = { 0 };
#endif
        printk("%s: als_value %d\n",__func__, als_raw_data);
        stk3a6x_als_data->als_code_last = als_raw_data;

#ifdef STK_ALS_FIR
    memset(cpraw, 0x00, sizeof(cpraw) / sizeof(cpraw));
    if(data_filter.number < STK_FIR_LEN)
    {                
        data_filter.raw[data_filter.number] = als_raw_data;
        data_filter.number++;
        data_filter.idx++;
    }
    else
    {
        fir_index = data_filter.idx % data_filter.number;
        data_filter.raw[fir_index] = als_raw_data;
        data_filter.idx++;
    
        memcpy(cpraw, data_filter.raw, sizeof(data_filter.raw));
        printk("%s: :~~~~david cpraw_CP1 = %u %u %u %u %u %u\n",__func__, cpraw[0], cpraw[1], cpraw[2], cpraw[3], cpraw[4], sizeof(data_filter.raw));  
        stk3a6x_als_bubble_sort(cpraw, sizeof(cpraw) / sizeof(cpraw[0]));
        mid_als = cpraw[STK_FIR_LEN / 2];
        als_raw_data = mid_als;
        printk("%s: :~~~~cpraw_CP2 = %u %u %u %u %u %u\n",__func__, cpraw[0], cpraw[1], cpraw[2], cpraw[3], cpraw[4], als_raw_data);  
    }


/*
        memset(cpraw, 0x00, sizeof(cpraw) / sizeof(cpraw));
        if(stk3a6x_als_data->fir.number < STK_FIR_LEN)
        {                
            stk3a6x_als_data->fir.raw[stk3a6x_als_data->fir.number] = als_raw_data;
            stk3a6x_als_data->fir.number++;
            stk3a6x_als_data->fir.idx++;
        }
        else
        {
            fir_index = stk3a6x_als_data->fir.idx % stk3a6x_als_data->fir.number;
            stk3a6x_als_data->fir.raw[fir_index] = als_raw_data;
            stk3a6x_als_data->fir.idx++;

            memcpy(cpraw, stk3a6x_als_data->fir.raw, sizeof(stk3a6x_als_data->fir.raw));
            printk("%s: :~~~~david cpraw_CP1 = %d %d %d %d %d %d\n",__func__, cpraw[0], cpraw[1], cpraw[2], cpraw[3], cpraw[4], sizeof(stk3a6x_als_data->fir.raw));  
            stk3a6x_als_bubble_sort(cpraw, sizeof(cpraw) / sizeof(cpraw[0]));
            mid_als = cpraw[STK_FIR_LEN / 2];
            als_raw_data = mid_als;
            printk("%s: :~~~~cpraw_CP2 = %d %d %d %d %d %d\n",__func__, cpraw[0], cpraw[1], cpraw[2], cpraw[3], cpraw[4], als_raw_data);  
        }
        */
#endif

    return als_raw_data;
}

//add auto gain david 20210514
static bool stk3a6x_set_als_gain(struct i2c_client *client, uint16_t level)
{
    int ret = 0;
    uint8_t alsctrl_reg, gainctrl_reg, gain;
    uint8_t rx_buf[1] = {0};

    rx_buf[0] = sensor_read_reg(client, STK_ALSCTRL_REG);
    alsctrl_reg = rx_buf[0];
    rx_buf[0] = sensor_read_reg(client, STK_GAINCTRL_REG);
    gainctrl_reg = rx_buf[0];
    printk("%s: starting pre alsctrl 0x%x gainctrl 0x%x level %d\n", __func__, alsctrl_reg, gainctrl_reg, level);

    if (level == 0) {
        // Highest gain for low light :  ALS Gain X128 & C Gain X128
        // Reg[0x02] don't care
        // Write Reg[0x4E] = 0x06, GAIN_ALS_DX128 = 1, GAIN_C_DX128 = 1
        alsctrl_reg = (alsctrl_reg & 0xCF) | 0x20;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x06; //als&c2 c1 data gain x128
        gain = 128;
        printk("%s aft alsctrl=0x%x gainctrl=0x%x level =%d\n", __func__, alsctrl_reg, gainctrl_reg, level);
    } else if (level == 1) {
        // Middle gain for middle light :  ALS Gain X16 & C Gain X16
        // Write Reg[0x02] = 0x21, ALS IT = 50ms, GAIN_ALS = 2' b10
        // Write Reg[0x4E] = 0x20, GAIN_ALS_DX128 = 0, GAIN_C_DX128 = 0, GAIN_C = 2' b10
        alsctrl_reg = (alsctrl_reg & 0xCF) | 0x20;//C1 gain 16 
        gainctrl_reg = (gainctrl_reg & 0xC9) |0x20; //als&c2 c1 data gain x16
        gain = 16;
        printk("%s aft alsctrl=0x%x gainctrl=0x%x level =%d\n", __func__, alsctrl_reg, gainctrl_reg, level);
    } else if (level == 2) {
        //Lowest gain for high light :  ALS Gain X1 & C Gain X1
        //Write Reg[0x02] = 0x01, ALS IT = 50ms, GAIN_ALS = 2' b00
        //Write Reg[0x4E] = 0x00, GAIN_ALS_DX128 = 0, GAIN_C_DX128 = 0, GAIN_C = 2' b00
        alsctrl_reg = (alsctrl_reg & 0xCF) | 0x00; //C1 gain 1
        gainctrl_reg = (gainctrl_reg & 0xC9) |0x00; //als&c2 c1 data gain x1
        gain = 1;
        printk("%s aft alsctrl=0x%x gainctrl=0x%x level =%d\n", __func__, alsctrl_reg, gainctrl_reg, level);
    } else {
        printk("%s aft alsctrl=0x%x gainctrl=0x%x level =%d\n", __func__, alsctrl_reg, gainctrl_reg, level);
        return false;
    }

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
    //restart FSM
    rx_buf[0] = sensor_read_reg(client, 0x5F);
    ret = sensor_write_reg(client, 0x5F, (rx_buf[0] | 0x01));
    if(ret < 0)
    {
        printk("stk %s sensor_write_reg err \n", __func__);
    }

    stk3a6x_als_gain = gain;

    return true;
}

static bool stk3a6x_als_auto_gain(struct i2c_client *client, uint16_t *als_data)
{
    bool result = false;

    //printk("%s: starting\n", __func__);
    if (((als_data[0]) > STK_MAX_ALS_THD || (als_data[4] > STK_MAX_ALS_THD)) && (stk3a6x_als_gain_level < 2)) {
        // Reduce gain
        stk3a6x_als_gain_level++;
        result = stk3a6x_set_als_gain(client, stk3a6x_als_gain_level);
    } else if (((als_data[0] < STK_MIN_ALS_THD) && (als_data[4] < STK_MIN_ALS_THD)) && (stk3a6x_als_gain_level > 0)) {
        // Raise gain
        stk3a6x_als_gain_level--;
        result = stk3a6x_set_als_gain(client, stk3a6x_als_gain_level);
    }
    return result;
}
//add auto gain david 20210514 end

//read fifo begin
#ifdef STK_FIFO
static void stk3a6x_get_max_min(struct i2c_client *client, uint16_t *data_array, int data_len)
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

static void stk3a6x_fifo_data_tran(int data_mode, uint16_t tran_data_len)
{
    int i = 0;
    uint16_t data_len = stk3a6x_fifo.byte_per_frame;

    switch(data_mode)
    {
    case PS_ONLY:
        for( i= 0; i < tran_data_len; i++){
            stk3a6x_fifo.frame[i].ps = (stk3a6x_fifo.data[0 + i * data_len] << 8)|(stk3a6x_fifo.data[1+ i * data_len]);
#ifdef FIFO_DATA_DEBUG
            printk("%s: fifo data i~ps=\t%d\t%d\n", __func__, i, stk3a6x_fifo.frame[i].ps);
            usleep_range(1000,1100);//for donot lost log
#endif
        }
        break;
    case ALS_C1:
        for( i= 0; i < tran_data_len; i++){
            stk3a6x_fifo.frame[i].als = (stk3a6x_fifo.data[0 + i * data_len] << 8)|(stk3a6x_fifo.data[1+ i * data_len]);
            stk3a6x_fifo.frame[i].c1  = (stk3a6x_fifo.data[2 + i * data_len] << 8)|(stk3a6x_fifo.data[3+ i * data_len]);
#ifdef FIFO_DATA_DEBUG
            printk("%s: fifo data i~c1=\t%d\t%d\t%d\n", __func__, i,
                                                    stk3a6x_fifo.frame[i].als,
                                                    stk3a6x_fifo.frame[i].c1);
            usleep_range(1000,1100);//for donot lost log
#endif
        }
        break;
    case ALS_C2:
        for( i= 0; i < tran_data_len; i++){
            stk3a6x_fifo.frame[i].als = (stk3a6x_fifo.data[0 + i * data_len] << 8)|(stk3a6x_fifo.data[1 + i * data_len]);
            stk3a6x_fifo.frame[i].c2  = (stk3a6x_fifo.data[2 + i * data_len] << 8)|(stk3a6x_fifo.data[3 + i * data_len]);
#ifdef FIFO_DATA_DEBUG
            printk("%s: fifo data i~c2=\t%d\t%d\t%d\n", __func__, i,
                                                    stk3a6x_fifo.frame[i].als,
                                                    stk3a6x_fifo.frame[i].c2);
            usleep_range(1000,1100);//for donot lost log
#endif
        }
        break;
    case ALS_C1_C2_PS:
        for( i= 0; i < tran_data_len; i++){
            stk3a6x_fifo.frame[i].als = (stk3a6x_fifo.data[0 + i * data_len] << 8)|(stk3a6x_fifo.data[1 + i * data_len]);
            stk3a6x_fifo.frame[i].c1  = (stk3a6x_fifo.data[2 + i * data_len] << 8)|(stk3a6x_fifo.data[3 + i * data_len]);
            stk3a6x_fifo.frame[i].c2  = (stk3a6x_fifo.data[4 + i * data_len] << 8)|(stk3a6x_fifo.data[5 + i * data_len]);
            stk3a6x_fifo.frame[i].ps  = (stk3a6x_fifo.data[6 + i * data_len] << 8)|(stk3a6x_fifo.data[7 + i * data_len]);
#ifdef FIFO_DATA_DEBUG
            printk("%s: fifo data i~ps=\t%d\t%d\t%d\t%d\t%d\n", __func__, i,
                                                    stk3a6x_fifo.frame[i].als,
                                                    stk3a6x_fifo.frame[i].c1,
                                                    stk3a6x_fifo.frame[i].c2,
                                                    stk3a6x_fifo.frame[i].ps);
            usleep_range(1000,1100);//for donot lost log
#endif
        }
        break;
    default:
        break;
    }
}

static int stk3a6x_get_fifo_data(struct i2c_client *client, uint16_t *data)
{
    int cnt = 0, min_index = 0, max_index = 0, ret = 0;
    char buffer[2] = {0};
    uint16_t fifo_read_len = 0;

    if (!stk3a6x_als_data->als_enabled) {
        printk("%s:als disable !!!,donot read fifo data\n", __func__);
        return 0;
    }
    printk("%s in als_enabled=%d\n", __func__, stk3a6x_als_data->als_enabled);

    memset((void *)stk3a6x_fifo.frame, 0, sizeof(struct stk3a6x_fifo_frame) * STK_FIFO_MAX_FRAME);
    memset((void *)stk3a6x_fifo.data, 0, sizeof(uint8_t) * STK_FIFO_MAX_LEN);

    //stk3a6x_get_fifo_info(client, stk3a6x_fifo.first_fifo_read);
    buffer[0] = STK_FIFOFCNT1_REG;
    ret = sensor_rx_data(client, buffer, 2);
    if(ret){
        printk("%s:read fifo cnt ERR\n", __func__);
    }
    stk3a6x_fifo.frame_cnt = (buffer[0] << 8) | buffer[1];
    fifo_read_len = stk3a6x_fifo.frame_cnt / 2;
    stk3a6x_fifo.fifo_frame_bytes = fifo_read_len * stk3a6x_fifo.byte_per_frame; // 1 Frame = ALS+C1+C2+PS = 8 bytes
    printk("%s: byte_per_frame=%d frame_cnt =%d (%d) fifo_frame_bytes = %d buf0=0x%x buf1=0x%x\n",
        __func__,
        stk3a6x_fifo.byte_per_frame,
        stk3a6x_fifo.frame_cnt,
        fifo_read_len,
        stk3a6x_fifo.fifo_frame_bytes,
        buffer[0],
        buffer[1]);

    if (stk3a6x_fifo.frame_cnt != 0) {
        if (stk3a6x_fifo.frame_cnt > STK_FIFO_MAX_FRAME){
            //never happen
            stk3a6x_fifo.frame_cnt = STK_FIFO_MAX_FRAME;  //FIFO_DATA_SEL = ALS+C1+C2+PS mode, Max FIFO frame = 256
        }
        /*read fifo data*/
        stk3a6x_fifo.data[0] = STK_FIFO_OUT_REG;
        ret = sensor_rx_data(client, stk3a6x_fifo.data, stk3a6x_fifo.fifo_frame_bytes);
        if(ret){
            printk("%s:read fifo cnt ERR\n", __func__);
        }
        stk3a6x_fifo_data_tran(stk3a6x_fifo.fifo_data_sel, fifo_read_len);
        /*get min max index*/
        for(cnt = 1; cnt < fifo_read_len; cnt++)
            if(stk3a6x_fifo.frame[cnt].als < stk3a6x_fifo.frame[min_index].als)
                min_index = cnt;
        for(cnt= 1; cnt < fifo_read_len; cnt++)
            if(stk3a6x_fifo.frame[cnt].als > stk3a6x_fifo.frame[max_index].als)
                max_index = cnt;

        data[0] = stk3a6x_fifo.frame[min_index].als;//als data
        data[1] = stk3a6x_fifo.frame[min_index].c1;//gdata 550
        data[2] = stk3a6x_fifo.frame[min_index].c2;//als data
        data[3] = stk3a6x_fifo.frame[min_index].ps;//gdata 550
        data[4] = stk3a6x_fifo.frame[max_index].als;//als data
        data[5] = stk3a6x_fifo.frame[max_index].c1;//gdata 550
        data[6] = stk3a6x_fifo.frame[max_index].c2;//als data
        data[7] = stk3a6x_fifo.frame[max_index].ps;//gdata 550

        printk("%s: min_als_ps=\t%d\t%d\t%d\t%d\t%d\n", __func__, data[0], data[1], data[2], data[3], min_index);
        printk("%s: max_als_ps=\t%d\t%d\t%d\t%d\t%d\n", __func__, data[4], data[5], data[6], data[7], max_index);
    }else{
        printk("%s:fifo frame_cnt ERR =%d\n", __func__, stk3a6x_fifo.frame_cnt);
    }
    //auto_gain = check_auto_gain(scp_service,port_handle,als_raw_data);
    /*clear fifo*/
    buffer[0] = STK_FIFOCTRL1_REG;//0x60;
    buffer[1] = STK_FIFOCTRL1_VAL;
    ret = sensor_tx_data(client, buffer, 2);
    if(ret){
        printk("%s: clear fifo ERR\n", __func__);
    }

    stk3a6x_als_data->last_als = data[0];
    stk3a6x_als_data->last_data_g = data[1];
    stk3a6x_als_data->last_data_c = data[2];
    
    return ret;
}
//read fifo end
#endif
static int stk3a6x_light_report_value(struct i2c_client *client)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);    
    int result = 0;
    bool auto_gain = false;
    int flag_data = 0;
    uint8_t count = 0;
    uint16_t als_data[5] = {0};//f/c
    uint16_t fifo_data[8] = {0};//f/c
    unsigned char buffer[10] = {0};
    uint32_t als_raw = 0.0;
    uint32_t c_raw = 0.0;

    if(stk3a6x_als_data->debug_cnt == 50000)
        stk3a6x_als_data->debug_cnt = 0;
    stk3a6x_als_data->debug_cnt ++;

    if ((stk3a6x_als_data->debug_cnt %20 == 1))
        stk3a6x_dump_reg(client);

    if(sensor->ops->read_len < 2)    //sensor->ops->read_len = 1
    {
        printk("%s:lenth is error,len=%d\n",__func__, sensor->ops->read_len);
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

//read fifo
#ifdef STK_FIFO
        result = stk3a6x_get_fifo_data(client, fifo_data);
        printk("%s, david get fifo data als-c1-c2-ps:\t%d\t%d\t%d\t%d\n",
            __func__, fifo_data[0], fifo_data[1], fifo_data[2], fifo_data[3]);
#endif

    if (!first_als) {
        if ((flag_data & STK_FLG_ALSDR_MASK)) {
            auto_gain = stk3a6x_als_auto_gain(client, als_data);
        }
    } else {
        printk("%s, first als data\n", __func__);
        first_als = false;
    }

    if (auto_gain) {
        // last data
        als_raw = last_als;
        c_raw = last_data_c;
    } else {
        als_raw = als_data[0]  * 128 / stk3a6x_als_gain; // als data
        c_raw = als_data[4] * 128 / stk3a6x_als_gain;// c data
        //todo
        //als = stk3a6x_get_als_lux(als, c_raw);
    }

#ifdef STK_DEBUG_PRINTF
    printk("%s:lux_als__f_c_gain= %u\t%u\t%u\t%u\t%u\t%u\t%u\n",
        __func__,als_raw, als_data[0], als_data[1], als_data[2], als_data[3], als_data[4], stk3a6x_als_gain);
#endif    
    last_als = als_raw;
    last_data_c = c_raw;

    //als_raw = stk3a6x_als_fir(client, als_raw);
    als_raw = fifo_data[0];
    printk("%s:als_raw=%d\n",__func__, als_raw);
    result = stk3a6x_light_report_abs_value(sensor->input_dev, als_raw);

    return result;
}

struct sensor_operate light_stk3a6x_ops = {
    .name                = "ls_stk3a6x",
    .type                = SENSOR_TYPE_LIGHT,    //sensor type and it should be correct
    .id_i2c              = LIGHT_ID_STK3A6X,        //i2c id number
    .read_reg            = STK_DATA1_ALS_REG,            //read data
    .read_len            = 2,                //data length
    .id_reg              = 0x3E,//SENSOR_UNKNOW_DATA,        //read device id from this register
    .id_data             = STK3A6X_PID,//SENSOR_UNKNOW_DATA,        //device id
    .precision           = 16,                //16 bits
    .ctrl_reg            = STK_STATE_REG,            //enable or disable 
    .int_status_reg      = SENSOR_UNKNOW_DATA,            //intterupt status register
    .range               = {2,65535},        //range
    .brightness          ={5,255},     //brightness    
    .trig                = IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,        
    .active              = stk3a6x_light_sensor_active,    
    .init                = stk3a6x_light_sensor_init,
    .report              = stk3a6x_light_report_value,
};

static struct sensor_operate *light_get_ops(void)
{
    return &light_stk3a6x_ops;
}

static int __init light_stk3a6x_init(void)
{
    struct sensor_operate *ops = light_get_ops();
    int result = 0;
    int type = ops->type;
    result = sensor_register_slave(type, NULL, NULL, light_get_ops);
    return result;
}

static void __exit light_stk3a6x_exit(void)
{
    struct sensor_operate *ops = light_get_ops();
    int type = ops->type;
    sensor_unregister_slave(type, NULL, NULL, light_get_ops);
}


module_init(light_stk3a6x_init);
module_exit(light_stk3a6x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3a6x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

