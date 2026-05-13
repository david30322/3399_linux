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
#include "ps_stk6b1x.h"

#include "stk6b1x_ps_ver.h"

uint8_t stk6b1x_pspid_list[] = {0xA1, 0xA4, 0xA5, 0xA6, 0xA8, 0xA9};
/*
static int stk6b1x_sensor_check_id();
static int stk6b1x_sensor_hw_init();
struct sensor_driver* stk6b1x_ps_init();
static int stk6b1x_sensor_set_cali_data(void *cali_data);
static int stk6b1x_sensor_rate(int32_t sampling_period_us);
static int stk6b1x_ps_enable();
static int stk6b1x_sensor_disable();
static int stk6b1x_sensor_set_status();
static int stk6b1x_sensor_get_status();
static void stk6b1x_sensor_cali_cmd_handle(int cal_cmd, int cali_type, int golden_sample);
static void stk6b1x_sensor_get_cali_data();
static int stk6b1x_ps_get_data(struct sensor_data *sensor_data, uint64_t timestamp);
static int stk6b1x_sensor_set_mode(int mode);
static int stk6b1x_sensor_get_fifo_data(struct sensor_data *sensor_data);
static int stk6b1x_sensor_flush(int sensor);
static int stk6b1x_sensor_selftest();

sensor_info_t stk6b1x_prox_info =
{
    .name = "prox_stk6b1x",
    .vendor = "sensortek",
    .version = 0,
    .sensor_type = SENSOR_TYPE_PROXIMITY,
    .maxrange = 5,
    .resolution = 1.0f,
    .power = 0.15f,
    .mindelay_us = 0,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .maxdelay_us = 0,
};

struct sensor_hw_info stk6b1x_prox_i2c_info =
{
    .position = 0,
    .i2c_info = {
        .interface_freq = 400,
        .interface_num = 0,
        .slave_addr = 0x48,
        .reg_addr_len = 1
    },
};

struct sensor_driver stk6b1x_sensor_driver =
{
    .sensor_info = &stk6b1x_prox_info,
    .sensor_data = {0},
    .report_mode = POLLING_MODE,
    .sensor_driver_handle = DRV_PROXIMITY,
    .enabled = 0, //int enabled;
    .sampling_period_us = 110000, //110ms -> polling timer ,default 320000us;
    .sampling_timer = 40000,
    .max_report_latency_us = 0,
    .sensor_support_status = 0,

    .init = stk6b1x_ps_init,
    .check_id = stk6b1x_sensor_check_id,
    .hw_init = stk6b1x_sensor_hw_init,
    .set_cali_data = stk6b1x_sensor_set_cali_data,
    .set_rate = stk6b1x_sensor_rate,
    .activate = stk6b1x_ps_enable,
    .deactivate = stk6b1x_sensor_disable,
    .set_status = stk6b1x_sensor_set_status,
    .get_status = stk6b1x_sensor_get_status,
    .get_data = stk6b1x_ps_get_data,
    .set_mode = stk6b1x_sensor_set_mode,
    .get_fifo_data = stk6b1x_sensor_get_fifo_data,
    .self_test = stk6b1x_sensor_selftest,
    .flush = stk6b1x_sensor_flush,
    .cali_cmd = stk6b1x_sensor_cali_cmd_handle,
    .get_cali_data = stk6b1x_sensor_get_cali_data,
};

proximity_cali_params stk6b1x_prox_cali_params =
{
    .ps_threshold_high = 6286,
    .ps_threshold_low = 6114,
    .dyna_noise = 8000,
    .dyna_noise_offset = 10,
    .dyna_noise_max = 8000,
    .noise_high_add = 286,
    .noise_low_add = 114,
    .ps_threshold_high_def = 6286,
    .ps_threshold_low_def = 6114,
    .min_noise = 100,
    .max_noise = 6000
};

struct ps_filter stk6b1x_prox_filter =
{
    .close_flag = 0,
    .far_away_flag = 0,
    .status_backup = 5.0
};
*/

static uint8_t enable_state;
static uint8_t enable_flag;
static int last_proximit_status = -1;
static struct stk6b1x_ps_data stk6b1x_pdata;

stk6b1x_register_table stk6b1x_ps_def_reg_table[] =
{
    {STK6B1X_REG_PS_DGAIN,          STK6B1X_PS_GAIN256,                                             STK6B1X_PS_DGAIN_MASK},
    {STK6B1X_REG_PS_AGAIN,          STK6B1X_PS_AGAIN_2_0,                                           STK6B1X_PS_AGAIN_MASK},

    {STK6B1X_REG_PS_IT,             STK6B1X_PS_IT48,                                               STK6B1X_PS_IT_MASK},
    {STK6B1X_REG_PS_WAIT1,          STK6B1X_WAIT_H(STK6B1X_PS_WAIT20),                                  0xFF},
    {STK6B1X_REG_PS_WAIT2,          STK6B1X_WAIT_L(STK6B1X_PS_WAIT20),                                  0xFF},
    {STK6B1X_REG_PS_LED_SET,        STK6B1X_PS_IRDR_5_46mA,                                         STK6B1X_PS_IRDR_MAKS},
    {0x53,                      0xDD,                                                       0xFF},
#ifdef STK6B1X_GPIO_PS
    {STK6B1X_REG_GPIO_SET1,         STK6B1X_GPIO_EN_MEASURE_MASK,                                   STK6B1X_GPIO_EN_MEASURE_MASK},
    {STK6B1X_REG_GPIO_SET2,         0x40,                                                       0x40},
    {STK6B1X_REG_GPIO_SET23,        STK6B1X_GPIO_FREQ_LOST_THD_5_PERCENT,                           0x1F},

    {STK6B1X_REG_GPIO_SET24,        STK6B1X_WAIT_H(STK6B1X_GPIO_TIMER_20MS),                            0x3F},
    {STK6B1X_REG_GPIO_SET25,        STK6B1X_WAIT_L(STK6B1X_GPIO_TIMER_20MS),                            0xFF},
    {STK6B1X_REG_GPIO_SET17,        0x00,                                                       0x1F}, //PS TD, At least 323.25us when RESET_FSM(0x6B[6]) enable
    {STK6B1X_REG_GPIO_SET18,        0x01,                                                       0xFF},
    {STK6B1X_REG_GPIO_SET19,        0xCD,                                                       0xFF},
    {0x62,                      0x98,                                                       0xFE},
#endif
};

#if (defined(STK6B1X_GPIO_PS) || defined(STK6B1X_GPIO_ALS))
void stk6b1x_ps_gpio_enable(struct i2c_client *client, bool enable)
{
    uint8_t i2c_flag_reg = enable ? STK6B1X_GPIO_PS_SEL_MASK : 0;
    int32_t err = 0;

    if (stk6b1x_pdata.gpio_enable == enable)
    {
        STK_LOG("Already Set\n");
        return;
    }

    err = sensor_write_reg_mask(client, STK6B1X_REG_GPIO_SET0, i2c_flag_reg, STK6B1X_GPIO_PS_SEL_MASK);
    if (err < 0)
    {
        STK_LOG("read modify write i2c (0x%X) error\n", STK6B1X_REG_GPIO_SET0);
        return;
    }

    stk6b1x_pdata.gpio_enable = enable;
}

static int32_t stk6b1x_ps_fsm_pause(struct i2c_client *client, bool is_pause)
{
    int ret = 0;
    uint8_t i2c_data = 0;
#ifdef STK6B1X_GPIO_PS
    uint8_t reg_data = 0, reg = 0;
#endif
    i2c_data = is_pause ? STK6B1X_FSM_PS_PAUSE_MASK : 0;
    reg = STK6B1X_REG_FSM_CTRL;
    ret = sensor_write_reg_mask(client, STK6B1X_REG_FSM_CTRL, i2c_data, STK6B1X_FSM_PS_PAUSE_MASK);

    if (ret < 0)
    {
        STK_LOG("read modify write i2c (0x%X) error\n", STK6B1X_REG_FSM_CTRL);
        return ret;
    }

#ifdef STK6B1X_GPIO_PS
    if (!is_pause)
    {
        reg_data = 0;
        reg = STK6B1X_REG_GPIO_SET1;
        ret = sensor_write_reg_mask(client, reg, reg_data, STK6B1X_GPIO_EN_MEASURE_MASK);
        if (ret < 0)
        {
            STK_LOG("read modify write i2c (0x%X) error\n", STK6B1X_REG_GPIO_SET1);
            return ret;
        }

        reg_data = STK6B1X_GPIO_EN_MEASURE_MASK;
        reg = STK6B1X_REG_GPIO_SET1;
        ret = sensor_write_reg_mask(client, reg, reg_data, STK6B1X_GPIO_EN_MEASURE_MASK);
        if (ret < 0)
        {
            STK_LOG("read modify write i2c (0x%X) error\n", STK6B1X_REG_GPIO_SET1);
            return ret;
        }
    }
#endif
    return ret;
}

static void stk6b1x_ps_gpio_lost_handle(struct i2c_client *client)
{
    int32_t ret = 0;
    uint8_t i2c_data[4] = {0}, reg_addr;
    uint8_t screen_hz = stk6b1x_pdata.display_freq, ps_duty = 0;
    uint32_t measure_time = 0;
    uint32_t ps_td = 0, target_timer = 0;
    uint32_t temp_target_timer = 0;
    uint32_t temp_ps_td = 0;

    i2c_data[0] = STK6B1X_REG_GPIO_SET7;
    ret = sensor_rx_data(client, i2c_data, 3);

    if (ret < 0)
    {
        STK_LOG("read i2c measre time fail\n");
        return ;
    }

    measure_time = ((((i2c_data[0] & 0x1F) << 16) | (i2c_data[1] << 8) | i2c_data[2]) * 3) / 4;
    STK_LOG("Measure time:%dus(0x%X%X%X)\n", measure_time, i2c_data[1], i2c_data[2], i2c_data[3]);

    switch (measure_time / 1000)
    {
        case 20:
            screen_hz = 48;
            target_timer = 208333; //us
            ps_td = 32325;//us
            ps_duty = 0;
            break;

        case 16:
            screen_hz = 60;
            target_timer = 166666; //us
            ps_td = 8704;//8982;//8774//us
            ps_duty = 0;
            break;

        case 11:
            screen_hz = 90;
            target_timer = 111111; //us
            ps_td = 32325;//us
            ps_duty = 0;
            break;

        case 8:
            screen_hz = 120;
            target_timer = 83333; //us
            ps_td = 642;//530;//32325;//us
            ps_duty = 0;
            break;

        case 6:
            screen_hz = 144;
            target_timer = 69444; //us
            ps_td = 32325;//us
            ps_duty = 0;
            break;

        default:
            STK_LOG("need to implement\n");
            break;
    }

    if (screen_hz != stk6b1x_pdata.display_freq)
    {
        stk6b1x_pdata.display_freq = screen_hz;
        STK_LOG("Current is %dHz\n", stk6b1x_pdata.display_freq);
        temp_target_timer = (uint32_t)STK6B1X_GPIO_TD_TIMER(target_timer) / 10;
        i2c_data[0] = STK6B1X_REG_GPIO_SET14;
        i2c_data[1] = (temp_target_timer >> 16) & 0x1F;
        i2c_data[2] = (temp_target_timer >> 8)  & 0xFF;
        i2c_data[3] = (temp_target_timer >> 0)  & 0xFF;
        
        ret = sensor_tx_data(client, i2c_data, 4);

        if (ret < 0)
        {
            STK_LOG("set TARGET TIMER fail\n");
        }
#ifdef STK6B1X_GPIO_PS

        if (stk6b1x_pdata.gpio_enable)
        {
            stk6b1x_ps_fsm_pause(client, true);
            temp_ps_td = (uint32_t)STK6B1X_GPIO_TD_TIMER(ps_td) / 100;
            i2c_data[0] = STK6B1X_REG_GPIO_SET17;
            i2c_data[1] = (temp_ps_td >> 16) & 0x1F;
            i2c_data[2] = (temp_ps_td >> 8)  & 0xFF;
            i2c_data[3] = (temp_ps_td >> 0)  & 0xFF;

            ret = sensor_tx_data(client, i2c_data, 4);

            if (ret < 0)
            {
                STK_LOG("set PS_TD fail\n");
            }

            i2c_data[0] = ps_duty;
            reg_addr = STK6B1X_REG_GPIO_SET28;
            ret = sensor_write_reg_mask(client, reg_addr, i2c_data[0], 0xFF);

            if (ret < 0)
            {
                STK_LOG("set PS_DUTY fail\n");
                return;
            }

            stk6b1x_ps_fsm_pause(client, false);
        }

#endif
    }
}
#endif

static int stk6b1x_sensor_check_id(struct i2c_client *client)
{
    int ret = FAIL;
    uint8_t pid_count = 0;
    int reg_data = 0;
    reg_data = sensor_read_reg(client, STK6B1X_REG_PID);

    for (pid_count = 0; pid_count < (sizeof(stk6b1x_pspid_list) / sizeof(uint8_t)); pid_count++)
    {
        if ( reg_data == stk6b1x_pspid_list[pid_count])
        {
            ret = NO_ERROR;
            break;
        }

        ret = FAIL;
    }

    if (ret != NO_ERROR)
        STK_LOG("proximity check id failed!\n");
    else
        STK_LOG("proximity check id successed!\n");

    return ret;
}

static int stk6b1x_set_ps_thd(struct i2c_client *client, uint32_t threshold_high, uint32_t threshold_low)
{
    int ret = 0;
    uint8_t reg_data[5] = {0};
    reg_data[0] = STK6B1X_REG_PS_THDH1;
    reg_data[1] = (uint8_t)((threshold_high & 0xFF00) >> 8);
    reg_data[2] = (uint8_t)(threshold_high & 0x00FF);
    reg_data[3] = (uint8_t)((threshold_low & 0xFF00) >> 8);
    reg_data[4] = (uint8_t)(threshold_low & 0x00FF);
    STK_LOG("h:%d l:%d\n", threshold_high, threshold_low);
    ret = sensor_tx_data(client, reg_data, 5);

    if (ret < 0)
    {
        STK_LOG("stk6b1x_set_ps_thd failed!");
        return ret ;
    }

    return ret;
}

static int stk6b1x_sensor_hw_init(struct i2c_client *client)
{
    int i, ret = -1;

    for (i = 0; i < (sizeof(stk6b1x_ps_def_reg_table) / sizeof(stk6b1x_register_table)); i++)
    {
        ret = sensor_write_reg_mask(client, stk6b1x_ps_def_reg_table[i].address, stk6b1x_ps_def_reg_table[i].value, stk6b1x_ps_def_reg_table[i].mask);

        if (ret < 0)
        {
            STK_LOG("proximity init failed!\n");
            return ret;
        }
    }

    stk6b1x_set_ps_thd(client, STK6B1X_HT_N_CT, STK6B1X_LT_N_CT);
    STK_LOG("proximity init success!\n");
    return ret;
}

static int stk6b1x_ps_init(struct i2c_client *client)
{
    int ret = FAIL;
    ret = stk6b1x_sensor_check_id(client);

    if (ret < NO_ERROR)
    {
        STK_LOG("proximity check id error ret = %d!\n", ret);
        return 0;
    }

    ret = stk6b1x_sensor_hw_init(client);

    if (ret < NO_ERROR)
    {
        STK_LOG("proximity hw init error ret = %d!\n", ret);
        return 0;
    }

    stk6b1x_pdata.ht_n_ct           = STK6B1X_HT_N_CT;
    stk6b1x_pdata.lt_n_ct           = STK6B1X_LT_N_CT;
    stk6b1x_pdata.fac_ct            = STK6B1X_DEFAULT_CT;
    stk6b1x_pdata.psi_set           = 0xFFFF;
    stk6b1x_pdata.ps_stat_data[0]   = 0;
    stk6b1x_pdata.ps_stat_data[1]   = 0;
    stk6b1x_pdata.ps_stat_data[2]   = 0xFFFF;
    stk6b1x_pdata.data_count        = 0;
    stk6b1x_pdata.smudge_update     = 0;
    stk6b1x_pdata.psa               = 0x0;
    stk6b1x_pdata.psi               = 0xFFFF;
    stk6b1x_pdata.last_ps_psi       = 0xFFFF;
    if (stk6b1x_pdata.tracking_time == 0)
        stk6b1x_pdata.tracking_time = 1000000 / 1000; //ms
    stk6b1x_pdata.compensation_target = STK6B1X_TC_TRACKING_TIME / stk6b1x_pdata.tracking_time;
    return 0;
}
/*
static int stk6b1x_sensor_set_cali_data(void *cali_data)
{
    if (cali_data == NULL)
        return NO_ERROR;

    return proximity_sensor_set_cali_data(&stk6b1x_prox_cali_params, cali_data);
}

static int stk6b1x_sensor_rate(int32_t sampling_period_us)
{
    (void)sampling_period_us;
    return NO_ERROR;
}
*/
static int stk6b1x_ps_enable(struct i2c_client *client, int enable, int rate)
{
    int ret = FAIL;
    uint8_t buf[2] = {STK6B1X_REG_ENABLE, 0x00};
    buf[1] = sensor_read_reg(client, buf[0]);
    buf[1] |= (STK6B1X_STATE_EN_PS_MASK | STK6B1X_STATE_EN_PS_WAIT_MASK);

    if(enable){
        if (enable_state != 1)
        {
            ret = sensor_write_reg_mask(client, buf[0], buf[1], 0xFF);
            if (ret < 0)
            {
                STK_LOG("proximity enable %d sensor failed! ret=%d\n", enable, ret);
                return ret;
            }

            enable_state = 1;
            enable_flag = 1;
            ret = NO_ERROR;
            STK_LOG("proximity enable sensor successed!\n");
#ifdef STK6B1X_GPIO_PS
            stk6b1x_ps_gpio_enable(client, true);
#endif
        }
        else
        {
            ret = NO_ERROR;
            STK_LOG("already enabled!\n");
        }
    }else{
        buf[1] &= (uint8_t)(~(STK6B1X_STATE_EN_PS_MASK | STK6B1X_STATE_EN_PS_WAIT_MASK));

        if (enable_state != 0)
        {
        /*
            stk6b1x_prox_filter.status_backup = 5.0f;
            stk6b1x_prox_filter.close_flag = 0;
            stk6b1x_prox_filter.far_away_flag = 0;
            */
            ret = sensor_write_reg_mask(client, buf[0], buf[1], 0xFF);

            if (ret >= 0)
            {
                enable_state = 0;
                ret = NO_ERROR;
                STK_LOG("proximity disable sensor successed!\n");
            }
            else
            {
                STK_LOG("proximity disable sensor failed! error! ret=%d\n", ret);
                return ret;
            }
#ifdef STK6B1X_GPIO_PS
            stk6b1x_ps_gpio_enable(client, false);
#endif
        }
        else
        {
            ret = NO_ERROR;
            STK_LOG("already disabled!");
        }

        last_proximit_status = -1;
        stk6b1x_pdata.ps_stat_data[0]   = 0;
        stk6b1x_pdata.ps_stat_data[1]   = 0;
        stk6b1x_pdata.ps_stat_data[2]   = 0xFFFF;
        stk6b1x_pdata.data_count        = 0;
        stk6b1x_pdata.smudge_update     = 0;
        stk6b1x_pdata.psa               = 0x0;

        if (stk6b1x_pdata.psi_set != 0xFFFF)
        {
            stk6b1x_pdata.last_ps_psi   = stk6b1x_pdata.psi;
        }
    }
    stk6b1x_pdata.ps_last_status    = -1;
    stk6b1x_pdata.ps_need_report = false;

    return ret;
}
/*
static int stk6b1x_sensor_disable()
{
    int ret = FAIL;
    uint8_t buf[2] = {STK6B1X_REG_ENABLE, 0x00};
    sensor_read_reg(client, &buf[0], &buf[1], 1, 0xFF);
    buf[1] &= (uint8_t)(~(STK6B1X_STATE_EN_PS_MASK | STK6B1X_STATE_EN_PS_WAIT_MASK));

    if (enable_state != 0)
    {
        stk6b1x_prox_filter.status_backup = 5.0f;
        stk6b1x_prox_filter.close_flag = 0;
        stk6b1x_prox_filter.far_away_flag = 0;
        ret = sensor_write_reg_mask(client, &buf[0], &buf[1], 1, 0xFF);

        if (ret >= 0)
        {
            enable_state = 0;
            ret = NO_ERROR;
            STK_LOG("proximity disable sensor successed!\n");
        }
        else
        {
            STK_LOG("proximity disable sensor failed! error! ret=%d\n", ret);
            return ret;
        }
#ifdef STK6B1X_GPIO_PS
        stk6b1x_ps_gpio_enable(false);
#endif
    }
    else
    {
        ret = NO_ERROR;
        STK_LOG("already disabled!");
    }

    last_proximit_status = -1;
    stk6b1x_pdata.ps_last_status    = -1;
    stk6b1x_pdata.ps_stat_data[0]   = 0;
    stk6b1x_pdata.ps_stat_data[1]   = 0;
    stk6b1x_pdata.ps_stat_data[2]   = 0xFFFF;
    stk6b1x_pdata.data_count        = 0;
    stk6b1x_pdata.smudge_update     = 0;
    stk6b1x_pdata.psa               = 0x0;

    if (stk6b1x_pdata.psi_set != 0xFFFF)
    {
        stk6b1x_pdata.last_ps_psi   = stk6b1x_pdata.psi;
    }

    return ret;
}

static int stk6b1x_sensor_set_status()
{
    return NO_ERROR;
}


static int stk6b1x_sensor_get_status(struct i2c_client *client)
{
    uint8_t reg_addr = STK6B1X_REG_PS_FLAG1;
    uint8_t reg_data = STK6B1X_FLG_PSDR_MASK;
    return sensor_i2c_check_reg_data(client, &reg_addr, &reg_data, STK6B1X_FLG_PSDR_MASK);
}

static void stk6b1x_sensor_cali_cmd_handle(int cal_cmd, int cali_type, int golden_sample)
{
    proximity_sensor_cali_cmd_handle(cal_cmd, cali_type, golden_sample);
}


static void stk6b1x_sensor_get_cali_data()
{
    proximity_sensor_get_cali_data();
}
*/

static int stk6b1x_prx_val(struct i2c_client *client)
{
    int ret = NO_ERROR;
    uint8_t reg_addr, reg_value[2] = {0};
    uint8_t ps_invalid_flag;
    uint8_t bgir_out_of_range = 0;
    uint16_t bgir_raw;
    reg_addr = STK6B1X_REG_PS_FLAG1;
    reg_value[0] = sensor_read_reg(client, STK6B1X_REG_PS_FLAG1);

    ret = reg_value[0];
    if (ret != NO_ERROR)
    {
        STK_LOG("get PS invald flag error! ret = %d\n", ret);
        return FAIL;
    }

    reg_value[0] = STK6B1X_REG_PS_BGIR_DATA1;
    ret = sensor_rx_data(client, reg_value, 2);

    if (ret != NO_ERROR)
    {
        STK_LOG("get PS invald flag error! ret = %d\n", ret);
        return FAIL;
    }

    bgir_raw = (uint16_t)(reg_value[0] << 8 | reg_value[1]);

    if (bgir_raw > STK6B1X_PS_BGIR_THRESHOLD)
    {
        bgir_out_of_range = true;
        STK_LOG("BGIR invalid, BGIR: %d", bgir_raw);
    }

    if ((ps_invalid_flag & STK6B1X_FLG_INVALID_PS_MASK) || bgir_out_of_range)
    {
        ret = FAIL;
    }

    return ret;
}

static void stk6b1x_ps_smudge_judgement(uint16_t ps_raw_data)
{
    uint16_t raw_data = 0.0;

    if (ps_raw_data > stk6b1x_pdata.psi)
    {
        raw_data = ps_raw_data - stk6b1x_pdata.psi;
    }

    if ((raw_data > STK6B1X_SMUDGE_DIFF) && (stk6b1x_pdata.smudge_update == 0) && (stk6b1x_pdata.psi != 0xFFFF))
    {
        stk6b1x_pdata.ps_thd_h          = (uint16_t)(stk6b1x_pdata.psi + stk6b1x_pdata.ht_n_ct * STK6B1X_PS_SMUDGE_RATIO);
        stk6b1x_pdata.ps_thd_l          = (uint16_t)(stk6b1x_pdata.psi + stk6b1x_pdata.lt_n_ct * STK6B1X_PS_SMUDGE_RATIO);
        stk6b1x_pdata.smudge_update     = 1;
        stk6b1x_pdata.set_thd           = 1;
    }
}

static void stk6b1x_ps_prx_dynamicK_reset(void)
{
    stk6b1x_pdata.ps_stat_data[0] = 0;
    stk6b1x_pdata.ps_stat_data[1] = 0;
    stk6b1x_pdata.ps_stat_data[2] = 9999;
    stk6b1x_pdata.data_count = 0;
}

static void stk6b1x_ps_prx_dynamicK_threshold_reset(uint16_t ps_data)
{
    if (ps_data > stk6b1x_pdata.ps_stat_data[0])
        stk6b1x_pdata.ps_stat_data[0] = ps_data;

    if (ps_data < stk6b1x_pdata.ps_stat_data[2])
        stk6b1x_pdata.ps_stat_data[2] = ps_data;
}

void stk_prx_sorting_max_min(uint16_t* sort_array, uint16_t size_n)
{
    uint16_t i = 0;
    uint16_t max = 0, min = 0xFFFF;

    for (i = 0; i < size_n; i++)
    {
        if (max < sort_array[i])
        {
            max = sort_array[i];
        }

        if (min > sort_array[i])
        {
            min = sort_array[i];
        }
    }
    stk6b1x_pdata.ps_data_tc_filter.max = max;
    stk6b1x_pdata.ps_data_tc_filter.min = min;
}

static void stk_prx_dynamicK_compensation(uint16_t word_data,
                                   int32_t ps_status,
                                   uint16_t *ps_thd_h,
                                   uint16_t *ps_thd_l)
{
    uint16_t temp_idx = 0, ct_value = 0;
    uint16_t i = 0;
    uint32_t ps_upstat_data[3] = {0};

    if ((stk6b1x_pdata.compensation_target != 0) && (++stk6b1x_pdata.compensation_cnt < stk6b1x_pdata.compensation_target))
        return;

    stk6b1x_pdata.compensation_cnt = 0;

    if (stk6b1x_pdata.ps_data_tc_filter.idx < STK6B1X_CT_FIR_LEN)
        stk6b1x_pdata.ps_data_tc_filter.raw[stk6b1x_pdata.ps_data_tc_filter.idx % STK6B1X_CT_FIR_LEN] = word_data;
    else
    {
        memmove(&stk6b1x_pdata.ps_data_tc_filter.raw[0],
                &stk6b1x_pdata.ps_data_tc_filter.raw[1],
                sizeof(stk6b1x_pdata.ps_data_tc_filter.raw) - sizeof(stk6b1x_pdata.ps_data_tc_filter.raw[0]));
        stk6b1x_pdata.ps_data_tc_filter.raw[STK6B1X_CT_FIR_LEN - 1] = word_data;
        STK_LOG(": raw[0] = %d, raw[1] = %d, raw[2] = %d, raw[3] = %d, raw[4] = %d, raw[5] = %d\n",
                stk6b1x_pdata.ps_data_tc_filter.raw[0],
                stk6b1x_pdata.ps_data_tc_filter.raw[1],
                stk6b1x_pdata.ps_data_tc_filter.raw[2],
                stk6b1x_pdata.ps_data_tc_filter.raw[3],
                stk6b1x_pdata.ps_data_tc_filter.raw[4],
                stk6b1x_pdata.ps_data_tc_filter.raw[5]);
    }

    stk6b1x_pdata.ps_data_tc_filter.idx++;
    if (stk6b1x_pdata.ps_data_tc_filter.idx < STK6B1X_CT_FIR_LEN)
        return;

    stk_prx_sorting_max_min(stk6b1x_pdata.ps_data_tc_filter.raw, STK6B1X_CT_FIR_LEN);
    STK_LOG(":(%d) MAX = %d, MIN = %d\n", stk6b1x_pdata.ps_data_tc_filter.idx, stk6b1x_pdata.ps_data_tc_filter.max, stk6b1x_pdata.ps_data_tc_filter.min);
    if (stk6b1x_pdata.ps_data_tc_filter.idx == (10 * STK6B1X_CT_FIR_LEN))
        stk6b1x_pdata.ps_data_tc_filter.idx = STK6B1X_CT_FIR_LEN;

    if ((stk6b1x_pdata.ps_data_tc_filter.max - stk6b1x_pdata.ps_data_tc_filter.min) < STK6B1X_TC_MAX_MIN_DIFF)
    {
        temp_idx = STK6B1X_CT_FIR_LEN / 2;
        for (i = 0; i < temp_idx; i++)
        {
            ps_upstat_data[0] += stk6b1x_pdata.ps_data_tc_filter.raw[i];
        }
        ps_upstat_data[0] /= temp_idx;

        for (i = temp_idx; i < STK6B1X_CT_FIR_LEN; i++)
        {
            ps_upstat_data[1] += stk6b1x_pdata.ps_data_tc_filter.raw[i];
        }
        ps_upstat_data[1] /= temp_idx;
        STK_LOG(": ps_upstat_data[0] = %d, ps_upstat_data[1] = %d\n", ps_upstat_data[0], ps_upstat_data[1]);
    }

    ps_upstat_data[2] = ps_upstat_data[1] - ps_upstat_data[0];
    if ((ps_upstat_data[1] > ps_upstat_data[0]) && (ps_upstat_data[2] < STK6B1X_TC_SLOPE_THD))
    {
        if (ps_status == PROX_STATE_NEAR)
        {
            *ps_thd_h += ps_upstat_data[2];
            *ps_thd_l += ps_upstat_data[2];
            stk6b1x_pdata.psi = *ps_thd_h - stk6b1x_pdata.ht_n_ct;
            stk6b1x_pdata.set_thd = true;
        }
        else
        {
            ct_value = *ps_thd_h - stk6b1x_pdata.ht_n_ct;
            STK_LOG(": ct_value = %d, ht_n_ct: %d, ps_upstat_data[1] = %d\n", ct_value, stk6b1x_pdata.ht_n_ct, ps_upstat_data[1]);
            if ((ps_upstat_data[1] > ct_value) && ((ps_upstat_data[1] - ct_value) > STK6B1X_TC_CT_DIFF))
            {
                *ps_thd_h = ps_upstat_data[1] + stk6b1x_pdata.ht_n_ct;
                *ps_thd_l = ps_upstat_data[1] + stk6b1x_pdata.lt_n_ct;
                stk6b1x_pdata.psi = ps_upstat_data[1];
                stk6b1x_pdata.set_thd = true;
            }
        }
    }
}

static void stk6b1x_ps_prx_dynamicK_tracking_recali(uint16_t ps_data)
{
    uint16_t ct_value;

    if (stk6b1x_pdata.ps_last_status == PROX_STATE_FAR)
    {
        stk6b1x_pdata.ps_stat_data[1] += ps_data;
        stk6b1x_ps_prx_dynamicK_threshold_reset(ps_data);
        stk6b1x_pdata.data_count ++;

        if (stk6b1x_pdata.data_count == STK6B1X_TRACKING_QUANTI)
        {
            stk6b1x_pdata.ps_stat_data[1] /= stk6b1x_pdata.data_count;
            ct_value = stk6b1x_pdata.ps_thd_h - stk6b1x_pdata.ht_n_ct;

            //STK_LOG(":ct_value = %d, ps_data = %d\n", ct_value, ps_data);
            if ((stk6b1x_pdata.ps_stat_data[1] < ct_value) &&
                ((ct_value - stk6b1x_pdata.ps_stat_data[1]) >= 5) &&
                ((stk6b1x_pdata.ps_stat_data[0] - stk6b1x_pdata.ps_stat_data[2]) <= STK6B1X_QUANTI_RANGE))
            {
                STK_LOG("ps variation = %d\n", stk6b1x_pdata.ps_stat_data[0] - stk6b1x_pdata.ps_stat_data[2]);
                stk6b1x_pdata.ps_thd_h          = (uint16_t)(stk6b1x_pdata.ps_stat_data[1] + stk6b1x_pdata.ht_n_ct);
                stk6b1x_pdata.ps_thd_l          = (uint16_t)(stk6b1x_pdata.ps_stat_data[1] + stk6b1x_pdata.lt_n_ct);
                stk6b1x_pdata.psi               = (uint16_t)(stk6b1x_pdata.ps_stat_data[1]);
                stk6b1x_pdata.smudge_update     = 0;
                stk6b1x_pdata.set_thd           = 1;
            }

            stk6b1x_ps_prx_dynamicK_reset();
        }
    }
    else
    {
        stk6b1x_ps_prx_dynamicK_reset();
        stk6b1x_ps_smudge_judgement(ps_data);
    }

    if (stk6b1x_pdata.compensation_target != 0)
        stk_prx_dynamicK_compensation(ps_data, stk6b1x_pdata.ps_last_status, &stk6b1x_pdata.ps_thd_h, &stk6b1x_pdata.ps_thd_l);
}

static void stk6b1x_ps_prx_dynamicK_tracking_max_min(uint16_t ps_data)
{
    if (ps_data > stk6b1x_pdata.psa)
    {
        stk6b1x_pdata.psa = ps_data;
    }

    if (ps_data < stk6b1x_pdata.psi)
    {
        stk6b1x_pdata.psi = ps_data;
    }
}

static void stk6b1x_ps_prx_dynamicK_tracking_task(struct i2c_client *client, uint16_t ps_data)
{
    int ret = NO_ERROR;
    uint16_t diff, ps_invalid = 1;
    ret = stk6b1x_prx_val(client);

    if (ret == NO_ERROR)
        ps_invalid = 0;

    if ((ps_invalid == 1) || (ps_data == 0))
        return;

    stk6b1x_pdata.set_thd = 0;

    if (stk6b1x_pdata.psi_set != 0xFFFF)
    {
        stk6b1x_ps_prx_dynamicK_tracking_recali(ps_data);
    }
    else
    {
        stk6b1x_ps_prx_dynamicK_tracking_max_min(ps_data);
        diff = (stk6b1x_pdata.psa - stk6b1x_pdata.psi);

        if (diff > STK6B1X_MAX_MIN_DIFF)
        {
            stk6b1x_pdata.psi_set = stk6b1x_pdata.psi;
            stk6b1x_pdata.ps_thd_h = stk6b1x_pdata.psi + stk6b1x_pdata.ht_n_ct;
            stk6b1x_pdata.ps_thd_l = stk6b1x_pdata.psi + stk6b1x_pdata.lt_n_ct;
            stk6b1x_pdata.set_thd = 1;
        }
        stk6b1x_pdata.compensation_cnt = 0;
        memset(&(stk6b1x_pdata.ps_data_tc_filter), 0, sizeof(stk6b1x_pdata.ps_data_tc_filter));
    }

    if (stk6b1x_pdata.set_thd)
    {
        ret = stk6b1x_set_ps_thd(client, stk6b1x_pdata.ps_thd_h, stk6b1x_pdata.ps_thd_l);

        if (ret != NO_ERROR)
            return;

        stk6b1x_pdata.set_thd = 0;
    }
}

static int stk6b1x_sensor_get_raw_data(struct i2c_client *client, uint16_t *ps_data)
{
    int ret = FAIL;
    uint8_t reg_value[2];

    reg_value[0] = STK6B1X_REG_PS_DATA1;
    ret = sensor_rx_data(client, reg_value, 2);

    if (ret < 0)
    {
        STK_LOG("get data error! ret = %d\n", ret);
        return FAIL;
    }

    *ps_data = (reg_value[0] << 8) | (reg_value[1]);
    return NO_ERROR;
}

static int stk6b1x_ps_distance_last(struct i2c_client *client, int ps)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	

    if (stk6b1x_pdata.ps_need_report) {
        STK_LOG("%s:ps=%d,NF=%d\n",__func__, ps, stk6b1x_pdata.ps_last_status);
    	input_report_abs(sensor->input_dev, ABS_DISTANCE, stk6b1x_pdata.ps_last_status);
    	input_sync(sensor->input_dev);
    }
	return 0;
}


static int stk6b1x_ps_get_data(struct i2c_client *client)
{
    int ret = NO_ERROR, flag1 = 0, flag2 = 0;
    uint16_t ps_data;
    stk6b1x_pdata.ps_need_report = false;

    flag1 = sensor_read_reg(client, STK6B1X_REG_PS_FLAG1);

    if (flag1 < 0)
    {
        STK_LOG("Read STK6B1X_REG_PS_FLAG1 fail! ret = %d\n", flag1);
        return FAIL;
    }

    if (flag1 & STK6B1X_FLG_PSDR_MASK)
    {
        flag2 = sensor_read_reg(client, STK6B1X_REG_PS_FLAG2);
        if (flag2 < 0)
        {
            STK_LOG("Read STK6B1X_REG_PS_FLAG2 fail! ret = %d\n", ret);
            return FAIL;
        }

        ret = stk6b1x_sensor_get_raw_data(client, &ps_data);
        if(stk6b1x_pdata.first_report == true){
            stk6b1x_pdata.first_report == false;
            
        }

        if (ret < 0)
        {
            STK_LOG("proximity get data fail! ret = %d\n", ret);
            return FAIL;
        }

        stk6b1x_ps_prx_dynamicK_tracking_task(client, ps_data);
        if(flag2 & STK6B1X_FLG_NF_MASK){
            if(stk6b1x_pdata.ps_last_status != PS_FAR)
                stk6b1x_pdata.ps_need_report = true;
            stk6b1x_pdata.ps_last_status  = PS_FAR;

        }else if((flag2 & STK6B1X_FLG_NF_MASK) == 0){
            if(stk6b1x_pdata.ps_last_status != PS_NEAR)
                stk6b1x_pdata.ps_need_report = true;
            stk6b1x_pdata.ps_last_status  = PS_NEAR;
        }else{
            STK_LOG("proximity unknown status\n");
        }

        stk6b1x_ps_distance_last(client, ps_data);
    }

#ifdef STK6B1X_GPIO_PS
    stk6b1x_ps_gpio_lost_handle(client);
#endif

    return ret;
}

struct sensor_operate proximity_stk6b1x_ops = {
	.name				= "ps_stk6b1x",
	.type				= SENSOR_TYPE_PROXIMITY,	//sensor type and it should be correct
	.id_i2c				= PROXIMITY_ID_STK6B1X,		//i2c id number
	.read_reg			= STK6B1X_REG_PS_DATA1,			//read data
//	.addr               =0x47,
	.read_len			= 2,				        //data length
	.id_reg				= 0x3E,//SENSOR_UNKNOW_DATA,		//read device id from this register
	.id_data 			= 0xA1,//SENSOR_UNKNOW_DATA,		//device id
	.precision			= 16,				         //16 bits
	.ctrl_reg 			= STK6B1X_REG_PS_FLAG1,			//enable or disable 
	.int_status_reg 	= 0x00,			//intterupt status register
	.range				= {0,1},			//range
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,		
	.active				= stk6b1x_ps_enable,
	.init				= stk6b1x_ps_init,
	.report				= stk6b1x_ps_get_data,
	// int 	brightness[2];//backlight min_brightness max_brightness 
	// int int_ctrl_reg;
	// int (*suspend)(struct i2c_client *client);
	// int (*resume)(struct i2c_client *client);
	// struct miscdevice *misc_dev;	
};

static struct sensor_operate *proximity_get_ops(void)
{
	return &proximity_stk6b1x_ops;
}

static int __init stk6b1x_init(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, proximity_get_ops);
	return result;
}

static void __exit stk6b1x_exit(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, proximity_get_ops);
}


module_init(stk6b1x_init);
module_exit(stk6b1x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk6b1x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

