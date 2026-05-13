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
#include "ps_stk6bcx.h"

#include "stk6bcx_ps_ver.h"

uint8_t stk6bcx_ps_pid_list[STK6BCX_PID_LIST_NUM] = {0x11, 0x12};

extern void stk6bcx_dump_reg(struct i2c_client *client);
static uint8_t enable_state;
static uint8_t enable_flag;
static int last_proximit_status = -1;
static struct stk6bcx_ps_data stk6bcx_pdata;

stk6bcx_register_table stk6bcx_default_pre_register_table[] =
{
    {0xBF,                          0x01,                                                         0x01},
    {0xE8,                          0x08,                                                         0x08},
    {0xFB,                          0x40,                                                         0x40},
    {0xFD,                          0x80,                                                         0x80},
    {0xBF,                          0x00,                                                         0x01},
};

stk6bcx_register_table stk6bcx_default_register_table[] =
{
    {STK6BCX_REG_ENABLE,           STK6BCX_STATE_EN_PWR_MASK,                                   STK6BCX_STATE_EN_PWR_MASK},
    //PS
    {STK6BCX_REG_PS_DGAIN,         STK6BCX_PS_GAIN256,                                          STK6BCX_PS_GAIN_MASK},
    {STK6BCX_REG_PS_AGAIN,         STK6BCX_PS_CI4,                                              STK6BCX_PS_CI_MASK},
    {STK6BCX_REG_PS_IT1,           STK6BCX_ALPS_REG_H(STK6BCX_PS_IT95, STK6BCX_PS_IT2_H_MAKS),  STK6BCX_PS_IT2_H_MAKS},
    {STK6BCX_REG_PS_IT2,           STK6BCX_ALPS_REG_L(STK6BCX_PS_IT95),                         0xFF},
    {STK6BCX_REG_PS_WAIT1,         STK6BCX_ALPS_REG_H(STK6BCX_WAIT_1, STK6BCX_WAIT2_H_MAKS),  STK6BCX_WAIT2_H_MAKS},
    {STK6BCX_REG_PS_WAIT2,         STK6BCX_ALPS_REG_L(STK6BCX_WAIT_1),                        0xFF},
    {STK6BCX_REG_PS_LED_SET,       STK6BCX_LED_5mA,                                             STK6BCX_LED_MASK},
    {0xC3,                         0x01,                                                        0x01},
#ifdef STK6BCX_GPIO_PS
    {STK6BCX_REG_GPIO_SET1,         0x04,                                                         0x04},
    {STK6BCX_REG_GPIO_SET2,         0x09,                                                         0x09},
    {STK6BCX_REG_GPIO_SET27,        STK6BCX_ALPS_REG_H(STK6BCX_GPIO_TIMER_50MS, STK6BCX_GPIO_TIMER_HIGH_MASK),       0x3F},//gpio timer
    {STK6BCX_REG_GPIO_SET28,        STK6BCX_ALPS_REG_L(STK6BCX_GPIO_TIMER_50MS),                                     0xFF},
    {STK6BCX_REG_GPIO_SET26,        STK6BCX_GPIO_FREQ_LOST_THD_96US,                              STK6BCX_GPIO_FREQ_LOST_THD_MASK},
#endif
};

stk6bcx_register_table stk6bcx_default_ps_thd_table[] =
{
    {STK6BCX_REG_PS_THDH1,  0x00, 0xFF},
    {STK6BCX_REG_PS_THDH2,  0x00, 0xFF},
    {STK6BCX_REG_PS_THDL1,  0xFF, 0xFF},
    {STK6BCX_REG_PS_THDL2,  0xFF, 0xFF},
};


#ifdef STK6BCX_GPIO_PS
stk6bcx_gpio_config gpio_config_table[] =
{
    {.screen_hz = 60,  .target_timer = 16666, .ps_td = 1000, .ps_duty = 2, .ps_ignore = 50000},
    {.screen_hz = 90,  .target_timer = 11111, .ps_td = 1000, .ps_duty = 2, .ps_ignore = 50000},
    {.screen_hz = 120, .target_timer = 8333,  .ps_td = 1000, .ps_duty = 2, .ps_ignore = 50000},
    {.screen_hz = 144, .target_timer = 6944,  .ps_td = 5000, .ps_duty = 5, .ps_ignore = 500000},
};

void stk6bcx_ps_gpio_enable(struct i2c_client *client, bool enable)
{
    uint8_t i2c_flag_reg = enable ? STK6BCX_GPIO_PS_EN : 0;
    int32_t err = 0;

    if (stk6bcx_pdata.gpio_enable == enable)
    {
        STK_LOG("Already Set\n");
        return;
    }

    err = sensor_write_reg_mask(client, STK6BCX_REG_GPIO_SET0, i2c_flag_reg, STK6BCX_GPIO_PS_EN);
    if (err < 0)
    {
        STK_LOG("read modify write i2c (0x%X) error\n", STK6BCX_REG_GPIO_SET0);
        return;
    }

    stk6bcx_pdata.gpio_enable = enable;
}

static int32_t stk6bcx_ps_fsm_pause(struct i2c_client *client, bool is_pause)
{
    int ret = 0;
    uint8_t i2c_data = 0;
#ifdef STK6BCX_GPIO_PS
    uint8_t reg_data = 0, reg = 0;
#endif
    i2c_data = is_pause ? STK6BCX_PS_FSM_PAUSE_MASK : 0;
    reg = STK6BCX_REG_FSM_CTRL;
    ret = sensor_write_reg_mask(client, STK6BCX_REG_FSM_CTRL, i2c_data, STK6BCX_PS_FSM_PAUSE_MASK);

    if (ret < 0)
    {
        STK_LOG("read modify write i2c (0x%X) error\n", STK6BCX_REG_FSM_CTRL);
        return ret;
    }

#ifdef STK6BCX_GPIO_PS
    if (!is_pause)
    {
        reg_data = 0;
        reg = STK6BCX_REG_GPIO_SET1;
        ret = sensor_write_reg_mask(client, reg, reg_data, STK6BCX_GPIO_MEASURE_EN);
        if (ret < 0)
        {
            STK_LOG("read modify write i2c (0x%X) error\n", STK6BCX_REG_GPIO_SET1);
            return ret;
        }

        reg_data = STK6BCX_GPIO_MEASURE_EN;
        reg = STK6BCX_REG_GPIO_SET1;
        ret = sensor_write_reg_mask(client, reg, reg_data, STK6BCX_GPIO_MEASURE_EN);
        if (ret < 0)
        {
            STK_LOG("read modify write i2c (0x%X) error\n", STK6BCX_REG_GPIO_SET1);
            return ret;
        }
    }
#endif
    return ret;
}

static int stk6bcx_gpio_lost_handle(struct i2c_client *client, uint32_t measure_cnt)
{
    stk6bcx_gpio_config *cur_config = NULL;
    uint8_t  i2c_data[4] = {0};
    uint32_t measure_time = 0;
    uint8_t i, table_size = sizeof(gpio_config_table) / sizeof(gpio_config_table[0]);

    int ret = FAIL;
    cur_config = & gpio_config_table[0];//default 60Hz

    if (!measure_cnt)
    {
        i2c_data[0] = STK6BCX_REG_GPIO_SET7;
        ret = sensor_rx_data(client, i2c_data, 3);

        if (ret < 0)
        {
            STK_LOG("get data error! ret = %d\n", ret);
            return FAIL;
        }
        measure_time = ((i2c_data[0] << 16) | (i2c_data[1] << 8) | i2c_data[2]);
    }
    else
    {
        measure_time = measure_cnt;

        if ((measure_time / 1000) == stk6bcx_pdata.display_freq)
        {
            return ret;
        }
    }

//    STK_LOG(":: Measure time:%dus", measure_time);

    for (i = 0; i < table_size; i++)
    {
        if (gpio_config_table[i].target_timer / 1000 == measure_time / 1000)
        {
            cur_config = &gpio_config_table[i];
            break;
        }
    }

    if (!cur_config)
    {
        STK_LOG(":: need to implement");
        return NO_ERROR;
    }

    if (cur_config->screen_hz != stk6bcx_pdata.display_freq)
    {
        stk6bcx_pdata.display_freq = cur_config->screen_hz;
        STK_LOG(":: freq changed! Current is %dHz %dus", stk6bcx_pdata.display_freq, measure_time);
#ifdef STK6BCX_GPIO_PS
        STK_LOG(":: ps_td:%d, ps_duty:%d, ps_ignore:%d", (uint32_t)cur_config->ps_td, cur_config->ps_duty + 1, cur_config->ps_ignore);
#endif
        i2c_data[0] = STK6BCX_REG_GPIO_SET17;
        i2c_data[1] = (cur_config->target_timer >> 16) & 0x1F;
        i2c_data[2] = (cur_config->target_timer >>  8) & 0xFF;
        i2c_data[3] = (cur_config->target_timer >>  0) & 0xFF;
        ret = sensor_tx_data(client, i2c_data, 4);
        if (ret < 0)
        {
            STK_LOG("set timer failed!");
            return ret ;
        }

        i2c_data[0] = STK6BCX_REG_FSM_CTRL;
        i2c_data[1] = 0;
#ifdef STK6BCX_GPIO_PS
        if (stk6bcx_pdata.ps_enable && stk6bcx_pdata.gpio_enable)
        {
            i2c_data[1] |= STK6BCX_ALS_FSM_PAUSE_MASK;
        }
#endif
/*
#ifdef STK_GPIO_ALS
        if (stk6bcx_pdata.enable && stk6bcx_pdata.gpio_enable)
        {
            i2c_data[1] |= STK6BCX_PS_FSM_PAUSE_MASK;
        }
#endif
*/
        ret = sensor_write_reg_mask(client, i2c_data[0], i2c_data[1], i2c_data[1]);
//        ret = sensor_tx_data(client, i2c_data, 2);
        if (ret < 0)
        {
            STK_LOG("set FSM failed!");
            return ret ;
        }

        if (stk6bcx_pdata.gpio_enable)
        {
            stk6bcx_ps_gpio_enable(client, false);
            i2c_data[0] = STK6BCX_REG_GPIO_SET20;
            i2c_data[1] = (cur_config->ps_td >> 16) & 0x1F;
            i2c_data[2] = (cur_config->ps_td >>  8) & 0xFF;
            i2c_data[3] = (cur_config->ps_td >>  0) & 0xFF;
            ret = sensor_tx_data(client, i2c_data, 4);
            if (ret < 0)
            {
                STK_LOG("set td failed!");
                return ret ;
            }

            i2c_data[0] = STK6BCX_REG_GPIO_SET31;
            i2c_data[1] = cur_config->ps_duty;
            ret = sensor_tx_data(client, i2c_data, 2);
            if (ret < 0)
            {
                STK_LOG("set duty failed!");
                return ret ;
            }

            i2c_data[0] = STK6BCX_REG_GPIO_SET13;
            i2c_data[1] = ((STK6BCX_GPIO_IGNORE(cur_config->ps_ignore) >> 8) & 0xFF);
            i2c_data[2] = ((STK6BCX_GPIO_IGNORE(cur_config->ps_ignore) >> 0) & 0xFF);
            ret = sensor_tx_data(client, i2c_data, 3);
            if (ret < 0)
            {
                STK_LOG("set ignore failed!");
                return ret ;
            }

            stk6bcx_ps_gpio_enable(client, true);
        }

        i2c_data[0] = STK6BCX_REG_FSM_CTRL;
        i2c_data[1] = 0;
#ifdef STK6BCX_GPIO_PS
        if (stk6bcx_pdata.ps_enable && stk6bcx_pdata.gpio_enable)
        {
            i2c_data[1] |= STK6BCX_ALS_FSM_PAUSE_MASK;
        }
#endif
/*
#ifdef STK_GPIO_ALS
        if (stk6bcx_pdata.enable && stk6bcx_pdata.gpio_enable)
        {
            i2c_data[1] |= STK6BCX_PS_FSM_PAUSE_MASK;
        }

#endif
*/
        ret = sensor_write_reg_mask(client, i2c_data[0], 0, i2c_data[1]);
//        ret = sensor_tx_data(client, i2c_data, 2);
        if (ret < 0)
        {
            STK_LOG("set FSM failed!");
            return ret ;
        }

    }

    return ret;
}
#endif

static int stk6bcx_sensor_check_id(struct i2c_client *client)
{
    int ret = FAIL;
    uint8_t pid_count = 0;
    int reg_data = 0;
    reg_data = sensor_read_reg(client, STK6BCX_REG_PID);

    for (pid_count = 0; pid_count < (sizeof(stk6bcx_ps_pid_list) / sizeof(uint8_t)); pid_count++)
    {
        if ( reg_data == stk6bcx_ps_pid_list[pid_count])
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

static int stk6bcx_set_ps_thd(struct i2c_client *client, uint32_t threshold_high, uint32_t threshold_low)
{
    int ret = 0;
    uint8_t reg_data[5] = {0};
    reg_data[0] = STK6BCX_REG_PS_THDH1;
    reg_data[1] = (uint8_t)((threshold_high & 0xFF00) >> 8);
    reg_data[2] = (uint8_t)(threshold_high & 0x00FF);
    reg_data[3] = (uint8_t)((threshold_low & 0xFF00) >> 8);
    reg_data[4] = (uint8_t)(threshold_low & 0x00FF);
    STK_LOG("h:%d l:%d", threshold_high, threshold_low);
    ret = sensor_tx_data(client, reg_data, 5);

    if (ret < 0)
    {
        STK_LOG("stk6bcx_set_ps_thd failed!");
        return ret ;
    }

    return ret;
}

static int stk6bcx_sensor_hw_init(struct i2c_client *client)
{
    int i, ret = -1;

    for (i = 0; i < (sizeof(stk6bcx_default_register_table) / sizeof(stk6bcx_register_table)); i++)
    {
        ret = sensor_write_reg_mask(client, stk6bcx_default_register_table[i].address, stk6bcx_default_register_table[i].value, stk6bcx_default_register_table[i].mask);

        if (ret < 0)
        {
            STK_LOG("proximity init failed!\n");
            return ret;
        }
    }

    stk6bcx_set_ps_thd(client, STK6BCX_HT_N_CT, STK6BCX_LT_N_CT);
    STK_LOG("proximity init success!\n");
    return ret;
}

static int stk6bcx_ps_init(struct i2c_client *client)
{
    int ret = FAIL;
    uint8_t i2c_data[4];
    ret = stk6bcx_sensor_check_id(client);

    if (ret < NO_ERROR)
    {
        STK_LOG("proximity check id error ret = %d!\n", ret);
        return 0;
    }

    ret = stk6bcx_sensor_hw_init(client);

    if (ret < NO_ERROR)
    {
        STK_LOG("proximity hw init error ret = %d!\n", ret);
        return 0;
    }
#ifdef STK6BCX_GPIO_PS
    i2c_data[0] = STK6BCX_REG_GPIO_SET13;//ps ignore N*24,0x79
    i2c_data[1] = ((0x12C >> 8) & 0xFF);
    i2c_data[2] = ((0x12C >> 0) & 0xFF);
    ret = sensor_tx_data(client, i2c_data, 3);
    if (ret < 0)
    {
        STK_LOG("set ignore failed!");
        return ret ;
    }

    i2c_data[0] = STK6BCX_REG_GPIO_SET20; //ps td 0x81
    i2c_data[1] = (0x28 >> 16) & 0x1F;
    i2c_data[2] = (0x28 >>  8) & 0xFF;
    i2c_data[3] = (0x28 >>  0) & 0xFF;
    ret = sensor_tx_data(client, i2c_data, 4);
    if (ret < 0)
    {
        STK_LOG("set td failed!");
        return ret ;
    }
#endif
    stk6bcx_pdata.ht_n_ct           = STK6BCX_HT_N_CT;
    stk6bcx_pdata.lt_n_ct           = STK6BCX_LT_N_CT;
    stk6bcx_pdata.fac_ct            = STK6BCX_DEFAULT_CT;
    stk6bcx_pdata.psi_set           = 0xFFFF;
    stk6bcx_pdata.ps_stat_data[0]   = 0;
    stk6bcx_pdata.ps_stat_data[1]   = 0;
    stk6bcx_pdata.ps_stat_data[2]   = 0xFFFF;
    stk6bcx_pdata.data_count        = 0;
    stk6bcx_pdata.smudge_update     = 0;
    stk6bcx_pdata.psa               = 0x0;
    stk6bcx_pdata.psi               = 0xFFFF;
    stk6bcx_pdata.last_ps_psi       = 0xFFFF;
    stk6bcx_pdata.ps_enable         = false;
    if (stk6bcx_pdata.tracking_time == 0)
        stk6bcx_pdata.tracking_time = 1000000 / 1000; //ms
    stk6bcx_pdata.compensation_target = STK6BCX_TC_TRACKING_TIME / stk6bcx_pdata.tracking_time;
    return 0;
}
/*
static int stk6bcx_sensor_set_cali_data(void *cali_data)
{
    if (cali_data == NULL)
        return NO_ERROR;

    return proximity_sensor_set_cali_data(&stk6bcx_prox_cali_params, cali_data);
}

static int stk6bcx_sensor_rate(int32_t sampling_period_us)
{
    (void)sampling_period_us;
    return NO_ERROR;
}
*/
static int stk6bcx_ps_enable(struct i2c_client *client, int enable, int rate)
{
    int ret = FAIL;
    uint8_t buf[2] = {STK6BCX_REG_ENABLE, 0x00};
    buf[1] = sensor_read_reg(client, buf[0]);
    buf[1] |= (STK6BCX_STATE_EN_PS_MASK | STK6BCX_STATE_EN_PS_WAIT_MASK);

    if(enable){
        if (enable_state != 1)
        {
            ret = stk6bcx_set_ps_thd(client, stk6bcx_pdata.ps_thd_h + STK6BCX_DEFAULT_CT, stk6bcx_pdata.ps_thd_l + STK6BCX_DEFAULT_CT);
#ifdef STK6BCX_GPIO_PS
            stk6bcx_ps_gpio_enable(client, true);
#endif
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
        }
        else
        {
            ret = NO_ERROR;
            STK_LOG("already enabled!\n");
        }
        stk6bcx_pdata.ps_enable = true;
    }else{
        buf[1] &= (uint8_t)(~(STK6BCX_STATE_EN_PS_MASK | STK6BCX_STATE_EN_PS_WAIT_MASK));

        if (enable_state != 0)
        {
        /*
            stk6bcx_prox_filter.status_backup = 5.0f;
            stk6bcx_prox_filter.close_flag = 0;
            stk6bcx_prox_filter.far_away_flag = 0;
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
            stk6bcx_pdata.ps_enable = false;
#ifdef STK6BCX_GPIO_PS
            stk6bcx_ps_gpio_enable(client, false);
#endif
        }
        else
        {
            ret = NO_ERROR;
            STK_LOG("already disabled!");
        }

        last_proximit_status = -1;
        stk6bcx_pdata.ps_stat_data[0]   = 0;
        stk6bcx_pdata.ps_stat_data[1]   = 0;
        stk6bcx_pdata.ps_stat_data[2]   = 0xFFFF;
        stk6bcx_pdata.data_count        = 0;
        stk6bcx_pdata.smudge_update     = 0;
        stk6bcx_pdata.psa               = 0x0;

        if (stk6bcx_pdata.psi_set != 0xFFFF)
        {
            stk6bcx_pdata.last_ps_psi   = stk6bcx_pdata.psi;
        }
    }
    stk6bcx_pdata.ps_last_status    = -1;
    stk6bcx_pdata.ps_need_report = false;

    return ret;
}
/*
static int stk6bcx_sensor_disable()
{
    int ret = FAIL;
    uint8_t buf[2] = {STK6BCX_REG_ENABLE, 0x00};
    sensor_read_reg(client, &buf[0], &buf[1], 1, 0xFF);
    buf[1] &= (uint8_t)(~(STK6BCX_STATE_EN_PS_MASK | STK6BCX_STATE_EN_PS_WAIT_MASK));

    if (enable_state != 0)
    {
        stk6bcx_prox_filter.status_backup = 5.0f;
        stk6bcx_prox_filter.close_flag = 0;
        stk6bcx_prox_filter.far_away_flag = 0;
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
#ifdef STK6BCX_GPIO_PS
        stk6bcx_ps_gpio_enable(false);
#endif
    }
    else
    {
        ret = NO_ERROR;
        STK_LOG("already disabled!");
    }

    last_proximit_status = -1;
    stk6bcx_pdata.ps_last_status    = -1;
    stk6bcx_pdata.ps_stat_data[0]   = 0;
    stk6bcx_pdata.ps_stat_data[1]   = 0;
    stk6bcx_pdata.ps_stat_data[2]   = 0xFFFF;
    stk6bcx_pdata.data_count        = 0;
    stk6bcx_pdata.smudge_update     = 0;
    stk6bcx_pdata.psa               = 0x0;

    if (stk6bcx_pdata.psi_set != 0xFFFF)
    {
        stk6bcx_pdata.last_ps_psi   = stk6bcx_pdata.psi;
    }

    return ret;
}

static int stk6bcx_sensor_set_status()
{
    return NO_ERROR;
}


static int stk6bcx_sensor_get_status(struct i2c_client *client)
{
    uint8_t reg_addr = STK6BCX_REG_PS_FLAG1;
    uint8_t reg_data = STK6BCX_FLG_PSDR_MASK;
    return sensor_i2c_check_reg_data(client, &reg_addr, &reg_data, STK6BCX_FLG_PSDR_MASK);
}

static void stk6bcx_sensor_cali_cmd_handle(int cal_cmd, int cali_type, int golden_sample)
{
    proximity_sensor_cali_cmd_handle(cal_cmd, cali_type, golden_sample);
}


static void stk6bcx_sensor_get_cali_data()
{
    proximity_sensor_get_cali_data();
}
*/

static int stk6bcx_prx_val(struct i2c_client *client)
{
    int ret = NO_ERROR;
    uint8_t reg_addr, reg_value[2] = {0};
    uint8_t ps_invalid_flag;
    uint8_t bgir_out_of_range = 0;
    uint16_t bgir_raw;
    reg_addr = STK6BCX_REG_PS_FLAG1;
    reg_value[0] = sensor_read_reg(client, STK6BCX_REG_PS_FLAG1);

    ret = reg_value[0];
    if (ret < NO_ERROR)
    {
        STK_LOG("get PS invald flag error! ret = %d\n", ret);
        return FAIL;
    }

    reg_value[0] = STK6BCX_REG_PS_BGIR_DATA1;
    ret = sensor_rx_data(client, reg_value, 2);

    if (ret < NO_ERROR)
    {
        STK_LOG("get PS BGIR error! ret = %d\n", ret);
        return FAIL;
    }

    bgir_raw = (uint16_t)(reg_value[0] << 8 | reg_value[1]);

    if (bgir_raw > STK6BCX_PS_BGIR_THRESHOLD)
    {
        bgir_out_of_range = true;
        STK_LOG("BGIR invalid, BGIR: %d", bgir_raw);
    }

    if ((ps_invalid_flag & STK6BCX_FLG_PS_INVALID_MASK) || bgir_out_of_range)
    {
        ret = FAIL;
    }

    return ret;
}

static void stk6bcx_ps_smudge_judgement(uint16_t ps_raw_data)
{
    uint16_t raw_data = 0.0;

    if (ps_raw_data > stk6bcx_pdata.psi)
    {
        raw_data = ps_raw_data - stk6bcx_pdata.psi;
    }

    if ((raw_data > STK6BCX_SMUDGE_DIFF) && (stk6bcx_pdata.smudge_update == 0) && (stk6bcx_pdata.psi != 0xFFFF))
    {
        stk6bcx_pdata.ps_thd_h          = (uint16_t)(stk6bcx_pdata.psi + stk6bcx_pdata.ht_n_ct * STK6BCX_PS_SMUDGE_RATIO);
        stk6bcx_pdata.ps_thd_l          = (uint16_t)(stk6bcx_pdata.psi + stk6bcx_pdata.lt_n_ct * STK6BCX_PS_SMUDGE_RATIO);
        stk6bcx_pdata.smudge_update     = 1;
        stk6bcx_pdata.set_thd           = 1;
    }
}

static void stk6bcx_ps_prx_dynamicK_reset(void)
{
    stk6bcx_pdata.ps_stat_data[0] = 0;
    stk6bcx_pdata.ps_stat_data[1] = 0;
    stk6bcx_pdata.ps_stat_data[2] = 9999;
    stk6bcx_pdata.data_count = 0;
}

static void stk6bcx_ps_prx_dynamicK_threshold_reset(uint16_t ps_data)
{
    if (ps_data > stk6bcx_pdata.ps_stat_data[0])
        stk6bcx_pdata.ps_stat_data[0] = ps_data;

    if (ps_data < stk6bcx_pdata.ps_stat_data[2])
        stk6bcx_pdata.ps_stat_data[2] = ps_data;
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
    stk6bcx_pdata.ps_data_tc_filter.max = max;
    stk6bcx_pdata.ps_data_tc_filter.min = min;
}

static void stk_prx_dynamicK_compensation(uint16_t word_data,
                                   int32_t ps_status,
                                   uint16_t *ps_thd_h,
                                   uint16_t *ps_thd_l)
{
    uint16_t temp_idx = 0, ct_value = 0;
    uint16_t i = 0;
    uint32_t ps_upstat_data[3] = {0};

    if ((stk6bcx_pdata.compensation_target != 0) && (++stk6bcx_pdata.compensation_cnt < stk6bcx_pdata.compensation_target))
        return;

    stk6bcx_pdata.compensation_cnt = 0;

    if (stk6bcx_pdata.ps_data_tc_filter.idx < STK6BCX_CT_FIR_LEN)
        stk6bcx_pdata.ps_data_tc_filter.raw[stk6bcx_pdata.ps_data_tc_filter.idx % STK6BCX_CT_FIR_LEN] = word_data;
    else
    {
        memmove(&stk6bcx_pdata.ps_data_tc_filter.raw[0],
                &stk6bcx_pdata.ps_data_tc_filter.raw[1],
                sizeof(stk6bcx_pdata.ps_data_tc_filter.raw) - sizeof(stk6bcx_pdata.ps_data_tc_filter.raw[0]));
        stk6bcx_pdata.ps_data_tc_filter.raw[STK6BCX_CT_FIR_LEN - 1] = word_data;
        STK_LOG(": raw[0] = %d, raw[1] = %d, raw[2] = %d, raw[3] = %d, raw[4] = %d, raw[5] = %d\n",
                stk6bcx_pdata.ps_data_tc_filter.raw[0],
                stk6bcx_pdata.ps_data_tc_filter.raw[1],
                stk6bcx_pdata.ps_data_tc_filter.raw[2],
                stk6bcx_pdata.ps_data_tc_filter.raw[3],
                stk6bcx_pdata.ps_data_tc_filter.raw[4],
                stk6bcx_pdata.ps_data_tc_filter.raw[5]);
    }

    stk6bcx_pdata.ps_data_tc_filter.idx++;
    if (stk6bcx_pdata.ps_data_tc_filter.idx < STK6BCX_CT_FIR_LEN)
        return;

    stk_prx_sorting_max_min(stk6bcx_pdata.ps_data_tc_filter.raw, STK6BCX_CT_FIR_LEN);
    STK_LOG(":(%d) MAX = %d, MIN = %d\n", stk6bcx_pdata.ps_data_tc_filter.idx, stk6bcx_pdata.ps_data_tc_filter.max, stk6bcx_pdata.ps_data_tc_filter.min);
    if (stk6bcx_pdata.ps_data_tc_filter.idx == (10 * STK6BCX_CT_FIR_LEN))
        stk6bcx_pdata.ps_data_tc_filter.idx = STK6BCX_CT_FIR_LEN;

    if ((stk6bcx_pdata.ps_data_tc_filter.max - stk6bcx_pdata.ps_data_tc_filter.min) < STK6BCX_TC_MAX_MIN_DIFF)
    {
        temp_idx = STK6BCX_CT_FIR_LEN / 2;
        for (i = 0; i < temp_idx; i++)
        {
            ps_upstat_data[0] += stk6bcx_pdata.ps_data_tc_filter.raw[i];
        }
        ps_upstat_data[0] /= temp_idx;

        for (i = temp_idx; i < STK6BCX_CT_FIR_LEN; i++)
        {
            ps_upstat_data[1] += stk6bcx_pdata.ps_data_tc_filter.raw[i];
        }
        ps_upstat_data[1] /= temp_idx;
        STK_LOG(": ps_upstat_data[0] = %d, ps_upstat_data[1] = %d\n", ps_upstat_data[0], ps_upstat_data[1]);
    }

    ps_upstat_data[2] = ps_upstat_data[1] - ps_upstat_data[0];
    if ((ps_upstat_data[1] > ps_upstat_data[0]) && (ps_upstat_data[2] < STK6BCX_TC_SLOPE_THD))
    {
        if (ps_status == PROX_STATE_NEAR)
        {
            *ps_thd_h += ps_upstat_data[2];
            *ps_thd_l += ps_upstat_data[2];
            stk6bcx_pdata.psi = *ps_thd_h - stk6bcx_pdata.ht_n_ct;
            stk6bcx_pdata.set_thd = true;
        }
        else
        {
            ct_value = *ps_thd_h - stk6bcx_pdata.ht_n_ct;
            STK_LOG(": ct_value = %d, ht_n_ct: %d, ps_upstat_data[1] = %d\n", ct_value, stk6bcx_pdata.ht_n_ct, ps_upstat_data[1]);
            if ((ps_upstat_data[1] > ct_value) && ((ps_upstat_data[1] - ct_value) > STK6BCX_TC_CT_DIFF))
            {
                *ps_thd_h = ps_upstat_data[1] + stk6bcx_pdata.ht_n_ct;
                *ps_thd_l = ps_upstat_data[1] + stk6bcx_pdata.lt_n_ct;
                stk6bcx_pdata.psi = ps_upstat_data[1];
                stk6bcx_pdata.set_thd = true;
            }
        }
    }
}

static void stk6bcx_ps_prx_dynamicK_tracking_recali(uint16_t ps_data)
{
    uint16_t ct_value;

    if (stk6bcx_pdata.ps_last_status == PROX_STATE_FAR)
    {
        stk6bcx_pdata.ps_stat_data[1] += ps_data;
        stk6bcx_ps_prx_dynamicK_threshold_reset(ps_data);
        stk6bcx_pdata.data_count ++;

        if (stk6bcx_pdata.data_count == STK6BCX_TRACKING_QUANTI)
        {
            stk6bcx_pdata.ps_stat_data[1] /= stk6bcx_pdata.data_count;
            ct_value = stk6bcx_pdata.ps_thd_h - stk6bcx_pdata.ht_n_ct;

            //STK_LOG(":ct_value = %d, ps_data = %d\n", ct_value, ps_data);
            if ((stk6bcx_pdata.ps_stat_data[1] < ct_value) &&
                ((ct_value - stk6bcx_pdata.ps_stat_data[1]) >= 5) &&
                ((stk6bcx_pdata.ps_stat_data[0] - stk6bcx_pdata.ps_stat_data[2]) <= STK6BCX_QUANTI_RANGE))
            {
                STK_LOG("ps variation = %d", stk6bcx_pdata.ps_stat_data[0] - stk6bcx_pdata.ps_stat_data[2]);
                stk6bcx_pdata.ps_thd_h          = (uint16_t)(stk6bcx_pdata.ps_stat_data[1] + stk6bcx_pdata.ht_n_ct);
                stk6bcx_pdata.ps_thd_l          = (uint16_t)(stk6bcx_pdata.ps_stat_data[1] + stk6bcx_pdata.lt_n_ct);
                stk6bcx_pdata.psi               = (uint16_t)(stk6bcx_pdata.ps_stat_data[1]);
                stk6bcx_pdata.smudge_update     = 0;
                stk6bcx_pdata.set_thd           = 1;
            }

            stk6bcx_ps_prx_dynamicK_reset();
        }
    }
    else
    {
        stk6bcx_ps_prx_dynamicK_reset();
        stk6bcx_ps_smudge_judgement(ps_data);
    }

    if (stk6bcx_pdata.compensation_target != 0)
        stk_prx_dynamicK_compensation(ps_data, stk6bcx_pdata.ps_last_status, &stk6bcx_pdata.ps_thd_h, &stk6bcx_pdata.ps_thd_l);
}

static void stk6bcx_ps_prx_dynamicK_tracking_max_min(uint16_t ps_data)
{
    if (ps_data > stk6bcx_pdata.psa)
    {
        stk6bcx_pdata.psa = ps_data;
    }

    if (ps_data < stk6bcx_pdata.psi)
    {
        stk6bcx_pdata.psi = ps_data;
    }
}

static void stk6bcx_ps_prx_dynamicK_tracking_task(struct i2c_client *client, uint16_t ps_data)
{
    int ret = NO_ERROR;
    uint16_t diff, ps_invalid = 1;
    ret = stk6bcx_prx_val(client);

    if (ret == NO_ERROR)
        ps_invalid = 0;

    if ((ps_invalid == 1) || (ps_data == 0))
        return;

    stk6bcx_pdata.set_thd = 0;

    if (stk6bcx_pdata.psi_set != 0xFFFF)
    {
        stk6bcx_ps_prx_dynamicK_tracking_recali(ps_data);
    }
    else
    {
        stk6bcx_ps_prx_dynamicK_tracking_max_min(ps_data);
        diff = (stk6bcx_pdata.psa - stk6bcx_pdata.psi);

        if (diff > STK6BCX_MAX_MIN_DIFF)
        {
            stk6bcx_pdata.psi_set = stk6bcx_pdata.psi;
            stk6bcx_pdata.ps_thd_h = stk6bcx_pdata.psi + stk6bcx_pdata.ht_n_ct;
            stk6bcx_pdata.ps_thd_l = stk6bcx_pdata.psi + stk6bcx_pdata.lt_n_ct;
            stk6bcx_pdata.set_thd = 1;
        }
        stk6bcx_pdata.compensation_cnt = 0;
        memset(&(stk6bcx_pdata.ps_data_tc_filter), 0, sizeof(stk6bcx_pdata.ps_data_tc_filter));
    }

    if (stk6bcx_pdata.set_thd)
    {
        ret = stk6bcx_set_ps_thd(client, stk6bcx_pdata.ps_thd_h, stk6bcx_pdata.ps_thd_l);

        if (ret != NO_ERROR)
            return;

        stk6bcx_pdata.set_thd = 0;
    }
}

static int stk6bcx_sensor_get_raw_data(struct i2c_client *client, uint16_t *ps_data)
{
    int ret = FAIL;
    uint8_t reg_value[2];

    reg_value[0] = STK6BCX_REG_PS_DATA1;
    ret = sensor_rx_data(client, reg_value, 2);

    if (ret < 0)
    {
        STK_LOG("get data error! ret = %d\n", ret);
        return FAIL;
    }

    *ps_data = (reg_value[0] << 8) | (reg_value[1]);
    return NO_ERROR;
}

static int stk6bcx_ps_distance_last(struct i2c_client *client, int ps)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	

    if (stk6bcx_pdata.ps_need_report) {
        STK_LOG("%s:ps=%d,NF=%d",__func__, ps, stk6bcx_pdata.ps_last_status);
    	input_report_abs(sensor->input_dev, ABS_DISTANCE, stk6bcx_pdata.ps_last_status);
    	input_sync(sensor->input_dev);
    }
	return 0;
}


static int stk6bcx_ps_get_data(struct i2c_client *client)
{
    int ret = NO_ERROR, flag1 = 0, flag2 = 0;
    uint16_t ps_data;
    stk6bcx_pdata.ps_need_report = false;

#ifdef STK6BCX_GPIO_PS
    if (stk6bcx_pdata.gpio_enable)
    {
        uint8_t  i2c_flag_reg[3] = {0};

        i2c_flag_reg[0] = STK6BCX_REG_GPIO_SET7;
        ret = sensor_rx_data(client, i2c_flag_reg, 3);

        if (ret < 0)
        {
            STK_LOG("get data error! ret = %d", ret);
            return FAIL;
        }

        stk6bcx_gpio_lost_handle(client, (((i2c_flag_reg[0] & 0x1F) << 16) | (i2c_flag_reg[1] << 8) | i2c_flag_reg[2]));
    }
#endif

    if((++stk6bcx_pdata.ps_dbg_cnt == STK6BCX_PS_DUMP_CNT) ) {
        stk6bcx_dump_reg(client);
        stk6bcx_pdata.ps_dbg_cnt = 0;
    }

    flag1 = sensor_read_reg(client, STK6BCX_REG_PS_FLAG1);

    if (flag1 < 0)
    {
        STK_LOG("Read STK6BCX_REG_PS_FLAG1 fail! ret = %d", flag1);
        return FAIL;
    }
//    STK_LOG(" get flag1：0x%x", flag1);

    if (flag1 & STK6BCX_FLG_PS_DR_MASK)
    {
        flag2 = sensor_read_reg(client, STK6BCX_REG_PS_FLAG2);
        if (flag2 < 0)
        {
            STK_LOG("Read STK6BCX_REG_PS_FLAG2 fail! ret = %d", ret);
            return FAIL;
        }

        ret = stk6bcx_sensor_get_raw_data(client, &ps_data);
        if (ret < 0)
        {
            STK_LOG("proximity get data fail! ret = %d", ret);
            return FAIL;
        }

        STK_LOG(" get data：raw = %d 0x%x 0x%x %d", ps_data, flag1, flag2, stk6bcx_pdata.ps_last_status);
        stk6bcx_ps_prx_dynamicK_tracking_task(client, ps_data);
        if(flag2 & STK6BCX_FLG_PS_NF_MASK){
            if(stk6bcx_pdata.ps_last_status != PS_FAR)
                stk6bcx_pdata.ps_need_report = true;
            stk6bcx_pdata.ps_last_status  = PS_FAR;

        }else if((flag2 & STK6BCX_FLG_PS_NF_MASK) == 0){
            if(stk6bcx_pdata.ps_last_status != PS_NEAR)
                stk6bcx_pdata.ps_need_report = true;
            stk6bcx_pdata.ps_last_status  = PS_NEAR;
        }else{
            STK_LOG("proximity unknown status\n");
        }

        stk6bcx_ps_distance_last(client, ps_data);
    }

    return ret;
}

struct sensor_operate proximity_stk6bcx_ops = {
	.name				= "ps_stk6bcx",
	.type				= SENSOR_TYPE_PROXIMITY,	//sensor type and it should be correct
	.id_i2c				= PROXIMITY_ID_STK6BCX,		//i2c id number
	.read_reg			= STK6BCX_REG_PS_DATA1,			//read data
//	.addr               =0x47,
	.read_len			= 2,				        //data length
	.id_reg				= STK6BCX_REG_PID,//SENSOR_UNKNOW_DATA,		//read device id from this register
	.id_data 			= 0x12,//SENSOR_UNKNOW_DATA,		//device id
	.precision			= 16,				         //16 bits
	.ctrl_reg 			= STK6BCX_REG_PS_FLAG1,			//enable or disable 
	.int_status_reg 	= 0x00,			//intterupt status register
	.range				= {0,1},			//range
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,		
	.active				= stk6bcx_ps_enable,
	.init				= stk6bcx_ps_init,
	.report				= stk6bcx_ps_get_data,
	// int 	brightness[2];//backlight min_brightness max_brightness 
	// int int_ctrl_reg;
	// int (*suspend)(struct i2c_client *client);
	// int (*resume)(struct i2c_client *client);
	// struct miscdevice *misc_dev;	
};

static struct sensor_operate *proximity_get_ops(void)
{
	return &proximity_stk6bcx_ops;
}

static int __init stk6bcx_init(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, proximity_get_ops);
	return result;
}

static void __exit stk6bcx_exit(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, proximity_get_ops);
}


module_init(stk6bcx_init);
module_exit(stk6bcx_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk6bcx Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

