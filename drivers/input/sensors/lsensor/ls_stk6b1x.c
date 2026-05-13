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

#include "ls_stk6b1x.h"
#include "stk6b1x_light_ver.h"

uint8_t stk6b1x_pid_list[] = {0xA1, 0xA4, 0xA5, 0xA6, 0xA8, 0xA9};
#ifdef STK6B1X_RGB_ENABLE
static float lux_coef_h[RAW_NUM] = {0.0, -0.004,  0.0, 0.1331};
static float lux_coef_l[RAW_NUM] = {0.0,  0.0894, 0.0, 2.2};
//#ifdef STK6B1X_RGB_ENABLE
stk6b1x_cluster_mean stk6b1x_cluster_mean_table[] =
{
    {0.5005, 1.0, 0.4835, 0.1555},
    {0.6663, 1.0, 0.3383, 1.927},
};

stk6b1x_cct_cluster stk6b1x_cct_cluster_table[] =
{
    {
        0.015668492,    -0.003182499,       0.012251103,        0.013784706,
        0.005818664,     0.003380047,       0.010842902,        0.018424187,
        0.002185413,    -0.002243747,       0.042691319,        0.008543594
    },
    {
        -0.01027816,      0.015437748,       0.000469412,        0.000401601,
            -0.01593433,      0.019368151,       0.000945301,        0.00033405,
            0.022134192,     0.017235924,       0.025161818,       -0.000155525
        },
};
#endif
/*
static int stk6b1x_sensor_check_id();
static int stk6b1x_sensor_hw_init();
static int stk6b1x_sensor_init(struct i2c_client *client);
static int stk6b1x_sensor_set_cali_data(void *cali_data);
static int stk6b1x_sensor_rate(int32_t sampling_period_us);
static int stk6b1x_sensor_enable();
static int stk6b1x_sensor_disable();
static int stk6b1x_sensor_set_status();
static int stk6b1x_sensor_get_status();
static void stk6b1x_sensor_cali_cmd_handle(int cal_cmd, int cali_type, int golden_sample);
static void stk6b1x_sensor_get_cali_data();
static int stk6b1x_sensor_get_data(struct i2c_client *client);
static int stk6b1x_sensor_set_mode(int mode);
static int stk6b1x_sensor_get_fifo_data(struct sensor_data *sensor_data);
static int stk6b1x_sensor_flush(int sensor);
static int stk6b1x_sensor_selftest();

sensor_info_t stk6b1x_light_info =
{
    .name = "stk6b1x",
    .vendor = "sensortek",
    .version = 0,
    .sensor_type = SENSOR_TYPE_LIGHT,
    .maxrange = 65535,
    .resolution = 1.0f,
    .power = 0.15f,
    .mindelay_us = 0,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .maxdelay_us = 0,
};

struct sensor_hw_info stk6b1x_light_i2c_info =
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
    .sensor_info = &stk6b1x_light_info,
    .sensor_data = {0},
    .report_mode = POLLING_MODE,
    .sensor_driver_handle = DRV_LIGHT,
    .enabled = 0, //int enabled;
    .sampling_period_us = 20000, //20ms -> polling timer ,default 320000us;
    .sampling_timer = 40000,
    .max_report_latency_us = 0,
    .sensor_support_status = 0,

    .init = stk6b1x_sensor_init,
    .check_id = stk6b1x_sensor_check_id,
    .hw_init = stk6b1x_sensor_hw_init,
    .set_cali_data = stk6b1x_sensor_set_cali_data,
    .set_rate = stk6b1x_sensor_rate,
    .activate = stk6b1x_sensor_enable,
    .deactivate = stk6b1x_sensor_disable,
    .set_status = stk6b1x_sensor_set_status,
    .get_status = stk6b1x_sensor_get_status,
    .get_data = stk6b1x_sensor_get_data,
    .set_mode = stk6b1x_sensor_set_mode,
    .get_fifo_data = stk6b1x_sensor_get_fifo_data,
    .self_test = stk6b1x_sensor_selftest,
    .flush = stk6b1x_sensor_flush,
    .cali_cmd = stk6b1x_sensor_cali_cmd_handle,
    .get_cali_data = stk6b1x_sensor_get_cali_data,
};
*/
static float last_luxdata = -1.0f;
static uint8_t enable_state;
static uint8_t reject_frames_num;

#if STK_ALGO_ENABLE
static stk6b1x_als_fac_cali_data als_cali_fgcc_data;
static struct stk6b1x_als_fac_cali_data_type als_cali_fgcc;
#endif

struct stk_data stk6b1x_als =
{
    0,     /* als count */
    0,     /* als enable */
    1.0,   /*als_scale*/
};

stk6b1x_register_table stk6b1x_als_default_register_table[] =
{
#ifndef STK_ALS_IT1_SHORT
    {STK6B1X_REG_ALS_IT_SET0,       STK6B1X_ALS_IT_50,                                              STK6B1X_ALS_IT_MASK},
#else
    {STK6B1X_REG_ALS_IT_SET0,       STK6B1X_ALS_IT_SEL_IT1_SHORT,                                   STK6B1X_ALS_IT_SEL_MASK},
    {STK6B1X_REG_ALS_IT_SET1,       STK6B1X_ALS_IT_SHORT_672,                                       STK6B1X_ALS_IT_SHORT_MASK},
#endif
#ifdef STK_ALS_HAGC
#ifndef STK_ALS_IT1_SHORT
    {STK6B1X_REG_ALS_AGC_SET6,    0x08,                                                          0x08},
#else
    {STK6B1X_REG_ALS_AGC_SET6,    0x0C,                                                          0x0F},
    {0x61,                        0x01,                                                          0xFF},
    {
        STK6B1X_REG_ALS_AGC_SET4,
        STK6B1X_ALS0_AGC1 | STK6B1X_ALS1_AGC1 | STK6B1X_ALS2_AGC1 | STK6B1X_ALS3_AGC1,
        0xFF
    },
    {
        STK6B1X_REG_ALS_AGC_SET5,
        STK6B1X_ALS4_AGC1,
        0xFF
    },
#endif
    {
        STK6B1X_REG_FIFO_SET0,
        // (STK6B1X_FIFO_SEL_SALS0_SALS1_SALS2_SALS3_SALS4 << STK6B1X_FIFO_SEL_SHIFT),
        // STK6B1X_FIFO_SEL_MASK
        (STK6B1X_FIFO_SEL_SALS0_SALS1_SALS2_SALS3_SALS4 << STK6B1X_FIFO_SEL_SHIFT) | 0x04, // for i2c < 16byte
        STK6B1X_FIFO_SEL_MASK | 0x04 // for i2c < 16byte
    },
#else
    {
        STK6B1X_REG_FIFO_SET0,
        // (STK6B1X_FIFO_SEL_ALS01234 << STK6B1X_FIFO_SEL_SHIFT),
        // STK6B1X_FIFO_SEL_MASK
        (STK6B1X_FIFO_SEL_ALS01234 << STK6B1X_FIFO_SEL_SHIFT) | 0x04, // for i2c < 16byte
        STK6B1X_FIFO_SEL_MASK | 0x04 // for i2c < 16byte
    },
#endif
    {
        STK6B1X_REG_ALS_DGAIN,
        STK6B1X_ALS_DGAIN2048 << STK6B1X_ALS0_DGAIN_SHIFT | STK6B1X_ALS_DGAIN2048 << STK6B1X_ALS1_DGAIN_SHIFT,
                              STK6B1X_ALS0_DGAIN_MASK | STK6B1X_ALS1_DGAIN_MASK
    },
    {
        STK6B1X_REG_ALS_DGAIN1,
        STK6B1X_ALS_DGAIN2048 << STK6B1X_ALS2_DGAIN_SHIFT | STK6B1X_ALS_DGAIN2048 << STK6B1X_ALS3_DGAIN_SHIFT,
                              STK6B1X_ALS2_DGAIN_MASK | STK6B1X_ALS3_DGAIN_MASK
    },
    {STK6B1X_REG_ALS_DGAIN2,        STK6B1X_ALS_DGAIN2048 << STK6B1X_ALS4_DGAIN_SHIFT,              STK6B1X_ALS4_DGAIN_MASK},
    {
        STK6B1X_REG_ALS_AGAIN,
        STK6B1X_ALS_AGAIN2_0 << STK6B1X_ALS0_AGAIN_SHIFT | STK6B1X_ALS_AGAIN2_0 << STK6B1X_ALS1_AGAIN_SHIFT,
                             STK6B1X_ALS0_AGAIN_MASK | STK6B1X_ALS1_AGAIN_MASK
    },
    {
        STK6B1X_REG_ALS_AGAIN1,
        STK6B1X_ALS_AGAIN2_0 << STK6B1X_ALS2_AGAIN_SHIFT | STK6B1X_ALS_AGAIN2_0 << STK6B1X_ALS3_AGAIN_SHIFT,
                             STK6B1X_ALS2_AGAIN_MASK | STK6B1X_ALS3_AGAIN_MASK
    },
    {STK6B1X_REG_ALS_AGAIN2,        STK6B1X_ALS_AGAIN2_0 << STK6B1X_ALS4_AGAIN_SHIFT,               STK6B1X_ALS4_AGAIN_MASK},
#ifdef STK_GPIO_ALS
    {STK6B1X_REG_GPIO_SET1,         STK6B1X_GPIO_EN_MEASURE_MASK,                                   STK6B1X_GPIO_EN_MEASURE_MASK},
    {STK6B1X_REG_GPIO_SET2,         0x40,                                                           0x40},
    {STK6B1X_REG_GPIO_SET23,        STK6B1X_GPIO_FREQ_LOST_THD_5_PERCENT,                           0x1F},

    {STK6B1X_REG_GPIO_SET24,        STK6B1X_H_BYTE(STK6B1X_GPIO_TIMER_20MS),                        0x3F},
    {STK6B1X_REG_GPIO_SET25,        STK6B1X_L_BYTE(STK6B1X_GPIO_TIMER_20MS),                        0xFF},

    {STK6B1X_REG_GPIO_SET20,        0x00,                                                           0x1F},
    {STK6B1X_REG_GPIO_SET21,        0x00,                                                           0xFF},
    {STK6B1X_REG_GPIO_SET22,        0xA8,                                                           0xFF},
#endif
};

#if STK_ALGO_ENABLE
/*The function use to get display RGB value and brightness sample, must implemented by customer*/
int get_underscreen_als_data(void *pata, int len)
{
    struct under_screen_als_info als_info;
    memcpy(&als_info, pata, sizeof(struct under_screen_als_info));

    if (als_info.send_flag == 1) // update RGB,brightness
    {
        stk6b1x_als.pixeldata.PixelR = (als_info.display_info_t.pixel_rgb & 0xFF0000) >> 16; //Current display R value
        stk6b1x_als.pixeldata.PixelG = (als_info.display_info_t.pixel_rgb & 0x00FF00) >> 8;  //Current display G value
        stk6b1x_als.pixeldata.PixelB = (als_info.display_info_t.pixel_rgb & 0x0000FF);       //Current display B value
        stk6b1x_als.pixeldata.Brightness = als_info.display_info_t.brightness;; //Current display brightness
        STK_LOG("[lib_use]brightness= %d, pixel_r =%d ,g = %d, b = %d", stk6b1x_als.pixeldata.Brightness,
                        stk6b1x_als.pixeldata.PixelR, stk6b1x_als.pixeldata.PixelG, stk6b1x_als.pixeldata.PixelB);
    }
}

// ID 0  1000lux ALS value
// ID 1, R255 G0   B0    L255
// ID 2, R0   G255 B0    L255
// ID 3, R0   G0   B255  L255
// ID 4, R64  G0   B0    L255
// ID 5, R0   G64  B0    L255
// ID 6, R0   G0   B64   L255

// ID 7,  R255 G0   B0   L200
// ID 8, R0   G255 B0    L200
// ID 9, R0   G0   B255  L200

AlgoParam CaliData;

static void stk_algo_init(void)
{
    FloatParameter TempGamma;
    uint32_t AlgoVer;
    uint32_t AlgoDate;
    stk6b1x_als.calibrated = 0;

    for (uint8_t i = 0; i < MAX_BACKGROUND_NUM - 1; i++)
    {
        stk6b1x_als.ChannelData[i].ChannelF = 0;
        stk6b1x_als.ChannelData[i].ChannelG = als_cali_fgcc_data.fac_cali_G_data[i] \
                                            * als_cali_fgcc_data.fac_cali_other_data[i];//Each picture value;
        STK_LOG("stk6b1x_als.ChannelData[%d].ChannelG=%d\n", i, stk6b1x_als.ChannelData[i].ChannelG);
    }

    if ( als_cali_fgcc_data.fac_cali_G_data[0] != 0 && als_cali_fgcc_data.fac_cali_G_data[1] != 0 &&
         als_cali_fgcc_data.fac_cali_G_data[2] != 0 && als_cali_fgcc_data.fac_cali_G_data[3] != 0 &&
         als_cali_fgcc_data.fac_cali_G_data[4] != 0 && als_cali_fgcc_data.fac_cali_G_data[5] != 0 &&
         als_cali_fgcc_data.fac_cali_G_data[6] != 0 && als_cali_fgcc_data.fac_cali_G_data[7] != 0 &&
         als_cali_fgcc_data.fac_cali_G_data[8] != 0 && als_cali_fgcc_data.fac_cali_G_data[9] != 0 )
    {
        stk6b1x_als.calibrated = 1;
        stk6b1x_als.als_scale = (float)(TARGET_LUX / (als_cali_fgcc_data.fac_cali_G_data[0]));
        memset(&CaliData, 0x0, sizeof(AlgoParam));
        // Calibration for Gamma Point
        CaliData.RGBGamma[0].Level = 64;
        STK_calcRGBGamma(64, &stk6b1x_als.ChannelData[4], 255, &stk6b1x_als.ChannelData[1], &TempGamma);
        CaliData.RGBGamma[0].R.ParameterF = TempGamma.ParameterF;
        CaliData.RGBGamma[0].R.ParameterG = TempGamma.ParameterG;
        STK_calcRGBGamma(64, &stk6b1x_als.ChannelData[5], 255, &stk6b1x_als.ChannelData[2], &TempGamma);
        CaliData.RGBGamma[0].G.ParameterF = TempGamma.ParameterF;
        CaliData.RGBGamma[0].G.ParameterG = TempGamma.ParameterG;
        STK_calcRGBGamma(64, &stk6b1x_als.ChannelData[6], 255, &stk6b1x_als.ChannelData[3], &TempGamma);
        CaliData.RGBGamma[0].B.ParameterF = TempGamma.ParameterF;
        CaliData.RGBGamma[0].B.ParameterG = TempGamma.ParameterG;
        // CaliData.BrightnessGamma[0] for Red Brightness
        CaliData.BrightnessGamma[0].Level = 3212;
        STK_calcBrightnessGamma(3212, &stk6b1x_als.ChannelData[7], 4095, &stk6b1x_als.ChannelData[1], &TempGamma);
        CaliData.BrightnessGamma[0].R.ParameterF = TempGamma.ParameterF;
        CaliData.BrightnessGamma[0].R.ParameterG = TempGamma.ParameterG;
        // CaliData.BrightnessGamma[1] for Green Brightness
        STK_calcBrightnessGamma(3212, &stk6b1x_als.ChannelData[8], 4095, &stk6b1x_als.ChannelData[2], &TempGamma);
        CaliData.BrightnessGamma[0].G.ParameterF = TempGamma.ParameterF;
        CaliData.BrightnessGamma[0].G.ParameterG = TempGamma.ParameterG;
        // CaliData.BrightnessGamma[2] for Blue Brightness
        STK_calcBrightnessGamma(3212, &stk6b1x_als.ChannelData[9], 4095, &stk6b1x_als.ChannelData[3], &TempGamma);
        CaliData.BrightnessGamma[0].B.ParameterF = TempGamma.ParameterF;
        CaliData.BrightnessGamma[0].B.ParameterG = TempGamma.ParameterG;
        // Modified for Every Device
        // R255, G0, B0, Maximum Brightness Screen -1 SensorData[1].channelF channelG
        CaliData.RGB255Data.R.ChannelF = stk6b1x_als.ChannelData[1].ChannelF ;
        CaliData.RGB255Data.R.ChannelG = stk6b1x_als.ChannelData[1].ChannelG;
        // R0, G255, B0, Maximum Brightness Screen -2  SensorData[2].channelF channelG
        CaliData.RGB255Data.G.ChannelF = stk6b1x_als.ChannelData[2].ChannelF;
        CaliData.RGB255Data.G.ChannelG = stk6b1x_als.ChannelData[2].ChannelG;
        // R0, G0, B255, Maximum Brightness Screen -3
        CaliData.RGB255Data.B.ChannelF = stk6b1x_als.ChannelData[3].ChannelF;
        CaliData.RGB255Data.B.ChannelG = stk6b1x_als.ChannelData[3].ChannelG;
        CaliData.RGB255Data.Brightness = 4095;
        /*
        //      Debug use
                STK_LOG("CaliData.RGBGamma[0].R.ParameterG = %f\n", CaliData.RGBGamma[0].R.ParameterG);
                STK_LOG("CaliData.RGBGamma[0].G.ParameterG = %f\n", CaliData.RGBGamma[0].G.ParameterG);
                STK_LOG("CaliData.RGBGamma[0].B.ParameterG = %f\n", CaliData.RGBGamma[0].B.ParameterG);

                STK_LOG("CaliData.BrightnessGamma[0].R.ParameterG = %f\n",
                                        CaliData.BrightnessGamma[0].R.ParameterG);
                STK_LOG("CaliData.BrightnessGamma[0].G.ParameterG = %f\n",
                                        CaliData.BrightnessGamma[0].G.ParameterG);
                STK_LOG("CaliData.BrightnessGamma[0].B.ParameterG = %f\n",
                                        CaliData.BrightnessGamma[0].B.ParameterG);
                STK_LOG("CaliData.RGB255Data.R.ChannelG = %d\n",
                                        CaliData.RGB255Data.R.ChannelG);
                STK_LOG("CaliData.RGB255Data.G.ChannelG = %d\n",
                                        CaliData.RGB255Data.G.ChannelG);
                STK_LOG("CaliData.RGB255Data.B.ChannelG = %d\n",
                                        CaliData.RGB255Data.B.ChannelG);
        */
    }
    else
    {
        // Calibration Parameter Configure
        // DO NOT MODIFIED !!
        CaliData.RGBGamma[0].Level = 64;
        CaliData.RGBGamma[0].R.ParameterF = 1.41935;
        CaliData.RGBGamma[0].R.ParameterG = 1.36767;
        CaliData.RGBGamma[0].G.ParameterF = 1.43815;
        CaliData.RGBGamma[0].G.ParameterG = 1.46427;
        CaliData.RGBGamma[0].B.ParameterF = 2.26108;
        CaliData.RGBGamma[0].B.ParameterG = 2.51930;
        CaliData.BrightnessGamma[0].Level = 1024;
        CaliData.BrightnessGamma[0].R.ParameterF = 0.72682;
        CaliData.BrightnessGamma[0].R.ParameterG = 0.72143;
        CaliData.BrightnessGamma[0].G.ParameterF = 0.67264;
        CaliData.BrightnessGamma[0].G.ParameterG = 0.67740;
        CaliData.BrightnessGamma[0].B.ParameterF = 0.86743;
        CaliData.BrightnessGamma[0].B.ParameterG = 0.87444;
        // Modified for Every Device
        // R255, G0, B0, Maximum Brightness Screen
        CaliData.RGB255Data.R.ChannelF = 4610 * 128;
        CaliData.RGB255Data.R.ChannelG = 3590 * 128;
        // R0, G255, B0, Maximum Brightness Screen
        CaliData.RGB255Data.G.ChannelF = 7484 * 128;
        CaliData.RGB255Data.G.ChannelG = 16071 * 128;
        // R0, G0, B255, Maximum Brightness Screen
        CaliData.RGB255Data.B.ChannelF = 1116 * 128;
        CaliData.RGB255Data.B.ChannelG = 716 * 128;
        // Maximum Brightness Level
        CaliData.RGB255Data.Brightness = 4095;
    }

    CaliData.RGBGammaRatio[0].Level = 32;
    CaliData.RGBGammaRatio[0].R.ParameterF = 0.99180;
    CaliData.RGBGammaRatio[0].R.ParameterG = 1.03221;
    CaliData.RGBGammaRatio[0].G.ParameterF = 0.92540;
    CaliData.RGBGammaRatio[0].G.ParameterG = 0.92559;
    CaliData.RGBGammaRatio[0].B.ParameterF = 1.00384;
    CaliData.RGBGammaRatio[0].B.ParameterG = 1.00000;
    CaliData.RGBGammaRatio[1].Level = 64;
    CaliData.RGBGammaRatio[1].R.ParameterF = 1.00000;
    CaliData.RGBGammaRatio[1].R.ParameterG = 1.00000;
    CaliData.RGBGammaRatio[1].G.ParameterF = 1.00000;
    CaliData.RGBGammaRatio[1].G.ParameterG = 1.00000;
    CaliData.RGBGammaRatio[1].B.ParameterF = 1.00000;
    CaliData.RGBGammaRatio[1].B.ParameterG = 1.00000;
    CaliData.RGBGammaRatio[2].Level = 96;
    CaliData.RGBGammaRatio[2].R.ParameterF = 1.01644;
    CaliData.RGBGammaRatio[2].R.ParameterG = 1.01109;
    CaliData.RGBGammaRatio[2].G.ParameterF = 1.04305;
    CaliData.RGBGammaRatio[2].G.ParameterG = 1.04275;
    CaliData.RGBGammaRatio[2].B.ParameterF = 1.00782;
    CaliData.RGBGammaRatio[2].B.ParameterG = 0.95708;
    CaliData.RGBGammaRatio[3].Level = 128;
    CaliData.RGBGammaRatio[3].R.ParameterF = 1.02863;
    CaliData.RGBGammaRatio[3].R.ParameterG = 1.02125;
    CaliData.RGBGammaRatio[3].G.ParameterF = 1.07046;
    CaliData.RGBGammaRatio[3].G.ParameterG = 1.07051;
    CaliData.RGBGammaRatio[3].B.ParameterF = 1.01451;
    CaliData.RGBGammaRatio[3].B.ParameterG = 0.94207;
    CaliData.RGBGammaRatio[4].Level = 160;
    CaliData.RGBGammaRatio[4].R.ParameterF = 1.03947;
    CaliData.RGBGammaRatio[4].R.ParameterG = 1.03145;
    CaliData.RGBGammaRatio[4].G.ParameterF = 1.09492;
    CaliData.RGBGammaRatio[4].G.ParameterG = 1.09569;
    CaliData.RGBGammaRatio[4].B.ParameterF = 1.02181;
    CaliData.RGBGammaRatio[4].B.ParameterG = 0.93911;
    CaliData.RGBGammaRatio[5].Level = 196;
    CaliData.RGBGammaRatio[5].R.ParameterF = 1.04260;
    CaliData.RGBGammaRatio[5].R.ParameterG = 1.03378;
    CaliData.RGBGammaRatio[5].G.ParameterF = 1.10545;
    CaliData.RGBGammaRatio[5].G.ParameterG = 1.10701;
    CaliData.RGBGammaRatio[5].B.ParameterF = 1.02130;
    CaliData.RGBGammaRatio[5].B.ParameterG = 0.93678;
    CaliData.RGBGammaRatio[6].Level = 224;
    CaliData.RGBGammaRatio[6].R.ParameterF = 1.03831;
    CaliData.RGBGammaRatio[6].R.ParameterG = 1.02730;
    CaliData.RGBGammaRatio[6].G.ParameterF = 1.10873;
    CaliData.RGBGammaRatio[6].G.ParameterG = 1.11116;
    CaliData.RGBGammaRatio[6].B.ParameterF = 1.04391;
    CaliData.RGBGammaRatio[6].B.ParameterG = 0.94416;
    CaliData.RGBGammaRatio[7].Level = 255;
    CaliData.RGBGammaRatio[7].R.ParameterF = 1.03831;
    CaliData.RGBGammaRatio[7].R.ParameterG = 1.02730;
    CaliData.RGBGammaRatio[7].G.ParameterF = 1.10873;
    CaliData.RGBGammaRatio[7].G.ParameterG = 1.11116;
    CaliData.RGBGammaRatio[7].B.ParameterF = 1.04391;
    CaliData.RGBGammaRatio[7].B.ParameterG = 0.94416;
    CaliData.BrightnessGammaRatio[0].Level = 200;
    CaliData.BrightnessGammaRatio[0].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[0].Ratio[0].R.ParameterF = 1.16247;
    CaliData.BrightnessGammaRatio[0].Ratio[0].R.ParameterG = 1.15110;
    CaliData.BrightnessGammaRatio[0].Ratio[0].G.ParameterF = 0.51141;
    CaliData.BrightnessGammaRatio[0].Ratio[0].G.ParameterG = 0.53237;
    CaliData.BrightnessGammaRatio[0].Ratio[0].B.ParameterF = 1.30391;
    CaliData.BrightnessGammaRatio[0].Ratio[0].B.ParameterG = 1.85170;
    CaliData.BrightnessGammaRatio[0].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[0].Ratio[1].R.ParameterF = 0.69577;
    CaliData.BrightnessGammaRatio[0].Ratio[1].R.ParameterG = 0.81269;
    CaliData.BrightnessGammaRatio[0].Ratio[1].G.ParameterF = 0.53928;
    CaliData.BrightnessGammaRatio[0].Ratio[1].G.ParameterG = 0.54307;
    CaliData.BrightnessGammaRatio[0].Ratio[1].B.ParameterF = 1.02394;
    CaliData.BrightnessGammaRatio[0].Ratio[1].B.ParameterG = 1.04805;
    CaliData.BrightnessGammaRatio[0].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[0].Ratio[2].R.ParameterF = 0.68281;
    CaliData.BrightnessGammaRatio[0].Ratio[2].R.ParameterG = 0.72512;
    CaliData.BrightnessGammaRatio[0].Ratio[2].G.ParameterF = 0.58857;
    CaliData.BrightnessGammaRatio[0].Ratio[2].G.ParameterG = 0.58973;
    CaliData.BrightnessGammaRatio[0].Ratio[2].B.ParameterF = 0.71967;
    CaliData.BrightnessGammaRatio[0].Ratio[2].B.ParameterG = 0.92248;
    CaliData.BrightnessGammaRatio[0].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[0].Ratio[3].R.ParameterF = 0.68622;
    CaliData.BrightnessGammaRatio[0].Ratio[3].R.ParameterG = 0.70408;
    CaliData.BrightnessGammaRatio[0].Ratio[3].G.ParameterF = 0.62642;
    CaliData.BrightnessGammaRatio[0].Ratio[3].G.ParameterG = 0.62658;
    CaliData.BrightnessGammaRatio[0].Ratio[3].B.ParameterF = 0.70927;
    CaliData.BrightnessGammaRatio[0].Ratio[3].B.ParameterG = 1.18229;
    CaliData.BrightnessGammaRatio[1].Level = 300;
    CaliData.BrightnessGammaRatio[1].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[1].Ratio[0].R.ParameterF = 1.16247;
    CaliData.BrightnessGammaRatio[1].Ratio[0].R.ParameterG = 1.15110;
    CaliData.BrightnessGammaRatio[1].Ratio[0].G.ParameterF = 0.54625;
    CaliData.BrightnessGammaRatio[1].Ratio[0].G.ParameterG = 0.56998;
    CaliData.BrightnessGammaRatio[1].Ratio[0].B.ParameterF = 1.30391;
    CaliData.BrightnessGammaRatio[1].Ratio[0].B.ParameterG = 1.85170;
    CaliData.BrightnessGammaRatio[1].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[1].Ratio[1].R.ParameterF = 0.73031;
    CaliData.BrightnessGammaRatio[1].Ratio[1].R.ParameterG = 0.81269;
    CaliData.BrightnessGammaRatio[1].Ratio[1].G.ParameterF = 0.59568;
    CaliData.BrightnessGammaRatio[1].Ratio[1].G.ParameterG = 0.59886;
    CaliData.BrightnessGammaRatio[1].Ratio[1].B.ParameterF = 0.75881;
    CaliData.BrightnessGammaRatio[1].Ratio[1].B.ParameterG = 1.04805;
    CaliData.BrightnessGammaRatio[1].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[1].Ratio[2].R.ParameterF = 0.72653;
    CaliData.BrightnessGammaRatio[1].Ratio[2].R.ParameterG = 0.74238;
    CaliData.BrightnessGammaRatio[1].Ratio[2].G.ParameterF = 0.65081;
    CaliData.BrightnessGammaRatio[1].Ratio[2].G.ParameterG = 0.64968;
    CaliData.BrightnessGammaRatio[1].Ratio[2].B.ParameterF = 0.74282;
    CaliData.BrightnessGammaRatio[1].Ratio[2].B.ParameterG = 0.92248;
    CaliData.BrightnessGammaRatio[1].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[1].Ratio[3].R.ParameterF = 0.73229;
    CaliData.BrightnessGammaRatio[1].Ratio[3].R.ParameterG = 0.74392;
    CaliData.BrightnessGammaRatio[1].Ratio[3].G.ParameterF = 0.69011;
    CaliData.BrightnessGammaRatio[1].Ratio[3].G.ParameterG = 0.69091;
    CaliData.BrightnessGammaRatio[1].Ratio[3].B.ParameterF = 0.74836;
    CaliData.BrightnessGammaRatio[1].Ratio[3].B.ParameterG = 0.81115;
    CaliData.BrightnessGammaRatio[2].Level = 400;
    CaliData.BrightnessGammaRatio[2].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[2].Ratio[0].R.ParameterF = 0.88056;
    CaliData.BrightnessGammaRatio[2].Ratio[0].R.ParameterG = 1.15110;
    CaliData.BrightnessGammaRatio[2].Ratio[0].G.ParameterF = 0.57219;
    CaliData.BrightnessGammaRatio[2].Ratio[0].G.ParameterG = 0.58680;
    CaliData.BrightnessGammaRatio[2].Ratio[0].B.ParameterF = 1.30391;
    CaliData.BrightnessGammaRatio[2].Ratio[0].B.ParameterG = 1.85170;
    CaliData.BrightnessGammaRatio[2].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[2].Ratio[1].R.ParameterF = 0.75251;
    CaliData.BrightnessGammaRatio[2].Ratio[1].R.ParameterG = 0.80447;
    CaliData.BrightnessGammaRatio[2].Ratio[1].G.ParameterF = 0.63507;
    CaliData.BrightnessGammaRatio[2].Ratio[1].G.ParameterG = 0.63543;
    CaliData.BrightnessGammaRatio[2].Ratio[1].B.ParameterF = 0.78529;
    CaliData.BrightnessGammaRatio[2].Ratio[1].B.ParameterG = 1.04805;
    CaliData.BrightnessGammaRatio[2].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[2].Ratio[2].R.ParameterF = 0.76033;
    CaliData.BrightnessGammaRatio[2].Ratio[2].R.ParameterG = 0.76893;
    CaliData.BrightnessGammaRatio[2].Ratio[2].G.ParameterF = 0.69697;
    CaliData.BrightnessGammaRatio[2].Ratio[2].G.ParameterG = 0.69578;
    CaliData.BrightnessGammaRatio[2].Ratio[2].B.ParameterF = 0.78075;
    CaliData.BrightnessGammaRatio[2].Ratio[2].B.ParameterG = 0.92248;
    CaliData.BrightnessGammaRatio[2].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[2].Ratio[3].R.ParameterF = 0.76872;
    CaliData.BrightnessGammaRatio[2].Ratio[3].R.ParameterG = 0.77467;
    CaliData.BrightnessGammaRatio[2].Ratio[3].G.ParameterF = 0.73918;
    CaliData.BrightnessGammaRatio[2].Ratio[3].G.ParameterG = 0.73896;
    CaliData.BrightnessGammaRatio[2].Ratio[3].B.ParameterF = 0.78152;
    CaliData.BrightnessGammaRatio[2].Ratio[3].B.ParameterG = 0.85078;
    CaliData.BrightnessGammaRatio[3].Level = 500;
    CaliData.BrightnessGammaRatio[3].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[3].Ratio[0].R.ParameterF = 0.83268;
    CaliData.BrightnessGammaRatio[3].Ratio[0].R.ParameterG = 1.15110;
    CaliData.BrightnessGammaRatio[3].Ratio[0].G.ParameterF = 0.59250;
    CaliData.BrightnessGammaRatio[3].Ratio[0].G.ParameterG = 0.60170;
    CaliData.BrightnessGammaRatio[3].Ratio[0].B.ParameterF = 1.30391;
    CaliData.BrightnessGammaRatio[3].Ratio[0].B.ParameterG = 1.85170;
    CaliData.BrightnessGammaRatio[3].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[3].Ratio[1].R.ParameterF = 0.76844;
    CaliData.BrightnessGammaRatio[3].Ratio[1].R.ParameterG = 0.80470;
    CaliData.BrightnessGammaRatio[3].Ratio[1].G.ParameterF = 0.66475;
    CaliData.BrightnessGammaRatio[3].Ratio[1].G.ParameterG = 0.66635;
    CaliData.BrightnessGammaRatio[3].Ratio[1].B.ParameterF = 0.79467;
    CaliData.BrightnessGammaRatio[3].Ratio[1].B.ParameterG = 1.04805;
    CaliData.BrightnessGammaRatio[3].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[3].Ratio[2].R.ParameterF = 0.78154;
    CaliData.BrightnessGammaRatio[3].Ratio[2].R.ParameterG = 0.78986;
    CaliData.BrightnessGammaRatio[3].Ratio[2].G.ParameterF = 0.73119;
    CaliData.BrightnessGammaRatio[3].Ratio[2].G.ParameterG = 0.73058;
    CaliData.BrightnessGammaRatio[3].Ratio[2].B.ParameterF = 0.80557;
    CaliData.BrightnessGammaRatio[3].Ratio[2].B.ParameterG = 0.89219;
    CaliData.BrightnessGammaRatio[3].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[3].Ratio[3].R.ParameterF = 0.79635;
    CaliData.BrightnessGammaRatio[3].Ratio[3].R.ParameterG = 0.79896;
    CaliData.BrightnessGammaRatio[3].Ratio[3].G.ParameterF = 0.77568;
    CaliData.BrightnessGammaRatio[3].Ratio[3].G.ParameterG = 0.77583;
    CaliData.BrightnessGammaRatio[3].Ratio[3].B.ParameterF = 0.80954;
    CaliData.BrightnessGammaRatio[3].Ratio[3].B.ParameterG = 0.85618;
    CaliData.BrightnessGammaRatio[4].Level = 700;
    CaliData.BrightnessGammaRatio[4].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[4].Ratio[0].R.ParameterF = 0.82793;
    CaliData.BrightnessGammaRatio[4].Ratio[0].R.ParameterG = 1.15110;
    CaliData.BrightnessGammaRatio[4].Ratio[0].G.ParameterF = 0.62715;
    CaliData.BrightnessGammaRatio[4].Ratio[0].G.ParameterG = 0.63230;
    CaliData.BrightnessGammaRatio[4].Ratio[0].B.ParameterF = 0.95405;
    CaliData.BrightnessGammaRatio[4].Ratio[0].B.ParameterG = 1.85170;
    CaliData.BrightnessGammaRatio[4].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[4].Ratio[1].R.ParameterF = 0.81468;
    CaliData.BrightnessGammaRatio[4].Ratio[1].R.ParameterG = 0.83523;
    CaliData.BrightnessGammaRatio[4].Ratio[1].G.ParameterF = 0.71990;
    CaliData.BrightnessGammaRatio[4].Ratio[1].G.ParameterG = 0.71915;
    CaliData.BrightnessGammaRatio[4].Ratio[1].B.ParameterF = 0.83870;
    CaliData.BrightnessGammaRatio[4].Ratio[1].B.ParameterG = 1.04805;
    CaliData.BrightnessGammaRatio[4].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[4].Ratio[2].R.ParameterF = 0.83268;
    CaliData.BrightnessGammaRatio[4].Ratio[2].R.ParameterG = 0.83488;
    CaliData.BrightnessGammaRatio[4].Ratio[2].G.ParameterF = 0.79115;
    CaliData.BrightnessGammaRatio[4].Ratio[2].G.ParameterG = 0.79045;
    CaliData.BrightnessGammaRatio[4].Ratio[2].B.ParameterF = 0.84851;
    CaliData.BrightnessGammaRatio[4].Ratio[2].B.ParameterG = 0.88938;
    CaliData.BrightnessGammaRatio[4].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[4].Ratio[3].R.ParameterF = 0.85167;
    CaliData.BrightnessGammaRatio[4].Ratio[3].R.ParameterG = 0.85268;
    CaliData.BrightnessGammaRatio[4].Ratio[3].G.ParameterF = 0.83807;
    CaliData.BrightnessGammaRatio[4].Ratio[3].G.ParameterG = 0.83780;
    CaliData.BrightnessGammaRatio[4].Ratio[3].B.ParameterF = 0.85897;
    CaliData.BrightnessGammaRatio[4].Ratio[3].B.ParameterG = 0.87581;
    CaliData.BrightnessGammaRatio[5].Level = 900;
    CaliData.BrightnessGammaRatio[5].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[5].Ratio[0].R.ParameterF = 0.87673;
    CaliData.BrightnessGammaRatio[5].Ratio[0].R.ParameterG = 1.01961;
    CaliData.BrightnessGammaRatio[5].Ratio[0].G.ParameterF = 0.65788;
    CaliData.BrightnessGammaRatio[5].Ratio[0].G.ParameterG = 0.65917;
    CaliData.BrightnessGammaRatio[5].Ratio[0].B.ParameterF = 0.94825;
    CaliData.BrightnessGammaRatio[5].Ratio[0].B.ParameterG = 1.85170;
    CaliData.BrightnessGammaRatio[5].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[5].Ratio[1].R.ParameterF = 0.87046;
    CaliData.BrightnessGammaRatio[5].Ratio[1].R.ParameterG = 0.88034;
    CaliData.BrightnessGammaRatio[5].Ratio[1].G.ParameterF = 0.78456;
    CaliData.BrightnessGammaRatio[5].Ratio[1].G.ParameterG = 0.78274;
    CaliData.BrightnessGammaRatio[5].Ratio[1].B.ParameterF = 0.88852;
    CaliData.BrightnessGammaRatio[5].Ratio[1].B.ParameterG = 1.01355;
    CaliData.BrightnessGammaRatio[5].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[5].Ratio[2].R.ParameterF = 0.89709;
    CaliData.BrightnessGammaRatio[5].Ratio[2].R.ParameterG = 0.89923;
    CaliData.BrightnessGammaRatio[5].Ratio[2].G.ParameterF = 0.85961;
    CaliData.BrightnessGammaRatio[5].Ratio[2].G.ParameterG = 0.85818;
    CaliData.BrightnessGammaRatio[5].Ratio[2].B.ParameterF = 0.90570;
    CaliData.BrightnessGammaRatio[5].Ratio[2].B.ParameterG = 0.93072;
    CaliData.BrightnessGammaRatio[5].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[5].Ratio[3].R.ParameterF = 0.91789;
    CaliData.BrightnessGammaRatio[5].Ratio[3].R.ParameterG = 0.91910;
    CaliData.BrightnessGammaRatio[5].Ratio[3].G.ParameterF = 0.90959;
    CaliData.BrightnessGammaRatio[5].Ratio[3].G.ParameterG = 0.90935;
    CaliData.BrightnessGammaRatio[5].Ratio[3].B.ParameterF = 0.92157;
    CaliData.BrightnessGammaRatio[5].Ratio[3].B.ParameterG = 0.93260;
    CaliData.BrightnessGammaRatio[6].Level = 1100;
    CaliData.BrightnessGammaRatio[6].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[6].Ratio[0].R.ParameterF = 0.93760;
    CaliData.BrightnessGammaRatio[6].Ratio[0].R.ParameterG = 1.05926;
    CaliData.BrightnessGammaRatio[6].Ratio[0].G.ParameterF = 0.72555;
    CaliData.BrightnessGammaRatio[6].Ratio[0].G.ParameterG = 0.72103;
    CaliData.BrightnessGammaRatio[6].Ratio[0].B.ParameterF = 0.97727;
    CaliData.BrightnessGammaRatio[6].Ratio[0].B.ParameterG = 1.85170;
    CaliData.BrightnessGammaRatio[6].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[6].Ratio[1].R.ParameterF = 0.94365;
    CaliData.BrightnessGammaRatio[6].Ratio[1].R.ParameterG = 0.94592;
    CaliData.BrightnessGammaRatio[6].Ratio[1].G.ParameterF = 0.86764;
    CaliData.BrightnessGammaRatio[6].Ratio[1].G.ParameterG = 0.86558;
    CaliData.BrightnessGammaRatio[6].Ratio[1].B.ParameterF = 0.96781;
    CaliData.BrightnessGammaRatio[6].Ratio[1].B.ParameterG = 1.03139;
    CaliData.BrightnessGammaRatio[6].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[6].Ratio[2].R.ParameterF = 0.97555;
    CaliData.BrightnessGammaRatio[6].Ratio[2].R.ParameterG = 0.97636;
    CaliData.BrightnessGammaRatio[6].Ratio[2].G.ParameterF = 0.94876;
    CaliData.BrightnessGammaRatio[6].Ratio[2].G.ParameterG = 0.94756;
    CaliData.BrightnessGammaRatio[6].Ratio[2].B.ParameterF = 0.98393;
    CaliData.BrightnessGammaRatio[6].Ratio[2].B.ParameterG = 0.98343;
    CaliData.BrightnessGammaRatio[6].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[6].Ratio[3].R.ParameterF = 1.00000;
    CaliData.BrightnessGammaRatio[6].Ratio[3].R.ParameterG = 1.00000;
    CaliData.BrightnessGammaRatio[6].Ratio[3].G.ParameterF = 1.00000;
    CaliData.BrightnessGammaRatio[6].Ratio[3].G.ParameterG = 1.00000;
    CaliData.BrightnessGammaRatio[6].Ratio[3].B.ParameterF = 1.00000;
    CaliData.BrightnessGammaRatio[6].Ratio[3].B.ParameterG = 1.00000;
    CaliData.BrightnessGammaRatio[7].Level = 1300;
    CaliData.BrightnessGammaRatio[7].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[7].Ratio[0].R.ParameterF = 1.03373;
    CaliData.BrightnessGammaRatio[7].Ratio[0].R.ParameterG = 1.12384;
    CaliData.BrightnessGammaRatio[7].Ratio[0].G.ParameterF = 0.81763;
    CaliData.BrightnessGammaRatio[7].Ratio[0].G.ParameterG = 0.81845;
    CaliData.BrightnessGammaRatio[7].Ratio[0].B.ParameterF = 1.05228;
    CaliData.BrightnessGammaRatio[7].Ratio[0].B.ParameterG = 1.85170;
    CaliData.BrightnessGammaRatio[7].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[7].Ratio[1].R.ParameterF = 1.04277;
    CaliData.BrightnessGammaRatio[7].Ratio[1].R.ParameterG = 1.04625;
    CaliData.BrightnessGammaRatio[7].Ratio[1].G.ParameterF = 0.97483;
    CaliData.BrightnessGammaRatio[7].Ratio[1].G.ParameterG = 0.97167;
    CaliData.BrightnessGammaRatio[7].Ratio[1].B.ParameterF = 1.06724;
    CaliData.BrightnessGammaRatio[7].Ratio[1].B.ParameterG = 1.11279;
    CaliData.BrightnessGammaRatio[7].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[7].Ratio[2].R.ParameterF = 1.08256;
    CaliData.BrightnessGammaRatio[7].Ratio[2].R.ParameterG = 1.08102;
    CaliData.BrightnessGammaRatio[7].Ratio[2].G.ParameterF = 1.06486;
    CaliData.BrightnessGammaRatio[7].Ratio[2].G.ParameterG = 1.06263;
    CaliData.BrightnessGammaRatio[7].Ratio[2].B.ParameterF = 1.08823;
    CaliData.BrightnessGammaRatio[7].Ratio[2].B.ParameterG = 1.09675;
    CaliData.BrightnessGammaRatio[7].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[7].Ratio[3].R.ParameterF = 1.11112;
    CaliData.BrightnessGammaRatio[7].Ratio[3].R.ParameterG = 1.10937;
    CaliData.BrightnessGammaRatio[7].Ratio[3].G.ParameterF = 1.11939;
    CaliData.BrightnessGammaRatio[7].Ratio[3].G.ParameterG = 1.11957;
    CaliData.BrightnessGammaRatio[7].Ratio[3].B.ParameterF = 1.11086;
    CaliData.BrightnessGammaRatio[7].Ratio[3].B.ParameterG = 1.10497;
    CaliData.BrightnessGammaRatio[8].Level = 1500;
    CaliData.BrightnessGammaRatio[8].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[8].Ratio[0].R.ParameterF = 1.19777;
    CaliData.BrightnessGammaRatio[8].Ratio[0].R.ParameterG = 1.25446;
    CaliData.BrightnessGammaRatio[8].Ratio[0].G.ParameterF = 0.97301;
    CaliData.BrightnessGammaRatio[8].Ratio[0].G.ParameterG = 0.96979;
    CaliData.BrightnessGammaRatio[8].Ratio[0].B.ParameterF = 1.23853;
    CaliData.BrightnessGammaRatio[8].Ratio[0].B.ParameterG = 1.75758;
    CaliData.BrightnessGammaRatio[8].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[8].Ratio[1].R.ParameterF = 1.22288;
    CaliData.BrightnessGammaRatio[8].Ratio[1].R.ParameterG = 1.22001;
    CaliData.BrightnessGammaRatio[8].Ratio[1].G.ParameterF = 1.15548;
    CaliData.BrightnessGammaRatio[8].Ratio[1].G.ParameterG = 1.15141;
    CaliData.BrightnessGammaRatio[8].Ratio[1].B.ParameterF = 1.24235;
    CaliData.BrightnessGammaRatio[8].Ratio[1].B.ParameterG = 1.27101;
    CaliData.BrightnessGammaRatio[8].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[8].Ratio[2].R.ParameterF = 1.26791;
    CaliData.BrightnessGammaRatio[8].Ratio[2].R.ParameterG = 1.26389;
    CaliData.BrightnessGammaRatio[8].Ratio[2].G.ParameterF = 1.25930;
    CaliData.BrightnessGammaRatio[8].Ratio[2].G.ParameterG = 1.25752;
    CaliData.BrightnessGammaRatio[8].Ratio[2].B.ParameterF = 1.27597;
    CaliData.BrightnessGammaRatio[8].Ratio[2].B.ParameterG = 1.27236;
    CaliData.BrightnessGammaRatio[8].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[8].Ratio[3].R.ParameterF = 1.29902;
    CaliData.BrightnessGammaRatio[8].Ratio[3].R.ParameterG = 1.29670;
    CaliData.BrightnessGammaRatio[8].Ratio[3].G.ParameterF = 1.31795;
    CaliData.BrightnessGammaRatio[8].Ratio[3].G.ParameterG = 1.31817;
    CaliData.BrightnessGammaRatio[8].Ratio[3].B.ParameterF = 1.29901;
    CaliData.BrightnessGammaRatio[8].Ratio[3].B.ParameterG = 1.29008;
    CaliData.BrightnessGammaRatio[9].Level = 1700;
    CaliData.BrightnessGammaRatio[9].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[9].Ratio[0].R.ParameterF = 1.45637;
    CaliData.BrightnessGammaRatio[9].Ratio[0].R.ParameterG = 1.48154;
    CaliData.BrightnessGammaRatio[9].Ratio[0].G.ParameterF = 1.20885;
    CaliData.BrightnessGammaRatio[9].Ratio[0].G.ParameterG = 1.20517;
    CaliData.BrightnessGammaRatio[9].Ratio[0].B.ParameterF = 1.47703;
    CaliData.BrightnessGammaRatio[9].Ratio[0].B.ParameterG = 2.14683;
    CaliData.BrightnessGammaRatio[9].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[9].Ratio[1].R.ParameterF = 1.48437;
    CaliData.BrightnessGammaRatio[9].Ratio[1].R.ParameterG = 1.47758;
    CaliData.BrightnessGammaRatio[9].Ratio[1].G.ParameterF = 1.42023;
    CaliData.BrightnessGammaRatio[9].Ratio[1].G.ParameterG = 1.41687;
    CaliData.BrightnessGammaRatio[9].Ratio[1].B.ParameterF = 1.50592;
    CaliData.BrightnessGammaRatio[9].Ratio[1].B.ParameterG = 1.55338;
    CaliData.BrightnessGammaRatio[9].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[9].Ratio[2].R.ParameterF = 1.54552;
    CaliData.BrightnessGammaRatio[9].Ratio[2].R.ParameterG = 1.53995;
    CaliData.BrightnessGammaRatio[9].Ratio[2].G.ParameterF = 1.54937;
    CaliData.BrightnessGammaRatio[9].Ratio[2].G.ParameterG = 1.54734;
    CaliData.BrightnessGammaRatio[9].Ratio[2].B.ParameterF = 1.54012;
    CaliData.BrightnessGammaRatio[9].Ratio[2].B.ParameterG = 1.51436;
    CaliData.BrightnessGammaRatio[9].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[9].Ratio[3].R.ParameterF = 1.58615;
    CaliData.BrightnessGammaRatio[9].Ratio[3].R.ParameterG = 1.58066;
    CaliData.BrightnessGammaRatio[9].Ratio[3].G.ParameterF = 1.61860;
    CaliData.BrightnessGammaRatio[9].Ratio[3].G.ParameterG = 1.61871;
    CaliData.BrightnessGammaRatio[9].Ratio[3].B.ParameterF = 1.57133;
    CaliData.BrightnessGammaRatio[9].Ratio[3].B.ParameterG = 1.55335;
    CaliData.BrightnessGammaRatio[10].Level = 1760;
    CaliData.BrightnessGammaRatio[10].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[10].Ratio[0].R.ParameterF = 1.59989;
    CaliData.BrightnessGammaRatio[10].Ratio[0].R.ParameterG = 1.62754;
    CaliData.BrightnessGammaRatio[10].Ratio[0].G.ParameterF = 1.22279;
    CaliData.BrightnessGammaRatio[10].Ratio[0].G.ParameterG = 1.21907;
    CaliData.BrightnessGammaRatio[10].Ratio[0].B.ParameterF = 1.67078;
    CaliData.BrightnessGammaRatio[10].Ratio[0].B.ParameterG = 2.42845;
    CaliData.BrightnessGammaRatio[10].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[10].Ratio[1].R.ParameterF = 1.63065;
    CaliData.BrightnessGammaRatio[10].Ratio[1].R.ParameterG = 1.62319;
    CaliData.BrightnessGammaRatio[10].Ratio[1].G.ParameterF = 1.43661;
    CaliData.BrightnessGammaRatio[10].Ratio[1].G.ParameterG = 1.43322;
    CaliData.BrightnessGammaRatio[10].Ratio[1].B.ParameterF = 1.70346;
    CaliData.BrightnessGammaRatio[10].Ratio[1].B.ParameterG = 1.75715;
    CaliData.BrightnessGammaRatio[10].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[10].Ratio[2].R.ParameterF = 1.69783;
    CaliData.BrightnessGammaRatio[10].Ratio[2].R.ParameterG = 1.69170;
    CaliData.BrightnessGammaRatio[10].Ratio[2].G.ParameterF = 1.56724;
    CaliData.BrightnessGammaRatio[10].Ratio[2].G.ParameterG = 1.56518;
    CaliData.BrightnessGammaRatio[10].Ratio[2].B.ParameterF = 1.74215;
    CaliData.BrightnessGammaRatio[10].Ratio[2].B.ParameterG = 1.71301;
    CaliData.BrightnessGammaRatio[10].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[10].Ratio[3].R.ParameterF = 1.74246;
    CaliData.BrightnessGammaRatio[10].Ratio[3].R.ParameterG = 1.73643;
    CaliData.BrightnessGammaRatio[10].Ratio[3].G.ParameterF = 1.63727;
    CaliData.BrightnessGammaRatio[10].Ratio[3].G.ParameterG = 1.63737;
    CaliData.BrightnessGammaRatio[10].Ratio[3].B.ParameterF = 1.77745;
    CaliData.BrightnessGammaRatio[10].Ratio[3].B.ParameterG = 1.75711;
    CaliData.BrightnessGammaRatio[11].Level = 1900;
    CaliData.BrightnessGammaRatio[11].Ratio[0].Level = 64;
    CaliData.BrightnessGammaRatio[11].Ratio[0].R.ParameterF = 1.47801;
    CaliData.BrightnessGammaRatio[11].Ratio[0].R.ParameterG = 1.50356;
    CaliData.BrightnessGammaRatio[11].Ratio[0].G.ParameterF = 1.22681;
    CaliData.BrightnessGammaRatio[11].Ratio[0].G.ParameterG = 1.22308;
    CaliData.BrightnessGammaRatio[11].Ratio[0].B.ParameterF = 1.49898;
    CaliData.BrightnessGammaRatio[11].Ratio[0].B.ParameterG = 2.17873;
    CaliData.BrightnessGammaRatio[11].Ratio[1].Level = 128;
    CaliData.BrightnessGammaRatio[11].Ratio[1].R.ParameterF = 1.50643;
    CaliData.BrightnessGammaRatio[11].Ratio[1].R.ParameterG = 1.49954;
    CaliData.BrightnessGammaRatio[11].Ratio[1].G.ParameterF = 1.44134;
    CaliData.BrightnessGammaRatio[11].Ratio[1].G.ParameterG = 1.43793;
    CaliData.BrightnessGammaRatio[11].Ratio[1].B.ParameterF = 1.52829;
    CaliData.BrightnessGammaRatio[11].Ratio[1].B.ParameterG = 1.57646;
    CaliData.BrightnessGammaRatio[11].Ratio[2].Level = 192;
    CaliData.BrightnessGammaRatio[11].Ratio[2].R.ParameterF = 1.56849;
    CaliData.BrightnessGammaRatio[11].Ratio[2].R.ParameterG = 1.56283;
    CaliData.BrightnessGammaRatio[11].Ratio[2].G.ParameterF = 1.57240;
    CaliData.BrightnessGammaRatio[11].Ratio[2].G.ParameterG = 1.57033;
    CaliData.BrightnessGammaRatio[11].Ratio[2].B.ParameterF = 1.56301;
    CaliData.BrightnessGammaRatio[11].Ratio[2].B.ParameterG = 1.53686;
    CaliData.BrightnessGammaRatio[11].Ratio[3].Level = 255;
    CaliData.BrightnessGammaRatio[11].Ratio[3].R.ParameterF = 1.60972;
    CaliData.BrightnessGammaRatio[11].Ratio[3].R.ParameterG = 1.60415;
    CaliData.BrightnessGammaRatio[11].Ratio[3].G.ParameterF = 1.64265;
    CaliData.BrightnessGammaRatio[11].Ratio[3].G.ParameterG = 1.64276;
    CaliData.BrightnessGammaRatio[11].Ratio[3].B.ParameterF = 1.59468;
    CaliData.BrightnessGammaRatio[11].Ratio[3].B.ParameterG = 1.57643;
    CaliData.Compensation[0].Level = 200;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[0].ParameterF = 9.01913;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[0].ParameterG = 2.60888;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[1].ParameterF = 10.75936;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[1].ParameterG = 15.18344;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[2].ParameterF = 2.56310;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[2].ParameterG = -200.03787;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[3].ParameterF = -0.26231;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[3].ParameterG = -0.24967;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[4].ParameterF = 7.47196;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[4].ParameterG = -0.18941;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[5].ParameterF = 7.99011;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[5].ParameterG = 10.50592;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[7].ParameterF = -0.18938;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[7].ParameterG = -0.17022;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[8].ParameterF = 51.40390;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[8].ParameterG = 7.99054;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[10].ParameterF = 22.27596;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[10].ParameterG = -3, 797.17283;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[11].ParameterF = -0.34289;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[11].ParameterG = 0.22217;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[13].ParameterF = 7.34632;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[13].ParameterG = 10.57701;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[14].ParameterF = 2.32949;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[14].ParameterG = -254.76649;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[15].ParameterF = -0.13980;
    CaliData.Compensation[0].Model.PolynomialFunc.Coeff[15].ParameterG = -0.14735;
    CaliData.Compensation[1].Level = 300;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[0].ParameterF = 4.07477;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[0].ParameterG = 3.14786;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[1].ParameterF = 4.33996;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[1].ParameterG = 7.23444;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[2].ParameterF = 0.99829;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[2].ParameterG = -1.19102;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[3].ParameterF = -0.17912;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[3].ParameterG = -0.20527;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[4].ParameterF = 3.36708;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[4].ParameterG = 2.31766;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[5].ParameterF = 3.50134;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[5].ParameterG = 4.58961;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[7].ParameterF = -0.13675;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[7].ParameterG = -0.13885;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[8].ParameterF = 12.46013;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[8].ParameterG = 35.23751;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[10].ParameterF = 3.66420;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[10].ParameterG = -1.47731;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[11].ParameterF = -0.16147;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[11].ParameterG = -0.27755;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[13].ParameterF = 2.36396;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[13].ParameterG = 5.48135;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[14].ParameterF = 0.25210;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[14].ParameterG = -1.20858;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[15].ParameterF = -0.07657;
    CaliData.Compensation[1].Model.PolynomialFunc.Coeff[15].ParameterG = -0.12302;
    CaliData.Compensation[2].Level = 400;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[0].ParameterF = 1.49831;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[0].ParameterG = 1.67278;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[1].ParameterF = 1.91328;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[1].ParameterG = 3.43414;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[2].ParameterF = 0.78615;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[2].ParameterG = 0.13836;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[3].ParameterF = -0.12424;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[3].ParameterG = -0.15038;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[4].ParameterF = 1.33523;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[4].ParameterG = 1.40081;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[5].ParameterF = 1.58150;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[5].ParameterG = 1.87602;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[7].ParameterF = -0.09378;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[7].ParameterG = -0.09392;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[8].ParameterF = 3.35339;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[8].ParameterG = 18.42784;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[10].ParameterF = 1.91563;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[10].ParameterG = 1.42503;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[11].ParameterF = -0.09211;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[11].ParameterG = -0.26734;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[13].ParameterF = 0.83043;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[13].ParameterG = 1.76466;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[14].ParameterF = 0.86052;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[14].ParameterG = -0.40894;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[15].ParameterF = -0.05497;
    CaliData.Compensation[2].Model.PolynomialFunc.Coeff[15].ParameterG = -0.06585;
    CaliData.Compensation[3].Level = 500;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[0].ParameterF = 0.65757;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[0].ParameterG = 1.09204;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[1].ParameterF = 0.84859;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[1].ParameterG = 1.86273;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[2].ParameterF = 0.56168;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[2].ParameterG = 0.09749;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[3].ParameterF = -0.08888;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[3].ParameterG = -0.11934;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[4].ParameterF = 0.68539;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[4].ParameterG = 1.10858;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[5].ParameterF = 0.70163;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[5].ParameterG = 1.17622;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[7].ParameterF = -0.06936;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[7].ParameterG = -0.08547;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[8].ParameterF = 1.07706;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[8].ParameterG = 7.58371;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[10].ParameterF = 0.82106;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[10].ParameterG = 0.77273;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[11].ParameterF = -0.05901;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[11].ParameterG = -0.18677;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[13].ParameterF = 0.35774;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[13].ParameterG = 1.28457;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[14].ParameterF = 0.46750;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[14].ParameterG = 0.30119;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[15].ParameterF = -0.03640;
    CaliData.Compensation[3].Model.PolynomialFunc.Coeff[15].ParameterG = -0.06563;
    CaliData.Compensation[4].Level = 700;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[0].ParameterF = 0.17108;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[0].ParameterG = 0.35849;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[1].ParameterF = 0.24501;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[1].ParameterG = 0.55280;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[2].ParameterF = 0.46890;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[2].ParameterG = 0.44076;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[3].ParameterF = -0.05294;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[3].ParameterG = -0.07354;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[4].ParameterF = 0.15797;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[4].ParameterG = 0.28727;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[5].ParameterF = 0.16200;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[5].ParameterG = 0.31067;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[7].ParameterF = -0.03799;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[7].ParameterG = -0.04862;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[8].ParameterF = 0.20310;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[8].ParameterG = 2.01128;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[10].ParameterF = 0.48774;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[10].ParameterG = 0.68459;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[11].ParameterF = -0.03475;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[11].ParameterG = -0.11708;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[13].ParameterF = -0.06559;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[13].ParameterG = 0.16722;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[14].ParameterF = 0.42405;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[14].ParameterG = 0.41190;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[15].ParameterF = -0.01512;
    CaliData.Compensation[4].Model.PolynomialFunc.Coeff[15].ParameterG = -0.02792;
    CaliData.Compensation[5].Level = 900;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[0].ParameterF = 0.09040;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[0].ParameterG = 0.15274;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[1].ParameterF = 0.12642;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[1].ParameterG = 0.22679;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[2].ParameterF = 0.39025;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[2].ParameterG = 0.41187;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[3].ParameterF = -0.03492;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[3].ParameterG = -0.04789;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[4].ParameterF = 0.09873;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[4].ParameterG = 0.13603;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[5].ParameterF = 0.09737;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[5].ParameterG = 0.10590;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[7].ParameterF = -0.02790;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[7].ParameterG = -0.03077;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[8].ParameterF = 0.00651;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[8].ParameterG = 0.62040;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[10].ParameterF = 0.35815;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[10].ParameterG = 0.43173;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[11].ParameterF = -0.01939;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[11].ParameterG = -0.06912;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[13].ParameterF = -0.04699;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[13].ParameterG = 0.00585;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[14].ParameterF = 0.38235;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[14].ParameterG = 0.38551;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[15].ParameterF = -0.00966;
    CaliData.Compensation[5].Model.PolynomialFunc.Coeff[15].ParameterG = -0.01314;
    CaliData.Compensation[6].Level = 1100;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[0].ParameterF = 0.04072;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[0].ParameterG = 0.08601;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[1].ParameterF = 0.06602;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[1].ParameterG = 0.05361;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[2].ParameterF = 0.35577;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[2].ParameterG = 0.37235;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[3].ParameterF = -0.02140;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[3].ParameterG = -0.02406;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[4].ParameterF = 0.06893;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[4].ParameterG = 0.10470;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[5].ParameterF = 0.05593;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[5].ParameterG = 0.03783;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[7].ParameterF = -0.01861;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[7].ParameterG = -0.01911;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[8].ParameterF = -0.03819;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[8].ParameterG = 0.16682;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[10].ParameterF = 0.32470;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[10].ParameterG = 0.39078;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[11].ParameterF = -0.01258;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[11].ParameterG = -0.03788;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[13].ParameterF = -0.05103;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[13].ParameterG = -0.05622;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[14].ParameterF = 0.32750;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[14].ParameterG = 0.38141;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[15].ParameterF = -0.00225;
    CaliData.Compensation[6].Model.PolynomialFunc.Coeff[15].ParameterG = -0.00328;
    CaliData.Compensation[7].Level = 1300;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[0].ParameterF = 0.03276;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[0].ParameterG = 0.08011;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[1].ParameterF = 0.04422;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[1].ParameterG = 0.00922;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[2].ParameterF = 0.32526;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[2].ParameterG = 0.34882;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[3].ParameterF = -0.01223;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[3].ParameterG = -0.01265;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[4].ParameterF = 0.05519;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[4].ParameterG = 0.09950;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[5].ParameterF = 0.04878;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[5].ParameterG = 0.01707;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[7].ParameterF = -0.01358;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[7].ParameterG = -0.01328;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[8].ParameterF = -0.04080;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[8].ParameterG = 0.06873;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[10].ParameterF = 0.26997;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[10].ParameterG = 0.31749;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[11].ParameterF = -0.00563;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[11].ParameterG = -0.02319;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[13].ParameterF = -0.05061;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[13].ParameterG = -0.08891;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[14].ParameterF = 0.29329;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[14].ParameterG = 0.36495;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[15].ParameterF = 0.00395;
    CaliData.Compensation[7].Model.PolynomialFunc.Coeff[15].ParameterG = 0.00594;
    CaliData.Compensation[8].Level = 1500;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[0].ParameterF = 0.02765;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[0].ParameterG = 0.06290;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[1].ParameterF = 0.03733;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[1].ParameterG = -0.01074;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[2].ParameterF = 0.29632;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[2].ParameterG = 0.32519;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[3].ParameterF = -0.00520;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[3].ParameterG = -0.00205;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[4].ParameterF = 0.06426;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[4].ParameterG = 0.09859;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[5].ParameterF = 0.04285;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[5].ParameterG = 0.01083;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[7].ParameterF = -0.01196;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[7].ParameterG = -0.00986;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[8].ParameterF = -0.03152;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[8].ParameterG = 0.00665;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[10].ParameterF = 0.24326;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[10].ParameterG = 0.27861;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[11].ParameterF = -0.00200;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[11].ParameterG = -0.01056;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[13].ParameterF = -0.04553;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[13].ParameterG = -0.09293;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[14].ParameterF = 0.27056;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[14].ParameterG = 0.33725;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[15].ParameterF = 0.00825;
    CaliData.Compensation[8].Model.PolynomialFunc.Coeff[15].ParameterG = 0.01305;
    CaliData.Compensation[9].Level = 1700;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[0].ParameterF = 0.02312;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[0].ParameterG = 0.05397;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[1].ParameterF = 0.02778;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[1].ParameterG = -0.02597;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[2].ParameterF = 0.27562;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[2].ParameterG = 0.30612;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[3].ParameterF = 0.00166;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[3].ParameterG = 0.00794;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[4].ParameterF = 0.05339;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[4].ParameterG = 0.08886;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[5].ParameterF = 0.04009;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[5].ParameterG = 0.01211;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[7].ParameterF = -0.00801;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[7].ParameterG = -0.00730;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[8].ParameterF = -0.03406;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[8].ParameterG = -0.02311;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[10].ParameterF = 0.22465;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[10].ParameterG = 0.25748;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[11].ParameterF = 0.00291;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[11].ParameterG = -0.00124;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[13].ParameterF = -0.04185;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[13].ParameterG = -0.09404;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[14].ParameterF = 0.24828;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[14].ParameterG = 0.31635;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[15].ParameterF = 0.01191;
    CaliData.Compensation[9].Model.PolynomialFunc.Coeff[15].ParameterG = 0.01936;
    CaliData.Compensation[10].Level = 1900;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[0].ParameterF = 0.01323;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[0].ParameterG = 0.03616;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[1].ParameterF = 0.01605;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[1].ParameterG = -0.04496;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[2].ParameterF = 0.23772;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[2].ParameterG = 0.27243;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[3].ParameterF = 0.01690;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[3].ParameterG = 0.03248;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[4].ParameterF = 0.04854;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[4].ParameterG = 0.07777;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[5].ParameterF = 0.04179;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[5].ParameterG = 0.01248;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[7].ParameterF = -0.00713;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[7].ParameterG = -0.00344;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[8].ParameterF = -0.02994;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[8].ParameterG = -0.03912;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[10].ParameterF = 0.19060;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[10].ParameterG = 0.22309;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[11].ParameterF = 0.01067;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[11].ParameterG = 0.01136;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[13].ParameterF = -0.04100;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[13].ParameterG = -0.10578;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[14].ParameterF = 0.21218;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[14].ParameterG = 0.28524;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[15].ParameterF = 0.02297;
    CaliData.Compensation[10].Model.PolynomialFunc.Coeff[15].ParameterG = 0.04046;
    CaliData.Compensation[11].Level = 2047;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[0].ParameterF = 0.00309;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[0].ParameterG = 0.02094;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[1].ParameterF = 0.00393;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[1].ParameterG = -0.05846;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[2].ParameterF = 0.21645;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[2].ParameterG = 0.25519;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[3].ParameterF = 0.05076;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[3].ParameterG = 0.07372;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[4].ParameterF = 0.03980;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[4].ParameterG = 0.06659;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[5].ParameterF = 0.03500;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[5].ParameterG = 0.00807;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[6].ParameterF = 0.00000;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[6].ParameterG = 0.00000;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[7].ParameterF = 0.01364;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[7].ParameterG = 0.01561;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[8].ParameterF = -0.03346;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[8].ParameterG = -0.04615;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[9].ParameterF = 0.00000;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[9].ParameterG = 0.00000;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[10].ParameterF = 0.17243;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[10].ParameterG = 0.20300;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[11].ParameterF = 0.03331;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[11].ParameterG = 0.03153;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[12].ParameterF = 0.00000;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[12].ParameterG = 0.00000;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[13].ParameterF = -0.04975;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[13].ParameterG = -0.11895;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[14].ParameterF = 0.19173;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[14].ParameterG = 0.27118;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[15].ParameterF = 0.05337;
    CaliData.Compensation[11].Model.PolynomialFunc.Coeff[15].ParameterG = 0.08010;
    // Modified for Every Model
    CaliData.luxBase = 0.548;    // Factory Calibration Factor, e.g. 1000/F
    CaliData.luxCode[0] = 4;     // CWF Threshold
    CaliData.luxCode[1] = 1;     // Slope CWF
    CaliData.luxCode[2] = 1;     // Slope CWF
    CaliData.luxCode[3] = 6.3;   // D65 Threshold
    CaliData.luxCode[4] = 0.478; // Slope D65
    CaliData.luxCode[5] = 1; // Slope D65
    CaliData.luxCode[6] = 9;     // A Threshold
    CaliData.luxCode[7] = 0.3;   // Slope A
    // Query algorithm version
    AlgoVer = STK_getAlgoVersion();
    STK_LOG("Algorithm Version: %08X", AlgoVer);
    // Query algorithm date
    AlgoDate = STK_getAlgoDate();
    STK_LOG("Algorithm Build Date: %d", AlgoDate);
    STK_initAlgo(&CaliData);
}
#endif

#ifdef STK_GPIO_ALS
static int32_t stk6b1x_als_fsm_pause(struct i2c_client *client, bool is_pause)
{
    int32_t ret = 0;
    uint8_t i2c_data = 0x0;
#ifdef STK_GPIO_ALS
    //uint8_t reg_data = 0;//, reg = 0;
#endif

    i2c_data = is_pause ? STK6B1X_FSM_ALS_PAUSE_MASK : 0;
    ret = sensor_write_reg_mask(client, STK6B1X_REG_FSM_CTRL, i2c_data, STK6B1X_FSM_ALS_PAUSE_MASK);

    if (ret < 0)
    {
        STK_LOG("read modify write i2c (0x%X) error\n", STK6B1X_REG_FSM_CTRL);
        return FAIL;
    }

#ifdef STK_GPIO_ALS

    if (!is_pause)
    {
        //reg_data = 0;
        ret = sensor_write_reg_mask(client, STK6B1X_REG_GPIO_SET1, 0, STK6B1X_GPIO_EN_MEASURE_MASK);
        if (ret < 0)
        {
            STK_LOG("read modify write i2c (0x%X) error\n", STK6B1X_REG_GPIO_SET1);
            return ret;
        }

        //reg_data = STK6B1X_GPIO_EN_MEASURE_MASK;
        ret = sensor_write_reg_mask(client, STK6B1X_REG_GPIO_SET1, STK6B1X_GPIO_EN_MEASURE_MASK, STK6B1X_GPIO_EN_MEASURE_MASK);
        if (ret < 0)
        {
            STK_LOG("read modify write i2c (0x%X) error\n", STK6B1X_REG_GPIO_SET1);
            return ret;
        }
    }

#endif
    return ret;
}

int32_t stk6b1x_als_set_gpio_ignore(struct i2c_client *client, uint16_t ignore_time)
{
    int32_t ret = 0;
    uint8_t tx_buf[3] = {0};//, reg_addr = 0;

    tx_buf[0] = STK6B1X_REG_ALS_WAIT1;
    tx_buf[1] = STK6B1X_H_BYTE(ignore_time) & 0x1F;
    tx_buf[2] = STK6B1X_L_BYTE(ignore_time);
    STK_LOG("ignore: %u us\n", ignore_time * 24);


    ret = sensor_tx_data(client, tx_buf, 3);
    if (ret < 0)
    {
        STK_LOG("fail, ret=%d\n", ret);
    }

    return ret;
}

int32_t stk6b1x_als_set_wait_time(struct i2c_client *client, uint16_t wait)
{
    int32_t ret = 0;
    uint8_t tx_buf[3] = {0};//, reg_addr = 0;

    tx_buf[0] = STK6B1X_H_BYTE(wait);
    tx_buf[1] = STK6B1X_H_BYTE(wait);
    tx_buf[2] = STK6B1X_L_BYTE(wait);
    STK_LOG("wait: %u us\n", wait * 21);

    ret = sensor_tx_data(client, tx_buf, 3);
    if (ret < 0)
    {
        STK_LOG("fail, ret=%d\n", ret);
    }

    return ret;
}

void stk6b1x_als_gpio_enable(struct i2c_client *client, bool enable)
{
    uint8_t i2c_flag_reg = enable ? STK6B1X_GPIO_ALS_SEL_MASK : 0;
    uint8_t reg_addr = 0;
    int32_t err = 0;

    if (stk6b1x_als.gpio_enable == enable)
    {
        STK_LOG("Already Set\n");
        return;
    }

    reg_addr = STK6B1X_REG_GPIO_SET0;
    err = sensor_write_reg_mask(client, reg_addr, i2c_flag_reg, STK6B1X_GPIO_ALS_SEL_MASK);
    if (err < 0)
    {
        STK_LOG("read modify write i2c (0x%X) error\n", STK6B1X_REG_GPIO_SET0);
        return;
    }

    stk6b1x_als.gpio_enable = enable;
}

void stk6b1x_gpio_lost_handle(struct i2c_client *client)
{
    int32_t ret = 0;
    uint8_t i2c_data[4] = {0}, reg_addr;
    uint8_t screen_hz = stk6b1x_als.display_freq, als_duty = 0;
    uint32_t measure_time = 0;
    uint32_t als_td = 0, target_timer = 0;
    uint32_t temp_target_timer = 0;
    uint32_t temp_als_td = 0;

    i2c_data[0] = STK6B1X_REG_GPIO_SET7;
    ret = sensor_rx_data(client, i2c_data, 3);

    if (ret < 0)
    {
        STK_LOG("read i2c measre time fail\n");
        return ;
    }

    measure_time = ((((i2c_data[1] & 0x1F) << 16) | (i2c_data[2] << 8) | i2c_data[3]) * 3) / 4;
    STK_LOG("Measure time:%dus(0x%X%X%X)\n", measure_time, i2c_data[1], i2c_data[2], i2c_data[3]);

    switch (measure_time / 1000)
    {
        case 20:
            screen_hz = 48;
            target_timer = 208333; //us
            als_td = 200; //us
            als_duty = 5; //without add 1
            break;

        case 16:
            screen_hz = 60;
            target_timer = 166666; //us
            als_td = 200; //us
            als_duty = 5; //without add 1
            break;

        case 11:
            screen_hz = 90;
            target_timer = 111111; //us
            als_td = 200;
            als_duty = 2;
            break;

        case 8:
            screen_hz = 120;
            target_timer = 83333; //us
            als_td = 200;
            als_duty = 1;
            break;

        case 6:
            screen_hz = 144;
            target_timer = 69444; //us
            als_td = 200;
            als_duty = 1;
            break;

        default:
            STK_LOG("need to implement\n");
            break;
    }

    if (screen_hz != stk6b1x_als.display_freq)
    {
        stk6b1x_als.display_freq = screen_hz;
        STK_LOG("Current is %dHz\n", stk6b1x_als.display_freq);
        temp_target_timer = (uint32_t)STK6B1X_GPIO_TD_TIMER(target_timer) / 10;
        i2c_data[0] = STK6B1X_REG_GPIO_SET14;
        i2c_data[1] = (temp_target_timer >> 16) & 0x1F;
        i2c_data[2] = (temp_target_timer >> 8)  & 0xFF;
        i2c_data[3] = (temp_target_timer >> 0)  & 0xFF;
        reg_addr = STK6B1X_REG_GPIO_SET14;
        ret = sensor_tx_data(client, i2c_data, 4);

        if (ret < 0)
        {
            STK_LOG("set TARGET TIMER fail\n");
        }

#ifdef STK_GPIO_ALS

        if (stk6b1x_als.gpio_enable)
        {
            stk6b1x_als_fsm_pause(client, false);
            temp_als_td = (uint32_t)STK6B1X_GPIO_TD_TIMER(als_td);
            i2c_data[0] = STK6B1X_REG_GPIO_SET20;
            i2c_data[1] = (temp_als_td >> 16) & 0x1F;
            i2c_data[2] = (temp_als_td >> 8)  & 0xFF;
            i2c_data[3] = (temp_als_td >> 0)  & 0xFF;
            //reg_addr = STK6B1X_REG_GPIO_SET20;
            ret = sensor_tx_data(client, i2c_data, 3);

            if (ret < 0)
            {
                STK_LOG("set ALS_TD fail\n");
            }

            i2c_data[0] = STK6B1X_REG_GPIO_SET27;
            i2c_data[0] = als_duty;
            //reg_addr = STK6B1X_REG_GPIO_SET27;
            ret = sensor_tx_data(client, i2c_data, 2);

            if (ret < 0)
            {
                STK_LOG("set ALS_DUTY fail\n");
                return;
            }

            stk6b1x_als_fsm_pause(client, false);
        }

#endif
    }
}
#endif

static int stk6b1x_sensor_check_id(struct i2c_client *client)
{
    int ret = FAIL;
    uint8_t pid_count = 0;
    uint8_t reg_addr = STK6B1X_REG_PID, reg_data = 0;

    reg_data = sensor_read_reg(client, reg_addr);

    for (pid_count = 0; pid_count < (sizeof(stk6b1x_pid_list) / sizeof(uint8_t)); pid_count++)
    {
        if ( reg_data == stk6b1x_pid_list[pid_count])
        {
            ret = NO_ERROR;
            break;
        }

        ret = FAIL;
    }

    if (ret != NO_ERROR)
        STK_LOG("light sensor check id failed!");
    else
        STK_LOG("light sensor check id successed!");

    return ret;
}

static int stk6b1x_sensor_hw_init(struct i2c_client *client)
{
    int i, ret = FAIL;

    for (i = 0; i < (sizeof(stk6b1x_als_default_register_table) / sizeof(stk6b1x_register_table)); i++)
    {
        ret = sensor_write_reg_mask(client, stk6b1x_als_default_register_table[i].address, stk6b1x_als_default_register_table[i].value, stk6b1x_als_default_register_table[i].mask);

        if (ret < 0)
        {
            STK_LOG("light init%d failed!", i);
            return FAIL;
        }
    }

    STK_LOG("init success!");
    return NO_ERROR;
}


static uint32_t stk_power(uint32_t base, uint32_t exp)
{
    uint32_t result = 1;

    while (exp)
    {
        if (exp & 1)
            result *= base;

        exp >>= 1;
        base *= base;
    }

    return result;
}

static void stk6b1x_reset_fifo_buffer(void)
{
    memset(stk6b1x_als.fifo_data0, 0x0, sizeof(stk6b1x_als.fifo_data0));
    memset(stk6b1x_als.fifo_data1, 0x0, sizeof(stk6b1x_als.fifo_data1));
    memset(stk6b1x_als.fifo_data2, 0x0, sizeof(stk6b1x_als.fifo_data2));
    memset(stk6b1x_als.fifo_data3, 0x0, sizeof(stk6b1x_als.fifo_data3));
    memset(stk6b1x_als.fifo_data4, 0x0, sizeof(stk6b1x_als.fifo_data4));
}

static void stk6b1x_fifo_init(struct i2c_client *client)
{
    uint8_t reg_value;

    reg_value = sensor_read_reg(client, STK6B1X_REG_FIFO_SET0);

    if (reg_value < 0)
    {
        STK_LOG("stk i2c failed\n");
        return ;
    }

    stk6b1x_als.data_type = (reg_value & STK6B1X_FIFO_SEL_MASK) >> STK6B1X_FIFO_SEL_SHIFT;
    stk6b1x_reset_fifo_buffer();

    switch (stk6b1x_als.data_type)
    {
        case STK6B1X_FIFO_SEL_ALS01234:
            stk6b1x_als.frame_byte = 10;
            break;

        case STK6B1X_FIFO_SEL_SALS0_SALS1_SALS2_SALS3_SALS4:
            stk6b1x_als.frame_byte = 16;
            break;

        default:
            stk6b1x_als.frame_byte = 0xFF;
            STK_LOG("ERROR!\n");
            break;
    }

    stk6b1x_als.read_frame = STK_FIFO_I2C_READ_FRAME;
    stk6b1x_als.target_frame_count = STK_FIFO_I2C_READ_FRAME_TARGET;
    stk6b1x_als.read_max_byte = stk6b1x_als.frame_byte * STK_FIFO_I2C_READ_FRAME;
    STK_LOG("target_frame_count = %d\n", stk6b1x_als.target_frame_count);
}

static void stk6b1x_fifo_enable(struct i2c_client *client, bool enabled)
{
    uint8_t ret = 0;
    uint8_t reg_value = 0;

    if ( stk6b1x_als.fifo_enable == enabled)
    {
        STK_LOG("fifo already set\n");
        return;
    }

    reg_value = sensor_read_reg(client, STK6B1X_REG_FIFO_SET0);

    if (enabled)
    {
#ifdef STK_ALS_IT1_SHORT
        reg_value |= STK6B1X_FIFO_MODE_STREAM;
#else
        reg_value |= STK6B1X_FIFO_MODE_BYPASS;
#endif
    }
    else
    {
        reg_value &= ~(0x03);
    }

    ret = sensor_write_reg(client, STK6B1X_REG_FIFO_SET0, reg_value);

    if (ret < 0)
    {
        STK_LOG("stk i2c failed\n");
        return;
    }

    ret = sensor_write_reg_mask(client, STK6B1X_REG_FIFO_SET4, STK6B1X_FIFO_EN_SRAM_PEW_MASK, STK6B1X_FIFO_EN_SRAM_PEW_MASK);

    if (ret < 0)
    {
        STK_LOG("stk i2c failed\n");
        return;
    }

    stk6b1x_als.fifo_enable = enabled;
}

static void stk6b1x_avg_data(void)
{
    uint16_t i;
    uint32_t data_sum[5] = {0};

    for (i = 0; i < stk6b1x_als.last_frame_count; i++)
    {
        data_sum[0] += stk6b1x_als.fifo_data0[i];

        if ((stk6b1x_als.data_type == STK6B1X_FIFO_SEL_ALS01234) || \
            (stk6b1x_als.data_type == STK6B1X_FIFO_SEL_SALS0_SALS1_SALS2_SALS3_SALS4) || \
            (stk6b1x_als.data_type == STK6B1X_FIFO_SEL_SALSR0_SALSR1_SALSR2_SALSR3_SALSR4))
        {
            data_sum[1] += stk6b1x_als.fifo_data1[i];
            data_sum[2] += stk6b1x_als.fifo_data2[i];
            data_sum[3] += stk6b1x_als.fifo_data3[i];
            data_sum[4] += stk6b1x_als.fifo_data4[i];
        }
    }

    if ((stk6b1x_als.data_type == STK6B1X_FIFO_SEL_ALS01234) || \
        (stk6b1x_als.data_type == STK6B1X_FIFO_SEL_SALS0_SALS1_SALS2_SALS3_SALS4) || \
        (stk6b1x_als.data_type == STK6B1X_FIFO_SEL_SALSR0_SALSR1_SALSR2_SALSR3_SALSR4))
    {
        stk6b1x_als.als_raw_data_u32[0] = (data_sum[0] / stk6b1x_als.last_frame_count);
        stk6b1x_als.als_raw_data_u32[1] = (data_sum[1] / stk6b1x_als.last_frame_count);
        stk6b1x_als.als_raw_data_u32[2] = (data_sum[2] / stk6b1x_als.last_frame_count);
        stk6b1x_als.als_raw_data_u32[3] = (data_sum[3] / stk6b1x_als.last_frame_count);
        stk6b1x_als.als_raw_data_u32[4] = (data_sum[4] / stk6b1x_als.last_frame_count);
    }

    return;
}

static void stk6b1x_get_fifo_data(struct i2c_client *client, uint16_t frame_num)
{
    int ret = 0;
    uint32_t read_bytes;
    uint8_t *raw_data = stk6b1x_als.raw_data;
    uint16_t i, offset, frame_count, read_frame_num;
    uint16_t chIdx = 0;
    uint32_t dg_ratio[5] = {0}, ag_ratio[5] = {0};

    stk6b1x_reset_fifo_buffer();

    for (frame_count = 0 ; frame_count < frame_num ; frame_count += (stk6b1x_als.read_frame))
    {
        read_frame_num = (int16_t)(frame_num - frame_count);

        if (read_frame_num >= stk6b1x_als.read_frame)
        {
            read_bytes = stk6b1x_als.read_max_byte;
            read_frame_num = stk6b1x_als.read_frame;
        }
        else
        {
            read_bytes = stk6b1x_als.frame_byte * read_frame_num;
        }

        memset(raw_data, 0, STK_FIFO_I2C_READ_BYTE);
        raw_data[0] = STK6B1X_REG_ALS_FIFO_OUT;
        ret = sensor_rx_data(client, raw_data, read_bytes);

        if (ret < 0)
        {
            STK_LOG("stk6b1x_get_fifo_data failed\n");
            return;
        }

        switch (stk6b1x_als.data_type)
        {
            case STK6B1X_FIFO_SEL_ALS01234:
                for (i = 0, offset = 0; i < read_frame_num; i++, offset += stk6b1x_als.frame_byte)
                {
                    stk6b1x_als.fifo_data0[frame_count + i] = ((raw_data[offset] << 8) | raw_data[offset + 1]);
                    stk6b1x_als.fifo_data1[frame_count + i] = ((raw_data[offset + 2] << 8) | raw_data[offset + 3]);
                    stk6b1x_als.fifo_data2[frame_count + i] = ((raw_data[offset + 4] << 8) | raw_data[offset + 5]);
                    stk6b1x_als.fifo_data3[frame_count + i] = ((raw_data[offset + 6] << 8) | raw_data[offset + 7]);
                    stk6b1x_als.fifo_data4[frame_count + i] = ((raw_data[offset + 8] << 8) | raw_data[offset + 9]);
                }

                break;
#ifdef STK_ALS_HAGC

            case STK6B1X_FIFO_SEL_SALS0_SALS1_SALS2_SALS3_SALS4:
                for (i = 0, offset = 0; i < read_frame_num; i++, offset += stk6b1x_als.frame_byte)
                {
                    for (chIdx = 0; chIdx < 5; chIdx ++)
                    {
                        dg_ratio[chIdx] = stk_power(2, STK6B1X_ALS_GET_STA_DG(raw_data[offset + 3 * chIdx]));
                        ag_ratio[chIdx] = 1 << (4 - (STK6B1X_ALS_GET_STA_AG(raw_data[offset + 3 * chIdx])));
                    }

                    stk6b1x_als.fifo_data0[frame_count + i] = ((raw_data[offset + 1] << 8) | raw_data[offset + 2]);
                    stk6b1x_als.fifo_data1[frame_count + i] = ((raw_data[offset + 4] << 8) | raw_data[offset + 5]);
                    stk6b1x_als.fifo_data2[frame_count + i] = ((raw_data[offset + 7] << 8) | raw_data[offset + 8]);
                    stk6b1x_als.fifo_data3[frame_count + i] = ((raw_data[offset + 10] << 8) | raw_data[offset + 11]);
                    stk6b1x_als.fifo_data4[frame_count + i] = ((raw_data[offset + 13] << 8) | raw_data[offset + 14]);
                    stk6b1x_als.fifo_data0[frame_count + i] *= STK6B1X_GET_GAIN_RATIO(dg_ratio[0], ag_ratio[0]);
                    stk6b1x_als.fifo_data1[frame_count + i] *= STK6B1X_GET_GAIN_RATIO(dg_ratio[1], ag_ratio[1]);
                    stk6b1x_als.fifo_data2[frame_count + i] *= STK6B1X_GET_GAIN_RATIO(dg_ratio[2], ag_ratio[2]);
                    stk6b1x_als.fifo_data3[frame_count + i] *= STK6B1X_GET_GAIN_RATIO(dg_ratio[3], ag_ratio[3]);
                    stk6b1x_als.fifo_data4[frame_count + i] *= STK6B1X_GET_GAIN_RATIO(dg_ratio[4], ag_ratio[4]);
                }

                break;
#endif

            default:
                STK_LOG("unavailable!\n");
                break;
        }
    }

    stk6b1x_als.last_frame_count = frame_num;
    // STK_LOG("para=%d, %d, %d, %d, %d",
    //                 stk6b1x_als->read_max_byte,
    //                 stk6b1x_als->read_frame,
    //                 stk6b1x_als->frame_byte,
    //                 read_bytes,
    //                 frame_num);
}

static int stk6b1x_get_fifo_data_polling(struct i2c_client *client)
{
    int ret = 0;
    char buf[3] = {0};
    uint16_t frame_num;
    stk6b1x_als.fifo_is_ready = false;
    buf[0] = STK6B1X_REG_ALS_FIFO_FLAG;
    buf[1] = sensor_read_reg(client, buf[0]);

    if (buf[0] < 0)
    {
        STK_LOG("stk i2c failed\n");
        return buf[0];
    }

    buf[0] = STK6B1X_REG_ALS_FIFO_CNT1;
    ret = sensor_rx_data(client, buf, 2);

    frame_num = ((buf[0] << 8) & 0x03) | buf[1];
    STK_LOG("frame_num = %d\n", frame_num);

    if (frame_num >= STK_FIFO_I2C_READ_FRAME_TARGET)
    {
        frame_num = STK_FIFO_I2C_READ_FRAME_TARGET;
    }

    stk6b1x_get_fifo_data(client, frame_num);

    if (stk6b1x_als.last_frame_count != 0)
    {
        stk6b1x_avg_data();
    }

    stk6b1x_als.fifo_is_ready = true;
    return ret;
}

static int stk6b1x_sensor_init(struct i2c_client *client)
{
    int ret = FAIL;

    ret = stk6b1x_sensor_check_id(client);

    if (ret < NO_ERROR)
    {
        STK_LOG("light check id error ret = %d!", ret);
        return 0;
    }

    //sw reset
    sensor_write_reg(client, STK6B1X_REG_SWRST, STK_STK6B1X_SWRESET);
    //tx_thread_sleep(15);
    usleep_range(13000, 15000);
    ret = stk6b1x_sensor_hw_init(client);

    if (ret < NO_ERROR)
    {
        STK_LOG("light hw init error ret = %d!", ret);
        return 0;
    }

    stk6b1x_fifo_init(client);
    return 0;
}

/*
static int stk6b1x_sensor_set_cali_data(void *cali_data)
{
    return NO_ERROR;
}

static int stk6b1x_sensor_rate(int32_t sampling_period_us)
{
    return NO_ERROR;
}
*/
static int stk6b1x_sensor_enable(struct i2c_client *client, int enable, int rate)
{
    int ret = FAIL;
    uint8_t buf[2] = {STK6B1X_REG_ENABLE, 0x00};

    buf[1] = sensor_read_reg(client, buf[0]);

    if(enable){
        buf[1] |= (STK6B1X_STATE_EN_ALS_WAIT_MASK | STK6B1X_STATE_EN_ALS_MASK);
        if (enable_state != 1)
        {
            ret = sensor_write_reg_mask(client, buf[0], buf[1], 0xFF);
            //tx_thread_sleep(10);

            if (ret >= 0)
            {
                enable_state = 1;
                ret = NO_ERROR;
                STK_LOG("light enable sensor successed!\n");
            }
            else
            {
                STK_LOG("light enable sensor failed! error! ret=%d\n", ret);
                return ret;
            }
#ifdef STK_GPIO_ALS
            stk6b1x_als_gpio_enable(client, true);
#endif
        }
        else
        {
            ret = NO_ERROR;
            STK_LOG("already enabled!");
        }

#ifdef STK_ALGO_ENABLE
        stk_algo_init();
#endif
        stk6b1x_als.last_frame_count = 0;
        stk6b1x_fifo_enable(client, true);
#ifdef STK6B1X_RGB_ENABLE
        stk6b1x_als.ch_scale[R_CALI] = 1000;
        stk6b1x_als.ch_scale[G_CALI] = 1000;
        stk6b1x_als.ch_scale[B_CALI] = 1000;
        stk6b1x_als.ch_scale[C_CALI] = 1000;
#endif
        reject_frames_num = 0;
        }else{
            buf[1] &= (~(STK6B1X_STATE_EN_ALS_WAIT_MASK | STK6B1X_STATE_EN_ALS_MASK));
            if (enable_state != 0)
            {
                ret = sensor_write_reg_mask(client, buf[0], buf[1], 0xFF);
        
                if (ret >= 0)
                {
                    enable_state = 0;
                    ret = NO_ERROR;
                    STK_LOG("light disable sensor successed!\n");
                }
                else
                {
                    STK_LOG("light disable sensor failed! error! ret=%d\n", ret);
                    return ret;
                }
#ifdef STK_GPIO_ALS
                stk6b1x_als_gpio_enable(client, false);
#endif
            }
            else
            {
                ret = NO_ERROR;
                STK_LOG("already disabled!");
            }
        
#ifdef STK_ALGO_ENABLE
            STK_deInitAlgo();
#endif
            stk6b1x_fifo_enable(client, false);
            last_luxdata = -1.0f;

        }
    return ret;
}
/*
static int stk6b1x_sensor_disable(struct i2c_client *client)
{
    int ret = FAIL;
    uint8_t buf[2] = {STK6B1X_REG_ENABLE, 0x00};
    buf[1] = sensor_read_reg(client, buf[0]);
    buf[1] &= (~(STK6B1X_STATE_EN_ALS_WAIT_MASK | STK6B1X_STATE_EN_ALS_MASK));

    if (enable_state != 0)
    {
        ret = sensor_write_reg_mask(client, buf[0], buf[1], 0xFF);

        if (ret >= 0)
        {
            enable_state = 0;
            ret = NO_ERROR;
            STK_LOG("light disable sensor successed!\n");
        }
        else
        {
            STK_LOG("light disable sensor failed! error! ret=%d\n", ret);
            return ret;
        }
#ifdef STK_GPIO_ALS
        stk6b1x_als_gpio_enable(false);
#endif
    }
    else
    {
        ret = NO_ERROR;
        STK_LOG("already disabled!");
    }

#if STK_ALGO_ENABLE
    STK_deInitAlgo();
#endif
    stk6b1x_fifo_enable(false);
    last_luxdata = -1.0f;
    return ret;
}

static int stk6b1x_sensor_set_status()
{
    return NO_ERROR;
}

static int stk6b1x_sensor_get_status()
{
    //uint8_t reg_addr = STK6B1X_REG_ALS_FLAG;
    //uint8_t reg_data = STK6B1X_FLG_ALS_DR_MASK;
    return NO_ERROR;
}

static void stk6b1x_sensor_cali_cmd_handle(int cal_cmd, int cali_type, int golden_sample)
{
    return NO_ERROR;
    //light_sensor_cali_cmd_handle(cal_cmd, cali_type, golden_sample);
}

static void stk6b1x_sensor_get_cali_data()
{
    return NO_ERROR;
    //light_sensor_get_cali_data();
}

static int stk6b1x_sensor_calibration(float *raw_data)
{
    (void)raw_data;
    return NO_ERROR;
}
*/
#ifdef STK6B1X_RGB_ENABLE
static int stk6b1x_als_rgb_cal(void)
{
    int8_t ret;
    float *lux_coef_matrix;
    float r_g, b_g, c_g, lux = 0;
    uint32_t normalization_data[STK6B1X_CH_CNT];
    uint16_t i = 0;
#ifdef STK_ALS_HAGC
    uint16_t j = 0;
    uint32_t u32_temp[STK6B1X_CH_CNT] = {0};
#endif
#ifdef STK6B1X_RGB_ENABLE
    uint8_t tab_index;
    uint16_t table_cnt = sizeof(stk6b1x_cluster_mean_table) / sizeof(stk6b1x_cluster_mean);
    float *cct_coef_matrix;
    float XX = 0, YY = 0, ZZ = 0, sum = 0, temp = 0;
    double dis_temp, mean_distance[2];// mean_distance[table_cnt];
    float matrix[CCTR_COEF_ROW] = {0};
#endif

    if (stk6b1x_als.als_is_ready)
    {
        for (i = 0; i < STK6B1X_CH_CNT; i++)
        {
            normalization_data[i] = stk6b1x_als.als_raw_data_u32[i];
            // STK_LOG("ch%d = %u\n", i, normalization_data[i]);
        }

        r_g = (float)(stk6b1x_als.als_last_raw_data[0] / stk6b1x_als.als_last_raw_data[1]);
        b_g = (float)(stk6b1x_als.als_last_raw_data[2] / stk6b1x_als.als_last_raw_data[1]);
        c_g = (float)(stk6b1x_als.als_last_raw_data[4] / stk6b1x_als.als_last_raw_data[1]);
        normalization_data[0] *= (stk6b1x_als.ch_scale[R_CALI] / 1000.0);
        normalization_data[1] *= (stk6b1x_als.ch_scale[G_CALI] / 1000.0);
        normalization_data[2] *= (stk6b1x_als.ch_scale[B_CALI] / 1000.0);
        normalization_data[3] *= (stk6b1x_als.ch_scale[C_CALI] / 1000.0);
        stk6b1x_als.rgb_sample.r = normalization_data[0];
        stk6b1x_als.rgb_sample.g = normalization_data[1];
        stk6b1x_als.rgb_sample.b = normalization_data[2];
        stk6b1x_als.rgb_sample.c = normalization_data[3];
        STK_LOG("R = %f, G = %f, B = %f, C = %f\n", stk6b1x_als.rgb_sample.r, stk6b1x_als.rgb_sample.g,
                        stk6b1x_als.rgb_sample.b, stk6b1x_als.rgb_sample.c);

        if (normalization_data[1] != 0)
            stk6b1x_als.rgb_sample.ir_ratio = (float)normalization_data[3] / (float)normalization_data[1];
        else
            stk6b1x_als.rgb_sample.ir_ratio = 0;

#ifdef STK6B1X_RGB_ENABLE

        for (i = 0; i < table_cnt; i++)
        {
            mean_distance[i] = stk_power(stk6b1x_cluster_mean_table[i].mean_r - r_g, 2) +
                               stk_power(stk6b1x_cluster_mean_table[i].mean_b - b_g, 2) +
                               stk_power(stk6b1x_cluster_mean_table[i].mean_c - c_g, 2);
            //STK_LOG("mean_distance[%d] = %f\n", i, mean_distance[i]);
        }

        dis_temp = mean_distance[0];
        tab_index = 0;

        for (i = 1; i < table_cnt; i++)
        {
            if (dis_temp > mean_distance[i])
            {
                dis_temp = mean_distance[i];
                tab_index = i;
            }
        }

        cct_coef_matrix = stk6b1x_cct_cluster_table[tab_index].cct_coef;

        //STK_LOG("tab_index = %d\n", tab_index);
        for (i = 0; i < CCTR_COEF_ROW; i++, cct_coef_matrix += 4)
        {
            matrix[i] = *cct_coef_matrix * r_g +
                        *(cct_coef_matrix + 1) +
                        *(cct_coef_matrix + 2) * b_g +
                        *(cct_coef_matrix + 3) * c_g;
        }

        //calculate X/Y/Z
        XX = matrix[0];
        YY = matrix[1];
        ZZ = matrix[2];
        //calculate x/y and rear_cct
        sum = XX + YY + ZZ;
        stk6b1x_als.rgb_sample.x = XX / sum;
        stk6b1x_als.rgb_sample.y = YY / sum;
        temp = (stk6b1x_als.rgb_sample.x - 0.332) / (stk6b1x_als.rgb_sample.y - 0.1858);
        stk6b1x_als.rgb_sample.cct = -499.0 * temp * temp * temp + 3525.0 * temp * temp - 6823.3 * temp + 5520.33;
#endif

        //calculate lux
        if ((c_g * 1000) > 50)
        {
            lux_coef_matrix = lux_coef_h;
        }
        else
        {
            lux_coef_matrix = lux_coef_l;
        }

        for (i = 0; i < RAW_NUM; i++)
        {
            lux += lux_coef_matrix[i] * normalization_data[i];
        }

        /* Reserve one digit after the decimal point*/
        stk6b1x_als.rgb_sample.lux = (float)((uint32_t)((lux + 0.05) * 10)) / 10;
    }

    STK_LOG("lux = %d, cct = %f, ir_ratio = %f\n", (int)stk6b1x_als.rgb_sample.lux, stk6b1x_als.rgb_sample.cct, stk6b1x_als.rgb_sample.ir_ratio);
    //STK_LOG("x = %f, y = %f\n", stk6b1x_als.rgb_sample.x, stk6b1x_als.rgb_sample.y);
}
#endif

static int stk6b1x_light_report_abs_value(struct input_dev *input, int data)
{
    input_report_abs(input, ABS_MISC, data);
    input_sync(input);
    return data;
}


static int stk6b1x_sensor_get_data(struct i2c_client *client)
{
    struct sensor_private_data *sensor =
    (struct sensor_private_data *) i2c_get_clientdata(client);
    int ret = 0;
    uint8_t i;
    uint32_t luxdata_flt;
//    struct sensor_event sensor_event_data;
#if STK_ALGO_ENABLE
    PixelData DisplayData; //input display info
    ChannelData RawData;
    ChannelData AmbientData;
    ChannelData DisplayNoiseData;
#endif
    // ret = stk6b1x_sensor_get_status();
    // if(ret < 0) {
    //     STK_LOG("light data no ready! ret = %d\r\n", ret);
    //     return NO_DATA;
    // }
    stk6b1x_get_fifo_data_polling(client);

    if (!stk6b1x_als.fifo_is_ready)
        return 0;

    stk6b1x_als.als_is_ready = stk6b1x_als.fifo_is_ready;

    for (i = 0; i < 5; i++)
    {
        stk6b1x_als.als_last_raw_data[i] = stk6b1x_als.als_raw_data_u32[i];
    }

    STK_LOG("R_raw = %d, G_raw = %d, , B_raw = %d, IR_raw = %d, C_raw = %d\n",
                    stk6b1x_als.als_last_raw_data[0], stk6b1x_als.als_last_raw_data[1],
                    stk6b1x_als.als_last_raw_data[2], stk6b1x_als.als_last_raw_data[3],
                    stk6b1x_als.als_last_raw_data[4]);
#ifdef STK3B6X_RGB_ENABLE
    stk6b1x_als_rgb_cal();
    luxdata_flt = stk6b1x_als.rgb_sample.lux;
#else
    luxdata_flt = stk6b1x_als.als_raw_data_u32[3]; //return F channel
#endif
#if STK_ALGO_ENABLE
    DisplayData.PixelR = stk6b1x_als.pixeldata.PixelR ;
    DisplayData.PixelG = stk6b1x_als.pixeldata.PixelG ;
    DisplayData.PixelB = stk6b1x_als.pixeldata.PixelB ;
    DisplayData.Brightness = stk6b1x_als.pixeldata.Brightness;
    memset(&DisplayNoiseData, 0, sizeof(ChannelData));
    STK_calcDisplayNoise(&DisplayNoiseData, &DisplayData);
    //STK_LOG("DisplayNoiseData.ChannelG : %d\n", DisplayNoiseData.ChannelG);
    memset(&AmbientData, 0, sizeof(ChannelData));
    /*
    STK_LOG("raw ALS: %d, G:%d , C:%d, gain =%d , G * gain =%d\n", \
    stk6b1x_als.als_raw_data[0], stk6b1x_als.als_raw_data[1], stk6b1x_als.als_raw_data[2], gain, \
    stk6b1x_als.als_raw_data_u32[1]);
    */
    RawData.ChannelF = stk6b1x_als.als_raw_data_u32[3]; // Channel F
    RawData.ChannelG = stk6b1x_als.als_raw_data_u32[1]; // CHannel G
    STK_calcAmbientInfo(&RawData, &AmbientData, &DisplayData, false);
    luxdata_flt = (float)AmbientData.ChannelG;
    STK_LOG("Ambient ALS: %u, C1: %u, lux =%d, als_scale : %f, calibrated =%d\n", \
                    AmbientData.ChannelF, AmbientData.ChannelG, luxdata_flt,  \
                    stk6b1x_als.als_scale, stk6b1x_als.calibrated);
#endif
#ifdef STK_GPIO_ALS
    stk6b1x_gpio_lost_handle(client);
#endif

    ret = stk6b1x_light_report_abs_value(sensor->input_dev, luxdata_flt);

    return ret;
}

/*
static int stk6b1x_sensor_set_mode(int mode)
{
    (void)mode;
    return NO_ERROR;
}

static int stk6b1x_sensor_get_fifo_data(struct sensor_data *sensor_data)
{
    (void)sensor_data;
    return NO_ERROR;
}

static int stk6b1x_sensor_flush(int sensor)
{
    (void)sensor;
    return NO_ERROR;
}

static int stk6b1x_sensor_selftest()
{
    return NO_ERROR;
}
*/

struct sensor_operate stk6b1x_ops = {
    .name                = "ls_stk6b1x",
    .type                = SENSOR_TYPE_LIGHT,    //sensor type and it should be correct
    .id_i2c              = LIGHT_ID_STK6B1X,        //i2c id number
    .read_reg            = STK6B1X_REG_ALS_FIFO_OUT,            //read data
    .read_len            = 2,                //data length
    .id_reg              = 0x3E,//SENSOR_UNKNOW_DATA,        //read device id from this register
    .id_data             = STK6B1X_PID,//SENSOR_UNKNOW_DATA,        //device id
    .precision           = 16,                //16 bits
    .ctrl_reg            = STK6B1X_REG_ALS_FLAG,            //enable or disable 
    .int_status_reg      = 0x00,            //intterupt status register
    .range               = {2,65535},        //range
    .brightness          = {5,255},     //brightness    
    .trig                = IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,        
    .active              = stk6b1x_sensor_enable,    
    .init                = stk6b1x_sensor_init,
    .report              = stk6b1x_sensor_get_data,
};

static struct sensor_operate *stk6b1x_light_get_ops(void)
{
    return &stk6b1x_ops;
}

static int __init stk6b1x_init(void)
{
    struct sensor_operate *ops = stk6b1x_light_get_ops();
    int result = 0;
    int type = ops->type;
    result = sensor_register_slave(type, NULL, NULL, stk6b1x_light_get_ops);
    return result;
}

static void __exit stk6b1x_exit(void)
{
    struct sensor_operate *ops = stk6b1x_light_get_ops();
    int type = ops->type;
    sensor_unregister_slave(type, NULL, NULL, stk6b1x_light_get_ops);
}


module_init(stk6b1x_init);
module_exit(stk6b1x_exit);
MODULE_AUTHOR("David <David@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk6b1x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION_STK6B1X);

