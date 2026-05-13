//#include <asm/neon.h>//for float?
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
#include <asm/neon.h>//for float?

#include "ls_stk6bcx.h"
#include "stk6bcx_light_ver.h"

uint8_t stk6bcx_pid_list[] = {0x11, 0x12};

static float last_luxdata = -1.0f;
static uint8_t enable_state;
static uint8_t reject_frames_num;

#ifdef STK_ALGO_ENABLE
static stk6bcx_als_fac_cali_data als_cali_fgcc_data;
static struct stk6bcx_als_fac_cali_data_type als_cali_fgcc;
#endif

struct stk6bcx_data stk6bcx_als =
{
    0,     /* als count */
    0,     /* als enable */
    1.0,   /*r_scale*/
    1.0,   /*g_scale*/
    1.0,   /*b_scale*/
    1.0,   /*w_scale*/
};

stk6bcx_register_table stk6bcx_als_default_register_table[] =
{
    {STK6BCX_REG_ENABLE,           STK6BCX_STATE_EN_PWR_MASK,                                   STK6BCX_STATE_EN_PWR_MASK},
    //ALS
    {
        STK6BCX_REG_ALS_DGAIN,
        (STK6BCX_ALS_GAIN256 << STK6BCX_ALS0_GAIN_SHIFT) | (STK6BCX_ALS_GAIN256 << STK6BCX_ALS1_GAIN_SHIFT),
        STK6BCX_ALS0_GAIN_MASK | STK6BCX_ALS1_GAIN_MASK
    },
    {
        STK6BCX_REG_ALS_DGAIN1,
        (STK6BCX_ALS_GAIN256 << STK6BCX_ALS2_GAIN_SHIFT) | (STK6BCX_ALS_GAIN256 << STK6BCX_ALS3_GAIN_SHIFT),
        STK6BCX_ALS2_GAIN_MASK | STK6BCX_ALS3_GAIN_MASK
    },
    {
        STK6BCX_REG_ALS_DGAIN2,
        (STK6BCX_ALS_GAIN256 << STK6BCX_ALS4_GAIN_SHIFT) | (STK6BCX_ALS_GAIN256 << STK6BCX_ALS5_GAIN_SHIFT),
        STK6BCX_ALS4_GAIN_MASK | STK6BCX_ALS5_GAIN_MASK
    },
    {
        STK6BCX_REG_ALS_AGAIN,
        (STK6BCX_ALS_CI_4 << STK6BCX_ALS0_CI_SHIFT) | (STK6BCX_ALS_CI_4 << STK6BCX_ALS1_CI_SHIFT),
        STK6BCX_ALS0_CI_MAKS | STK6BCX_ALS1_CI_MAKS
    },
    {
        STK6BCX_REG_ALS_AGAIN1,
        (STK6BCX_ALS_CI_4 << STK6BCX_ALS2_CI_SHIFT) | (STK6BCX_ALS_CI_4 << STK6BCX_ALS3_CI_SHIFT),
        STK6BCX_ALS2_CI_MAKS | STK6BCX_ALS3_CI_MAKS
    },
    {
        STK6BCX_REG_ALS_AGAIN2,
        (STK6BCX_ALS_CI_4 << STK6BCX_ALS4_CI_SHIFT) | (STK6BCX_ALS_CI_4 << STK6BCX_ALS5_CI_SHIFT),
        STK6BCX_ALS4_CI_MAKS | STK6BCX_ALS5_CI_MAKS
    },
    {STK6BCX_REG_ALS_IT_SET0,       STK6BCX_ALS_IT_BASE_SEL,                                      STK6BCX_ALS_IT_SEL_MASK},
#ifdef STK_FIFO_ENABLE
    {STK6BCX_REG_ALS_IT_SET1,       STK6BCX_ALPS_REG_H(STK6BCX_ALS_IT_50US, 0xFF),                0xFF},
    {STK6BCX_REG_ALS_IT_SET2,       STK6BCX_ALPS_REG_L(STK6BCX_ALS_IT_50US),                      0xFF},
#ifdef STK_FLK_ENABLE
    {STK6BCX_REG_NPST_SET1,         STK6BCX_ALPS_REG_H(0x410, 0x0F),                              0x0F},
    {STK6BCX_REG_NPST_SET2,         STK6BCX_ALPS_REG_L(0x410),                                    0xFF},
    {STK6BCX_REG_FIFO_FLK_SET0,     STK6BCX_FIFO_MODE_STREAM,                                     STK6BCX_FIFO_MODE_MASK},
#ifdef CONFIG_STK_FLK_DRI
    {STK6BCX_REG_FIFO_FLK_SET6,     STK6BCX_FIFO_FOVR_EN | STK6BCX_FIFO_FWM_EN | STK6BCX_FIFO_FFULL_EN, STK6BCX_ALS_IT_SEL_MASK},
#endif
#endif
#else
    {STK6BCX_REG_ALS_IT_SET1,       STK6BCX_ALPS_REG_H(STK6BCX_ALS_IT_10MS, 0xFF),                0xFF},
    {STK6BCX_REG_ALS_IT_SET2,       STK6BCX_ALPS_REG_L(STK6BCX_ALS_IT_10MS),                      0xFF},
#endif
    {STK6BCX_REG_ALS_WAIT1,         STK6BCX_ALPS_REG_H(0x5, 0xFF),                     0xFF},
    {STK6BCX_REG_ALS_WAIT2,         STK6BCX_ALPS_REG_L(0x5),                           0xFF},
    {STK6BCX_REG_FIFO_SET0,         STK6BCX_FIFO_DATA_SEL_STA012345_ALS012345,                    STK6BCX_FIFO_DATA_SEL_MASK},
    {0xB1,                          0xBF,                                                         0xBF},
};

stk6bcx_register_table stk6bcx_default_als_thd_table[] =
{
    {STK6BCX_REG_ALS_THDH1, 0x00, 0xFF},
    {STK6BCX_REG_ALS_THDH2, 0x00, 0xFF},
    {STK6BCX_REG_ALS_THDL1, 0xFF, 0xFF},
    {STK6BCX_REG_ALS_THDL2, 0xFF, 0xFF},
};

#ifdef STK_ALGO_ENABLE
/*The function use to get display RGB value and brightness sample, must implemented by customer*/
int get_underscreen_als_data(void *pata, int len)
{
    struct under_screen_als_info als_info;
    memcpy(&als_info, pata, sizeof(struct under_screen_als_info));

    if (als_info.send_flag == 1) // update RGB,brightness
    {
        stk6bcx_als.pixeldata.PixelR = (als_info.display_info_t.pixel_rgb & 0xFF0000) >> 16; //Current display R value
        stk6bcx_als.pixeldata.PixelG = (als_info.display_info_t.pixel_rgb & 0x00FF00) >> 8;  //Current display G value
        stk6bcx_als.pixeldata.PixelB = (als_info.display_info_t.pixel_rgb & 0x0000FF);       //Current display B value
        stk6bcx_als.pixeldata.Brightness = als_info.display_info_t.brightness;; //Current display brightness
        STK_LOG("[lib_use]brightness= %d, pixel_r =%d ,g = %d, b = %d", stk6bcx_als.pixeldata.Brightness,
                        stk6bcx_als.pixeldata.PixelR, stk6bcx_als.pixeldata.PixelG, stk6bcx_als.pixeldata.PixelB);
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
    stk6bcx_als.calibrated = 0;

    for (uint8_t i = 0; i < MAX_BACKGROUND_NUM - 1; i++)
    {
        stk6bcx_als.ChannelData[i].ChannelF = 0;
        stk6bcx_als.ChannelData[i].ChannelG = als_cali_fgcc_data.fac_cali_G_data[i] \
                                            * als_cali_fgcc_data.fac_cali_other_data[i];//Each picture value;
        STK_LOG("stk6bcx_als.ChannelData[%d].ChannelG=%d\n", i, stk6bcx_als.ChannelData[i].ChannelG);
    }

    if ( als_cali_fgcc_data.fac_cali_G_data[0] != 0 && als_cali_fgcc_data.fac_cali_G_data[1] != 0 &&
         als_cali_fgcc_data.fac_cali_G_data[2] != 0 && als_cali_fgcc_data.fac_cali_G_data[3] != 0 &&
         als_cali_fgcc_data.fac_cali_G_data[4] != 0 && als_cali_fgcc_data.fac_cali_G_data[5] != 0 &&
         als_cali_fgcc_data.fac_cali_G_data[6] != 0 && als_cali_fgcc_data.fac_cali_G_data[7] != 0 &&
         als_cali_fgcc_data.fac_cali_G_data[8] != 0 && als_cali_fgcc_data.fac_cali_G_data[9] != 0 )
    {
        stk6bcx_als.calibrated = 1;
        stk6bcx_als.als_scale = (float)(TARGET_LUX / (als_cali_fgcc_data.fac_cali_G_data[0]));
        memset(&CaliData, 0x0, sizeof(AlgoParam));
        // Calibration for Gamma Point
        CaliData.RGBGamma[0].Level = 64;
        STK_calcRGBGamma(64, &stk6bcx_als.ChannelData[4], 255, &stk6bcx_als.ChannelData[1], &TempGamma);
        CaliData.RGBGamma[0].R.ParameterF = TempGamma.ParameterF;
        CaliData.RGBGamma[0].R.ParameterG = TempGamma.ParameterG;
        STK_calcRGBGamma(64, &stk6bcx_als.ChannelData[5], 255, &stk6bcx_als.ChannelData[2], &TempGamma);
        CaliData.RGBGamma[0].G.ParameterF = TempGamma.ParameterF;
        CaliData.RGBGamma[0].G.ParameterG = TempGamma.ParameterG;
        STK_calcRGBGamma(64, &stk6bcx_als.ChannelData[6], 255, &stk6bcx_als.ChannelData[3], &TempGamma);
        CaliData.RGBGamma[0].B.ParameterF = TempGamma.ParameterF;
        CaliData.RGBGamma[0].B.ParameterG = TempGamma.ParameterG;
        // CaliData.BrightnessGamma[0] for Red Brightness
        CaliData.BrightnessGamma[0].Level = 3212;
        STK_calcBrightnessGamma(3212, &stk6bcx_als.ChannelData[7], 4095, &stk6bcx_als.ChannelData[1], &TempGamma);
        CaliData.BrightnessGamma[0].R.ParameterF = TempGamma.ParameterF;
        CaliData.BrightnessGamma[0].R.ParameterG = TempGamma.ParameterG;
        // CaliData.BrightnessGamma[1] for Green Brightness
        STK_calcBrightnessGamma(3212, &stk6bcx_als.ChannelData[8], 4095, &stk6bcx_als.ChannelData[2], &TempGamma);
        CaliData.BrightnessGamma[0].G.ParameterF = TempGamma.ParameterF;
        CaliData.BrightnessGamma[0].G.ParameterG = TempGamma.ParameterG;
        // CaliData.BrightnessGamma[2] for Blue Brightness
        STK_calcBrightnessGamma(3212, &stk6bcx_als.ChannelData[9], 4095, &stk6bcx_als.ChannelData[3], &TempGamma);
        CaliData.BrightnessGamma[0].B.ParameterF = TempGamma.ParameterF;
        CaliData.BrightnessGamma[0].B.ParameterG = TempGamma.ParameterG;
        // Modified for Every Device
        // R255, G0, B0, Maximum Brightness Screen -1 SensorData[1].channelF channelG
        CaliData.RGB255Data.R.ChannelF = stk6bcx_als.ChannelData[1].ChannelF ;
        CaliData.RGB255Data.R.ChannelG = stk6bcx_als.ChannelData[1].ChannelG;
        // R0, G255, B0, Maximum Brightness Screen -2  SensorData[2].channelF channelG
        CaliData.RGB255Data.G.ChannelF = stk6bcx_als.ChannelData[2].ChannelF;
        CaliData.RGB255Data.G.ChannelG = stk6bcx_als.ChannelData[2].ChannelG;
        // R0, G0, B255, Maximum Brightness Screen -3
        CaliData.RGB255Data.B.ChannelF = stk6bcx_als.ChannelData[3].ChannelF;
        CaliData.RGB255Data.B.ChannelG = stk6bcx_als.ChannelData[3].ChannelG;
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

void stk6bcx_dump_reg(struct i2c_client *client)
{
    int ret;
    uint8_t stk6bcx_reg_map[] =
    {
        STK6BCX_REG_ENABLE,
        STK6BCX_REG_FSM_CTRL,
            0x03,
            0x04,
            0x05,
            0x06,
            0x07,
            0x08,
            0x09,
        STK6BCX_REG_ALS_DGAIN,
        STK6BCX_REG_ALS_DGAIN1,
        STK6BCX_REG_ALS_DGAIN2,
        STK6BCX_REG_ALS_AGAIN,
        STK6BCX_REG_ALS_AGAIN1,
        STK6BCX_REG_ALS_AGAIN2,
        STK6BCX_REG_ALS_IT_SET0,
        STK6BCX_REG_ALS_IT_SET1,
        STK6BCX_REG_ALS_IT_SET2,
        STK6BCX_REG_ALS_WAIT1,
        STK6BCX_REG_ALS_WAIT2,
        STK6BCX_REG_NPST_SET1,
        STK6BCX_REG_NPST_SET2,
        STK6BCX_REG_ALS_PRST,
        STK6BCX_REG_ALS_THDH1,
        STK6BCX_REG_ALS_THDH2,
        STK6BCX_REG_ALS_THDL1,
        STK6BCX_REG_ALS_THDL2,
        STK6BCX_REG_INT_CTRL1,
        STK6BCX_REG_INT_CTRL2,
        0x30,
        0x38,
        0x39,
        0x3C,
        0x3D,
        0xB1,
        0xB3,
        0xB4,
        0xC3,

        0x59,
        0x5A,
        0x68,
        0x6A,
        0x6B,
        0x6C,
        0x78,
        0x79,
        0x7C,

        0x7D,
        0x7E,
        0x81,
        0x82,
        0x83,
        0x89,
        0x8A,
        0x8B,
        0x8E,
        STK6BCX_REG_FIFO_SET0,
        STK6BCX_REG_FIFO_SET1,
    };
    uint8_t i = 0;
    uint8_t i2c_data[1];
    uint16_t n = sizeof(stk6bcx_reg_map) / sizeof(stk6bcx_reg_map[0]);

    for (i = 0; i < n; i++)
    {
        i2c_data[0] = stk6bcx_reg_map[i];
        ret = sensor_rx_data(client, i2c_data, 1);

        STK_LOG("reg[0x%02X] = 0x%02X", stk6bcx_reg_map[i], i2c_data[0]);
    }
}


#ifdef STK_ALS_MID_FIR
static void stk6bcx_als_bubble_sort(uint16_t* sort_array, uint8_t size_n)
{
    int i, j, tmp;

    for (i = 1; i < size_n; i++)
    {
        tmp = sort_array[i];
        j = i - 1;

        while (j >= 0 && sort_array[j] > tmp)
        {
            sort_array[j + 1] =  sort_array[j];
            j = j - 1;
        }

        sort_array[j + 1] = tmp;
    }
}

static void stk6bcx_als_recalculate_filter(uint32_t* channel_data)
{
    if (stk6bcx_als.als_data_filter.number < STK_ALS_MID_FIR_LEN)
    {
        stk6bcx_als.als_data_filter.raw[stk6bcx_als.als_data_filter.number] = *channel_data;
        stk6bcx_als.als_data_filter.number++;
        stk6bcx_als.als_data_filter.index++;
    }
    else
    {
        uint8_t index;
        uint16_t mid_als;
        uint16_t cpraw[STK_ALS_MID_FIR_LEN] = {0};
        index = stk6bcx_als.als_data_filter.index % stk6bcx_als.als_data_filter.number;
        stk6bcx_als.als_data_filter.raw[index] = *channel_data;
        stk6bcx_als.als_data_filter.index++;
        sns_memscpy(cpraw,
                    sizeof(cpraw),
                    stk6bcx_als.als_data_filter.raw,
                    sizeof(stk6bcx_als.als_data_filter.raw));
        stk6bcx_als_bubble_sort(cpraw, sizeof(cpraw) / sizeof(cpraw[0]));
        mid_als = cpraw[STK_ALS_MID_FIR_LEN / 2];
        *channel_data = mid_als;
    }
}
#endif

void stk6bcx_als_set_thd(struct i2c_client *client,       uint16_t als_raw)
{
    int ret = 0;
    uint16_t high_thd, low_thd;
    uint8_t reg_high_thd[3], reg_low_thd[3];
    uint32_t temp;
    temp = (uint32_t)als_raw * + als_raw * 5 / 100;

    if (temp > 0xFFFF)
        high_thd = 0xFFFF;
    else
        high_thd = (uint16_t)temp;

    low_thd = als_raw * - als_raw * 5 / 100;
    reg_high_thd[0] = STK6BCX_REG_ALS_THDH1;
    reg_high_thd[1] = (uint8_t)((high_thd >> 8) & 0xFF);
    reg_high_thd[2] = (uint8_t)(high_thd & 0xFF);
    ret = sensor_tx_data(client, reg_high_thd, 3);
    if (ret < 0)
    {
        STK_LOG("fail, ret=%d\n", ret);
    }

    reg_low_thd[0] = STK6BCX_REG_ALS_THDL1;
    reg_low_thd[1] = (uint8_t)((low_thd >> 8) & 0xFF);
    reg_low_thd[2] = (uint8_t)(low_thd & 0xFF);
    ret = sensor_tx_data(client, reg_high_thd, 3);
    if (ret < 0)
    {
        STK_LOG("fail, ret=%d\n", ret);
    }
}


#ifdef STK_GPIO_ALS
/*
static int32_t stk6bcx_als_fsm_pause(struct i2c_client *client, bool is_pause)
{
    int32_t ret = 0;
    uint8_t i2c_data = 0x0;
#ifdef STK_GPIO_ALS
    //uint8_t reg_data = 0;//, reg = 0;
#endif

    i2c_data = is_pause ? STK6BCX_FSM_ALS_PAUSE_MASK : 0;
    ret = sensor_write_reg_mask(client, STK6BCX_REG_FSM_CTRL, i2c_data, STK6BCX_FSM_ALS_PAUSE_MASK);

    if (ret < 0)
    {
        STK_LOG("read modify write i2c (0x%X) error\n", STK6BCX_REG_FSM_CTRL);
        return FAIL;
    }

#ifdef STK_GPIO_ALS

    if (!is_pause)
    {
        //reg_data = 0;
        ret = sensor_write_reg_mask(client, STK6BCX_REG_GPIO_SET1, 0, STK6BCX_GPIO_EN_MEASURE_MASK);
        if (ret < 0)
        {
            STK_LOG("read modify write i2c (0x%X) error\n", STK6BCX_REG_GPIO_SET1);
            return ret;
        }

        //reg_data = STK6BCX_GPIO_EN_MEASURE_MASK;
        ret = sensor_write_reg_mask(client, STK6BCX_REG_GPIO_SET1, STK6BCX_GPIO_EN_MEASURE_MASK, STK6BCX_GPIO_EN_MEASURE_MASK);
        if (ret < 0)
        {
            STK_LOG("read modify write i2c (0x%X) error\n", STK6BCX_REG_GPIO_SET1);
            return ret;
        }
    }

#endif
    return ret;
}

int32_t stk6bcx_als_set_gpio_ignore(struct i2c_client *client, uint16_t ignore_time)
{
    int32_t ret = 0;
    uint8_t tx_buf[3] = {0};//, reg_addr = 0;

    tx_buf[0] = STK6BCX_REG_ALS_WAIT1;
    tx_buf[1] = STK6BCX_H_BYTE(ignore_time) & 0x1F;
    tx_buf[2] = STK6BCX_L_BYTE(ignore_time);
    STK_LOG("ignore: %u us\n", ignore_time * 24);


    ret = sensor_tx_data(client, tx_buf, 3);
    if (ret < 0)
    {
        STK_LOG("fail, ret=%d\n", ret);
    }

    return ret;
}

int32_t stk6bcx_als_set_wait_time(struct i2c_client *client, uint16_t wait)
{
    int32_t ret = 0;
    uint8_t tx_buf[3] = {0};//, reg_addr = 0;

    tx_buf[0] = STK6BCX_H_BYTE(wait);
    tx_buf[1] = STK6BCX_H_BYTE(wait);
    tx_buf[2] = STK6BCX_L_BYTE(wait);
    STK_LOG("wait: %u us\n", wait * 21);

    ret = sensor_tx_data(client, tx_buf, 3);
    if (ret < 0)
    {
        STK_LOG("fail, ret=%d\n", ret);
    }

    return ret;
}
*/
#endif
void stk6bcx_als_gpio_enable(struct i2c_client *client, bool enable)
{
    uint8_t i2c_flag_reg = enable ? STK6BCX_GPIO_ALS_EN : 0;//david
    uint8_t reg_addr = 0;
    int32_t err = 0;

    if (stk6bcx_als.gpio_enable == enable)
    {
        STK_LOG("Already Set\n");
        return;
    }

    reg_addr = STK6BCX_REG_GPIO_SET0;
    err = sensor_write_reg_mask(client, reg_addr, i2c_flag_reg, STK6BCX_GPIO_ALS_EN);//david
    if (err < 0)
    {
        STK_LOG("read modify write i2c (0x%X) error\n", STK6BCX_REG_GPIO_SET0);
        return;
    }

    stk6bcx_als.gpio_enable = enable;
}

stk6bcx_als_dgain_multi stk6bcx_als_get_dgain(uint8_t dg)
{
    return (1 << dg);
}

stk6bcx_als_again_multi stk6bcx_als_get_again(uint8_t ag)
{
    return (1 << (4 - ag));
}

void stk6bcx_als_get_curGain(struct i2c_client *client)
{
    int ret = 0;
    uint8_t  i2c_flag_reg[6] = {0};

    i2c_flag_reg[0] = STK6BCX_REG_ALS_DGAIN;
    ret = sensor_rx_data(client, i2c_flag_reg, 6);
    stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH0] = stk6bcx_als_get_dgain((i2c_flag_reg[0] & STK6BCX_ALS0_GAIN_MASK) >> STK6BCX_ALS0_GAIN_SHIFT);
    stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH1] = stk6bcx_als_get_dgain((i2c_flag_reg[0] & STK6BCX_ALS1_GAIN_MASK) >> STK6BCX_ALS1_GAIN_SHIFT);
    stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH2] = stk6bcx_als_get_dgain((i2c_flag_reg[1] & STK6BCX_ALS2_GAIN_MASK) >> STK6BCX_ALS2_GAIN_SHIFT);
    stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH3] = stk6bcx_als_get_dgain((i2c_flag_reg[1] & STK6BCX_ALS3_GAIN_MASK) >> STK6BCX_ALS3_GAIN_SHIFT);
    stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH4] = stk6bcx_als_get_dgain((i2c_flag_reg[2] & STK6BCX_ALS4_GAIN_MASK) >> STK6BCX_ALS4_GAIN_SHIFT);
    stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH5] = stk6bcx_als_get_dgain((i2c_flag_reg[2] & STK6BCX_ALS5_GAIN_MASK) >> STK6BCX_ALS5_GAIN_SHIFT);
    stk6bcx_als.als_cur_again[STK6BCX_ALS_CH0] = stk6bcx_als_get_again((i2c_flag_reg[3] & STK6BCX_ALS0_CI_MAKS) >> STK6BCX_ALS0_CI_SHIFT);
    stk6bcx_als.als_cur_again[STK6BCX_ALS_CH1] = stk6bcx_als_get_again((i2c_flag_reg[3] & STK6BCX_ALS1_CI_MAKS) >> STK6BCX_ALS1_CI_SHIFT);
    stk6bcx_als.als_cur_again[STK6BCX_ALS_CH2] = stk6bcx_als_get_again((i2c_flag_reg[4] & STK6BCX_ALS2_CI_MAKS) >> STK6BCX_ALS2_CI_SHIFT);
    stk6bcx_als.als_cur_again[STK6BCX_ALS_CH3] = stk6bcx_als_get_again((i2c_flag_reg[4] & STK6BCX_ALS3_CI_MAKS) >> STK6BCX_ALS3_CI_SHIFT);
    stk6bcx_als.als_cur_again[STK6BCX_ALS_CH4] = stk6bcx_als_get_again((i2c_flag_reg[5] & STK6BCX_ALS4_CI_MAKS) >> STK6BCX_ALS4_CI_SHIFT);
    stk6bcx_als.als_cur_again[STK6BCX_ALS_CH5] = stk6bcx_als_get_again((i2c_flag_reg[5] & STK6BCX_ALS5_CI_MAKS) >> STK6BCX_ALS5_CI_SHIFT);
    STK_LOG("stk6bcx_als_get_curGain:: ALS0 cur DG:%d, AG:%d", stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH0], stk6bcx_als.als_cur_again[STK6BCX_ALS_CH0]);
    STK_LOG("stk6bcx_als_get_curGain:: ALS1 cur DG:%d, AG:%d", stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH1], stk6bcx_als.als_cur_again[STK6BCX_ALS_CH1]);
    STK_LOG("stk6bcx_als_get_curGain:: ALS2 cur DG:%d, AG:%d", stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH2], stk6bcx_als.als_cur_again[STK6BCX_ALS_CH2]);
    STK_LOG("stk6bcx_als_get_curGain:: ALS3 cur DG:%d, AG:%d", stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH3], stk6bcx_als.als_cur_again[STK6BCX_ALS_CH3]);
    STK_LOG("stk6bcx_als_get_curGain:: ALS4 cur DG:%d, AG:%d", stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH4], stk6bcx_als.als_cur_again[STK6BCX_ALS_CH4]);
    STK_LOG("stk6bcx_als_get_curGain:: ALS5 cur DG:%d, AG:%d", stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH5], stk6bcx_als.als_cur_again[STK6BCX_ALS_CH5]);
}
/*
static void stk6bcx_gpio_lost_handle(struct i2c_client *client)
{
    int32_t ret = 0;
    uint8_t i2c_data[4] = {0}, reg_addr;
    uint8_t screen_hz = stk6bcx_als.display_freq, als_duty = 0;
    uint32_t measure_time = 0;
    uint32_t als_td = 0, target_timer = 0;
    uint32_t temp_target_timer = 0;
    uint32_t temp_als_td = 0;

    i2c_data[0] = STK6BCX_REG_GPIO_SET7;
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

    if (screen_hz != stk6bcx_als.display_freq)
    {
        stk6bcx_als.display_freq = screen_hz;
        STK_LOG("Current is %dHz\n", stk6bcx_als.display_freq);
        //temp_target_timer = (uint32_t)STK6BCX_GPIO_TD_TIMER(target_timer) / 10;
        i2c_data[0] = STK6BCX_REG_GPIO_SET14;
        i2c_data[1] = (temp_target_timer >> 16) & 0x1F;
        i2c_data[2] = (temp_target_timer >> 8)  & 0xFF;
        i2c_data[3] = (temp_target_timer >> 0)  & 0xFF;
        reg_addr = STK6BCX_REG_GPIO_SET14;
        ret = sensor_tx_data(client, i2c_data, 4);

        if (ret < 0)
        {
            STK_LOG("set TARGET TIMER fail\n");
        }

#ifdef STK_GPIO_ALS

        if (stk6bcx_als.gpio_enable)
        {
            //stk6bcx_als_fsm_pause(client, false);
            //temp_als_td = (uint32_t)STK6BCX_GPIO_TD_TIMER(als_td); //david
            i2c_data[0] = STK6BCX_REG_GPIO_SET20;
            i2c_data[1] = (temp_als_td >> 16) & 0x1F;
            i2c_data[2] = (temp_als_td >> 8)  & 0xFF;
            i2c_data[3] = (temp_als_td >> 0)  & 0xFF;
            //reg_addr = STK6BCX_REG_GPIO_SET20;
            ret = sensor_tx_data(client, i2c_data, 3);

            if (ret < 0)
            {
                STK_LOG("set ALS_TD fail\n");
            }

            i2c_data[0] = STK6BCX_REG_GPIO_SET27;
            i2c_data[0] = als_duty;
            //reg_addr = STK6BCX_REG_GPIO_SET27;
            ret = sensor_tx_data(client, i2c_data, 2);

            if (ret < 0)
            {
                STK_LOG("set ALS_DUTY fail\n");
                return;
            }

            //stk6bcx_als_fsm_pause(client, false);
        }

#endif
    }
}
*/
static int stk6bcx_sensor_check_id(struct i2c_client *client)
{
    int ret = FAIL;
    uint8_t pid_count = 0;
    uint8_t reg_addr = STK6BCX_REG_PID, reg_data = 0;

    reg_data = sensor_read_reg(client, reg_addr);
    STK_LOG("get pid = 0x%x\n", reg_data);

    for (pid_count = 0; pid_count < (sizeof(stk6bcx_pid_list) / sizeof(uint8_t)); pid_count++)
    {
        if ( reg_data == stk6bcx_pid_list[pid_count])
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

static int stk6bcx_sensor_hw_init(struct i2c_client *client)
{
    int i, ret = FAIL;

    for (i = 0; i < (sizeof(stk6bcx_als_default_register_table) / sizeof(stk6bcx_register_table)); i++)
    {
        ret = sensor_write_reg_mask(client, stk6bcx_als_default_register_table[i].address, stk6bcx_als_default_register_table[i].value, stk6bcx_als_default_register_table[i].mask);

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

static void stk6bcx_reset_fifo_buffer(void)
{
    stk6bcx_als.xflag = 0;
    memset(stk6bcx_als.fifo_data0, 0x0, sizeof(stk6bcx_als.fifo_data0));
    memset(stk6bcx_als.fifo_data1, 0x0, sizeof(stk6bcx_als.fifo_data1));
    memset(stk6bcx_als.fifo_data2, 0x0, sizeof(stk6bcx_als.fifo_data2));
    memset(stk6bcx_als.fifo_data3, 0x0, sizeof(stk6bcx_als.fifo_data3));
    memset(stk6bcx_als.fifo_data4, 0x0, sizeof(stk6bcx_als.fifo_data4));
    memset(stk6bcx_als.fifo_data5, 0x0, sizeof(stk6bcx_als.fifo_data5));
}

static void stk6bcx_fifo_init(struct i2c_client *client)
{
    uint8_t reg_value;

    reg_value = sensor_read_reg(client, STK6BCX_REG_FIFO_SET0);

    if (reg_value < 0)
    {
        STK_LOG("stk i2c failed\n");
        return ;
    }

    stk6bcx_als.data_type = reg_value & STK6BCX_FIFO_DATA_SEL_MASK;
    stk6bcx_reset_fifo_buffer();

    switch (stk6bcx_als.data_type)
    {
        case STK6BCX_FIFO_DATA_SEL_ALS012345:
            stk6bcx_als.fifo_channel_byte = 2;
            stk6bcx_als.frame_byte = STK6BCX_ALS_CNT * stk6bcx_als.fifo_channel_byte;
            break;

        case STK6BCX_FIFO_DATA_SEL_STA012345_ALS012345:
            stk6bcx_als.fifo_channel_byte = 3;
            stk6bcx_als.frame_byte = STK6BCX_ALS_CNT * stk6bcx_als.fifo_channel_byte;
            stk6bcx_als.xFlag_pos = 0x80;
            break;

        case STK6BCX_FIFO_DATA_SEL_STAR012345_ALSR012345:
            stk6bcx_als.fifo_channel_byte = 4;
            stk6bcx_als.frame_byte = STK6BCX_ALS_CNT * stk6bcx_als.fifo_channel_byte;
            stk6bcx_als.xFlag_pos = 0x40;
            break;

        default:
            stk6bcx_als.frame_byte = 0xFF;
            STK_LOG("stk6bcx_fifo_init:: ERROR!");
            break;
    }

    stk6bcx_als.read_frame = STK_FIFO_I2C_READ_FRAME;
    stk6bcx_als.target_frame_count = STK_FIFO_I2C_READ_FRAME_TARGET;
    stk6bcx_als.read_max_byte = stk6bcx_als.frame_byte * STK_FIFO_I2C_READ_FRAME;
    STK_LOG("target_frame_count = %d\n", stk6bcx_als.target_frame_count);
}

static void stk6bcx_fifo_enable(struct i2c_client *client, bool enabled)
{
    uint8_t ret = 0;
    uint8_t reg_value = 0;

    if ( stk6bcx_als.fifo_enable == enabled)
    {
        STK_LOG("fifo already set\n");
        return;
    }

    reg_value = sensor_read_reg(client, STK6BCX_REG_FIFO_SET0);

    if (enabled)
    {
#ifdef STK_ALS_IT1_SHORT
        reg_value |= STK6BCX_FIFO_MODE_STREAM;
#else
        reg_value |= STK6BCX_FIFO_MODE_BYPASS;
#endif
    }
    else
    {
        reg_value &= ~(0x03);
    }

    ret = sensor_write_reg(client, STK6BCX_REG_FIFO_SET0, reg_value);

    if (ret < 0)
    {
        STK_LOG("stk i2c failed\n");
        return;
    }

    ret = sensor_write_reg_mask(client, STK6BCX_REG_FIFO_SET0, reg_value, STK6BCX_FIFO_MODE_MASK);

    if (ret < 0)
    {
        STK_LOG("stk i2c failed\n");
        return;
    }

    //stk6bcx_als.fifo_enable = enabled;
}

static void stk6bcx_avg_data(void)
{
    uint16_t i;
    uint32_t data_sum[6] = {0};

    for (i = 0; i < stk6bcx_als.last_frame_count; i++)
    {
        data_sum[0] += *(stk6bcx_als.fifo_data0 + i);
        data_sum[1] += *(stk6bcx_als.fifo_data1 + i);
        data_sum[2] += *(stk6bcx_als.fifo_data2 + i);
        data_sum[3] += *(stk6bcx_als.fifo_data3 + i);
        data_sum[4] += *(stk6bcx_als.fifo_data4 + i);
        data_sum[5] += *(stk6bcx_als.fifo_data5 + i);
    }

    for (i = STK6BCX_ALS_CH0; i < STK6BCX_ALS_CNT; i++)
    {
        stk6bcx_als.last_raw_data[i] = (data_sum[i] / stk6bcx_als.last_frame_count);
    }

    stk6bcx_als.fifo_is_ready = true;
    return;
}

static void stk6bcx_get_fifo_data(struct i2c_client *client, uint16_t frame_num)
{
    int ret = 0;
    uint32_t read_bytes;
    //uint8_t raw_data[stk6bcx_als.read_max_byte];
    uint8_t raw_data[128];
    //uint8_t *raw_data = stk6b1x_als.raw_data;
    uint16_t i, offset, frame_count, read_frame_num;
//    uint16_t chIdx = 0;
   // uint32_t dg_ratio[5] = {0}, ag_ratio[5] = {0};

    stk6bcx_reset_fifo_buffer();

    for (frame_count = 0 ; frame_count < frame_num ; frame_count += (stk6bcx_als.read_frame))
    {
        read_frame_num = (int16_t)(frame_num - frame_count);

        if (read_frame_num >= stk6bcx_als.read_frame)
        {
            read_bytes = stk6bcx_als.read_max_byte;
            read_frame_num = stk6bcx_als.read_frame;
        } else {
            read_bytes = stk6bcx_als.frame_byte * read_frame_num;
        }

        memset(raw_data, 0, STK_FIFO_I2C_READ_BYTE);
        raw_data[0] = STK6BCX_REG_ALS_FIFO_OUT;
        ret = sensor_rx_data(client, raw_data, read_bytes);

        if (ret < 0)
        {
            STK_LOG("stk6bcx_get_fifo_data failed\n");
            return;
        }

        switch (stk6bcx_als.data_type)
        {
        case STK6BCX_FIFO_DATA_SEL_ALS012345:
            for (i = 0, offset = 0; i < read_frame_num; i++, offset += stk6bcx_als.frame_byte)
            {
                *(stk6bcx_als.fifo_data0 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0) + 0) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0) + 1) << 0);
                *(stk6bcx_als.fifo_data1 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1) + 0) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1) + 1) << 0);
                *(stk6bcx_als.fifo_data2 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2) + 0) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2) + 1) << 0);
                *(stk6bcx_als.fifo_data3 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3) + 0) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3) + 1) << 0);
                *(stk6bcx_als.fifo_data4 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4) + 0) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4) + 1) << 0);
                *(stk6bcx_als.fifo_data5 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5) + 0) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5) + 1) << 0);
            }
            break;
        case STK6BCX_FIFO_DATA_SEL_STA012345_ALS012345:
            for (i = 0, offset = 0; i < read_frame_num; i++, offset += stk6bcx_als.frame_byte)
            {
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH0) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH1) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH2) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH3) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH4) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH5) : 0;
                *(stk6bcx_als.fifo_data0 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0) + 1) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0) + 2) << 0);
                *(stk6bcx_als.fifo_data1 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1) + 1) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1) + 2) << 0);
                *(stk6bcx_als.fifo_data2 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2) + 1) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2) + 2) << 0);
                *(stk6bcx_als.fifo_data3 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3) + 1) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3) + 2) << 0);
                *(stk6bcx_als.fifo_data4 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4) + 1) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4) + 2) << 0);
                *(stk6bcx_als.fifo_data5 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5) + 1) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5) + 2) << 0);
            }
            break;
        case STK6BCX_FIFO_DATA_SEL_STAR012345_ALSR012345:
            for (i = 0, offset = 0; i < read_frame_num; i++, offset += stk6bcx_als.frame_byte)
            {
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH0) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH1) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH2) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH3) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH4) : 0;
                stk6bcx_als.xflag |=
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5)) & stk6bcx_als.xFlag_pos) ? (1 << STK6BCX_ALS_CH5) : 0;
                *(stk6bcx_als.fifo_data0 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0) + 1) << 16) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0) + 2) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH0) + 3) << 0);
                *(stk6bcx_als.fifo_data1 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1) + 1) << 16) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1) + 2) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH1) + 3));
                *(stk6bcx_als.fifo_data2 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2) + 1) << 16) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2) + 2) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH2) + 3) << 0);
                *(stk6bcx_als.fifo_data3 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3) + 1) << 16) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3) + 2) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH3) + 3) << 0);
                *(stk6bcx_als.fifo_data4 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4) + 1) << 16) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4) + 2) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH4) + 3) << 0);
                *(stk6bcx_als.fifo_data5 + frame_count + i) =
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5) + 1) << 16) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5) + 2) << 8) |
                    (*(raw_data + offset + (stk6bcx_als.fifo_channel_byte * STK6BCX_ALS_CH5) + 3) << 0);
            }
            break;
            default:
                STK_LOG("unavailable!\n");
                break;
        }
    }

    stk6bcx_als.last_frame_count = frame_num;
    for ( i = STK6BCX_ALS_CH0; i < STK6BCX_ALS_CNT; i++)
    {
        if (stk6bcx_als.xflag & (1 < i))
        {
            STK_LOG("stk6bcx_fifo_get_data:: ALS%d XFLAG OCCUR!!", i);
        }
    }
    // STK_LOG("para=%d, %d, %d, %d, %d",
    //                 stk6bcx_als->read_max_byte,
    //                 stk6bcx_als->read_frame,
    //                 stk6bcx_als->frame_byte,
    //                 read_bytes,
    //                 frame_num);
}

static int stk6bcx_get_fifo_data_polling(struct i2c_client *client)
{
    int ret = 0;
    char buf[3] = {0};
    uint16_t frame_num;
    stk6bcx_als.fifo_is_ready = false;
    buf[0] = STK6BCX_REG_ALS_FIFO_FLAG;
    buf[1] = sensor_read_reg(client, buf[0]);

    if (buf[1] < 0)
    {
        STK_LOG("stk i2c failed\n");
        return buf[1];
    }
    if ((buf[1] & (STK6BCX_FLG_FIFO_OVR_MAKS | STK6BCX_FLG_FIFO_WM_MAKS | STK6BCX_FLG_FIFO_FULL_MAKS)) != 0)
    {
        STK_LOG("stk6bcx_fifo_get_data_polling:: i2c_flag_reg = 0x%x", buf[1]);
    }

    buf[0] = STK6BCX_REG_ALS_FIFO_CNT1;
    ret = sensor_rx_data(client, buf, 2);

    frame_num = (buf[0] << 8) | buf[1];
    STK_LOG("frame_num = %d\n", frame_num);

    if (frame_num >= STK_FIFO_I2C_READ_FRAME_TARGET)
    {
        frame_num = STK_FIFO_I2C_READ_FRAME_TARGET;
    }

    stk6bcx_get_fifo_data(client, frame_num);

    if (stk6bcx_als.last_frame_count != 0)
    {
        stk6bcx_avg_data();
    }

    //stk6bcx_als.fifo_is_ready = true;
    return ret;
}

static int stk6bcx_sensor_init(struct i2c_client *client)
{
    int ret = FAIL;

    ret = stk6bcx_sensor_check_id(client);

    if (ret < NO_ERROR)
    {
        STK_LOG("light check id error ret = %d!", ret);
        return 0;
    }

    //sw reset
    sensor_write_reg(client, STK6BCX_REG_SWRST, STK_STK6BCX_SWRESET);
    //tx_thread_sleep(15);
    usleep_range(13000, 15000);
    ret = stk6bcx_sensor_hw_init(client);

    if (ret < NO_ERROR)
    {
        STK_LOG("light hw init error ret = %d!", ret);
        return 0;
    }

    stk6bcx_als_get_curGain(client);
    stk6bcx_fifo_init(client);
    return 0;
}

/*
static int stk6bcx_sensor_set_cali_data(void *cali_data)
{
    return NO_ERROR;
}

static int stk6bcx_sensor_rate(int32_t sampling_period_us)
{
    return NO_ERROR;
}
*/
static int stk6bcx_sensor_enable(struct i2c_client *client, int enable, int rate)
{
    int ret = FAIL;
    uint8_t buf[2] = {STK6BCX_REG_ENABLE, 0x00};

    buf[1] = sensor_read_reg(client, buf[0]);

    if(enable){
        buf[1] |= (STK6BCX_STATE_EN_ALS_WAIT_MASK | STK6BCX_STATE_EN_ALS_MASK);
        if (enable_state != 1)
        {
            ret = sensor_write_reg_mask(client, buf[0], buf[1], STK6BCX_STATE_EN_ALS_WAIT_MASK | STK6BCX_STATE_EN_ALS_MASK);
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
            stk6bcx_als_gpio_enable(client, true);
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
        stk6bcx_als.last_frame_count = 0;
//#ifdef STK6BCX_FIFO_EN
        stk6bcx_fifo_enable(client, true);
//#endif
#ifdef STK6BCX_RGB_ENABLE
        stk6bcx_als.ch_scale[STK6BCX_ALS_CH0] = 1.00;
        stk6bcx_als.ch_scale[STK6BCX_ALS_CH1] = 1.00;
        stk6bcx_als.ch_scale[STK6BCX_ALS_CH2] = 1.00;
        stk6bcx_als.ch_scale[STK6BCX_ALS_CH3] = 1.00;
        stk6bcx_als.ch_scale[STK6BCX_ALS_CH4] = 1.00;
        stk6bcx_als.ch_scale[STK6BCX_ALS_CH5] = 1.00;
#endif
        reject_frames_num = 0;
    }else{
            buf[1] &= (~(STK6BCX_STATE_EN_ALS_WAIT_MASK | STK6BCX_STATE_EN_ALS_MASK));
            if (enable_state != 0)
            {
                ret = sensor_write_reg_mask(client, buf[0], buf[1], STK6BCX_STATE_EN_ALS_WAIT_MASK | STK6BCX_STATE_EN_ALS_MASK);
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
                stk6bcx_als_gpio_enable(client, false);
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
#ifdef STK6BCX_FIFO_EN
            stk6bcx_fifo_enable(client, false);
#endif
            last_luxdata = -1.0f;

        }
    return ret;
}
/*
static int stk6bcx_sensor_disable(struct i2c_client *client)
{
    int ret = FAIL;
    uint8_t buf[2] = {STK6BCX_REG_ENABLE, 0x00};
    buf[1] = sensor_read_reg(client, buf[0]);
    buf[1] &= (~(STK6BCX_STATE_EN_ALS_WAIT_MASK | STK6BCX_STATE_EN_ALS_MASK));

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
        stk6bcx_als_gpio_enable(false);
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
    stk6bcx_fifo_enable(false);
    last_luxdata = -1.0f;
    return ret;
}

static int stk6bcx_sensor_set_status()
{
    return NO_ERROR;
}

static int stk6bcx_sensor_get_status()
{
    //uint8_t reg_addr = STK6BCX_REG_ALS_FLAG;
    //uint8_t reg_data = STK6BCX_FLG_ALS_DR_MASK;
    return NO_ERROR;
}

static void stk6bcx_sensor_cali_cmd_handle(int cal_cmd, int cali_type, int golden_sample)
{
    return NO_ERROR;
    //light_sensor_cali_cmd_handle(cal_cmd, cali_type, golden_sample);
}

static void stk6bcx_sensor_get_cali_data()
{
    return NO_ERROR;
    //light_sensor_get_cali_data();
}

static int stk6bcx_sensor_calibration(float *raw_data)
{
    (void)raw_data;
    return NO_ERROR;
}
*/
//#define STK6BCX_LSC
#ifdef STK6BCX_LSC
static void stk6bcx_als_compensation(uint32_t* als_data)
{
    float lux_calc = 0;
    float cct_calc = 0;
    float gc_ratio = 0;
    float rb_ratio = 0;
    float div_ratio = 0;
    float div_cct_ratio = 0;
    bool  entry_flag = false;

    float f_calc_rgb_data[4]; //fcrbg
    float color_axis[3] = {0.0};
    float alpha_v=0, beta_v=0, n_v=0;
    bool cct_modify_flag=false;
    float calc_tmp1=0.0;
    float calc_tmp2=0.0;

    float calc_tmp[4]={0};//fcrbg
    float calc_tmp_diff_ratio=0.0;

    uint8_t i = 0;
    uint8_t lux_calc_i = 0;
    uint8_t lux_calc_j = 0;
    uint8_t cct_calc_i = 0;
    uint8_t cct_calc_j = 0;
    uint8_t lux_size = 4;
    uint8_t cct_size = 4;

    light_pare lux_param[4] = {
        {0, 0,  1, 3, 5.4,   (0.0),(0.0591),(0.0),(-0.0),          0.010208,   0, 0,  1.02,  100.0,  0}, //FOR A\H
        {1, 0,  1, 3, 3.2,   (0.0),(0.0591),(0.0),(-0.0),          0.010208,   0, 0,  1.02,  100.0,  0},//FOR D65
        {2, 0,  1, 3, 0.0,    (0.0),(0.0591),(0.0),(-0.0),           0.010208,   0, 0,  1.02,  100.0,  0}, // for CWF
        {3, 0,  1, 3, 0.0,    (0.0),(0.0591),(0.0),(-0.0),          0.010208,   0, 0,  1.02,  100.0,  0},
    };

    float stk6bcx_mat_cal[4][18] = {
       {//FOR CWF 1123
            (0.111672),(0.851265),(0.136398),(-0.238495),
            (0),(1),(0),(0                                ),
            (-0.201656),(0.115007),(1.596035),(-0.45148 ),
            5000.0,      10,         0,           1,
            1,             3
        },
        {//FOR CWF1
            (0.111672),(0.851265),(0.136398),(-0.238495),
            (0),(1),(0),(0                                ),
            (-0.201656),(0.115007),(1.596035),(-0.45148 ),
            5000.0,       5,         0,           1,
            1,             3
        },
        {//FOR D65
            (0.111672),(0.851265),(0.136398),(-0.238495),
            (0),(1),(0),(0                                ),
            (-0.201656),(0.115007),(1.596035),(-0.45148 ),
            5000.0,      2,         0,           1,
            1,             3
        },
        {//FOR A\H
            (0.111672),(0.851265),(0.136398),(-0.238495),
            (0),(1),(0),(0                                ),
            (-0.201656),(0.115007),(1.596035),(-0.45148 ),
            5000.0,      0.1,         0,           1,
            1,             3
        },
    };


    f_calc_rgb_data[STK6BCX_ALS_R] = (float)(als_data[STK6BCX_ALS_CH4]);
    f_calc_rgb_data[STK6BCX_ALS_G] = (float)(als_data[STK6BCX_ALS_CH0]);
    f_calc_rgb_data[STK6BCX_ALS_B] = (float)(als_data[STK6BCX_ALS_CH5]);
    f_calc_rgb_data[STK6BCX_ALS_W] = (float)(als_data[STK6BCX_ALS_CH1]);
#ifdef STK6BCX_ENABLE_DEBUG_MSG
    STK_LOG( "stk6bcx_als_compensation:: get raw_data, RGBW = %d %d %d %d",
            als_data[STK6BCX_ALS_CH4],
            als_data[STK6BCX_ALS_CH0],
            als_data[STK6BCX_ALS_CH5],
            als_data[STK6BCX_ALS_CH1]);
#endif
    //with factory cali. scale

    //f_calc_rgb_data[STK_ALS_DATA_F] *= 0;
    f_calc_rgb_data[STK6BCX_ALS_R] *= stk6bcx_als.r_scale;
    f_calc_rgb_data[STK6BCX_ALS_G] *= stk6bcx_als.g_scale;
    f_calc_rgb_data[STK6BCX_ALS_B] *= stk6bcx_als.b_scale;
    f_calc_rgb_data[STK6BCX_ALS_W] *= stk6bcx_als.w_scale;
#ifdef STK6BCX_ENABLE_DEBUG_MSG
    STK_LOG( "stk6bcx_als_compensation:: get combine_data, rgbw = %d %d %d %d",
            (uint32_t)(f_calc_rgb_data[STK6BCX_ALS_R]),
            (uint32_t)(f_calc_rgb_data[STK6BCX_ALS_G]),
            (uint32_t)(f_calc_rgb_data[STK6BCX_ALS_B]),
            (uint32_t)(f_calc_rgb_data[STK6BCX_ALS_W]));
#endif

/************calc lux****************/
    for(lux_calc_i = 0; lux_calc_i<lux_size; lux_calc_i++) {

        gc_ratio = (lux_param[lux_calc_i].group_rule_mat);
        rb_ratio = (lux_param[lux_calc_i].group_rule_mat_2);

        div_ratio = f_calc_rgb_data[(uint8_t)gc_ratio];
        if(FLOAT_EPS < f_calc_rgb_data[(uint8_t)rb_ratio])
            div_ratio /= f_calc_rgb_data[(uint8_t)rb_ratio]; //c/g

        if(//(lux_param[0].param_lower_thd > als_data[STK6BCX_ALS_CH0])&&
            (lux_param[0].param_lower_thd > als_data[STK6BCX_ALS_CH4])&&
            (lux_param[0].param_lower_thd > als_data[STK6BCX_ALS_CH0])&&
            (lux_param[0].param_lower_thd > als_data[STK6BCX_ALS_CH5])&&
            (lux_param[0].param_lower_thd > als_data[STK6BCX_ALS_CH1]))
            {
                //default
                lux_calc_i = (lux_param[0].param_lower_sel);
                calc_tmp[STK6BCX_ALS_R] = 0;
                calc_tmp[STK6BCX_ALS_R] *= (lux_param[lux_calc_i].param_g2_scale);

                calc_tmp[STK6BCX_ALS_G] = f_calc_rgb_data[STK6BCX_ALS_G];
                calc_tmp[STK6BCX_ALS_G] *= (lux_param[lux_calc_i].param_g2_scale);

                calc_tmp[STK6BCX_ALS_B] = 0;
                calc_tmp[STK6BCX_ALS_B] *= (lux_param[lux_calc_i].param_b2_scale);

                calc_tmp[STK6BCX_ALS_W] = 0;
                calc_tmp[STK6BCX_ALS_W] *= (lux_param[lux_calc_i].param_b2_scale);

                    //normalize data
                calc_tmp1 = 0;

                for(i = STK6BCX_ALS_R; i <= STK6BCX_ALS_W; i++){
                    calc_tmp1 +=calc_tmp[i];
                }

                lux_calc = calc_tmp1;

                lux_calc_j = lux_calc_i;
                lux_calc_i = (lux_size+1);
#ifdef STK6BCX_ENABLE_DEBUG_MSG
                STK_LOG( "stk6bcx_als_compensation:: lux dark, (def) div_ratio*100=%d, group = %d, diff_ratio*10 = %d (lower_sel!)",
                (uint16_t)(div_ratio*100),
                (uint16_t)(lux_param[lux_calc_j].group_sel),
                (uint16_t)(calc_tmp_diff_ratio*10));
#endif
            }else{
                entry_flag= false; //initial flag
                if(0 == lux_param[lux_calc_i].group_rule){
                    if(div_ratio > lux_param[lux_calc_i].param_gc_ratio)
                        entry_flag = true;
                } else {
                    if(div_ratio < lux_param[lux_calc_i].param_gc_ratio)
                        entry_flag = true;
                }

                if((true == entry_flag) || (lux_calc_i == (lux_size-1))) {
                //add lux_calc_i==lux_size-1, to handle gc_ratio==0 situation

                    calc_tmp[STK6BCX_ALS_R] = f_calc_rgb_data[STK6BCX_ALS_R];
                    calc_tmp[STK6BCX_ALS_R] *= (lux_param[lux_calc_i].param_r_scale);

                    calc_tmp[STK6BCX_ALS_G] = f_calc_rgb_data[STK6BCX_ALS_G];
                    calc_tmp[STK6BCX_ALS_G] *= (lux_param[lux_calc_i].param_g_scale);

                    calc_tmp[STK6BCX_ALS_B] = f_calc_rgb_data[STK6BCX_ALS_B];
                    calc_tmp[STK6BCX_ALS_B] *= (lux_param[lux_calc_i].param_b_scale);

                    calc_tmp[STK6BCX_ALS_W] = f_calc_rgb_data[STK6BCX_ALS_W];
                    calc_tmp[STK6BCX_ALS_W] *= (lux_param[lux_calc_i].param_c_scale);

                    //normalize data
                    calc_tmp1 = 0;
                    calc_tmp2 = 0;
                    for(i = STK6BCX_ALS_R; i <= STK6BCX_ALS_W; i++){
                        if(0 < calc_tmp[i]){
                            calc_tmp1 +=calc_tmp[i];
                        } else {
                            calc_tmp2 +=calc_tmp[i];
                        }
                    }

                    calc_tmp_diff_ratio = calc_tmp1;
                    if(0 != calc_tmp2)
                        calc_tmp_diff_ratio /= STK_ABS(calc_tmp2);

                    if(calc_tmp_diff_ratio > (lux_param[lux_calc_i].param_limit)) {
                        lux_calc =  calc_tmp1 + calc_tmp2;

                    } else {
                        calc_tmp1 = f_calc_rgb_data[STK6BCX_ALS_G];
                        calc_tmp1 *= (lux_param[lux_calc_i].param_g2_scale);
                        lux_calc = calc_tmp1;
                    }

                    lux_calc_j = lux_calc_i;
                    lux_calc_i = (lux_size+1);

                    if((lux_size+1) == lux_calc_i) {
                    STK_LOG( "stk6bcx_als_compensation::lux = %d div_ratio*100=%d, group = %d(%d), diff_ratio*10 = %d",
                                        (uint32_t)lux_calc,
                                        (uint16_t)(div_ratio*100),
                                        (uint16_t)(lux_calc_j),
                                        (uint16_t)(lux_param[lux_calc_j].group_sel),
                                        (uint16_t)(calc_tmp_diff_ratio*10));
                    }
                }
            }
       }
/************calc lux end****************/

//#ifdef STK_RGB_ENABLE
/************calc CCT part****************/
        for(cct_calc_i = 0; cct_calc_i < cct_size; cct_calc_i++) {
            color_axis[0] = (stk6bcx_mat_cal[cct_calc_i][16]);
            color_axis[1] = (stk6bcx_mat_cal[cct_calc_i][17]);

            div_cct_ratio = f_calc_rgb_data[(uint8_t)color_axis[0]];
            if(FLOAT_EPS < f_calc_rgb_data[(uint8_t)color_axis[1]])
                div_cct_ratio /= f_calc_rgb_data[(uint8_t)color_axis[1]];// c/g

            entry_flag= false; //initial flag
            if(0 == stk6bcx_mat_cal[cct_calc_i][15]){
                if(div_cct_ratio > stk6bcx_mat_cal[cct_calc_i][13])
                    entry_flag = true;
            } else {
                if(div_cct_ratio < stk6bcx_mat_cal[cct_calc_i][13])
                    entry_flag = true;
            }

            if((true == entry_flag)|| (cct_calc_i == (cct_size-1))) {
#ifdef STK6BCX_ENABLE_DEBUG_MSG
                STK_LOG( "stk6bcx_als_compensation:: CCT p1(up)=%d, p2(low) = %d, div_cct_ratio*100 = %d,[%d, %d], group=%d!!",
                        (uint16_t)color_axis[0],
                        (uint16_t)color_axis[1],
                        (uint16_t)(div_cct_ratio*100),
                        (uint16_t)f_calc_rgb_data[(uint8_t)color_axis[0]],
                        (uint16_t)f_calc_rgb_data[(uint8_t)color_axis[1]],
                        (uint16_t)cct_calc_i);
#endif
                for (i = 0; i < 3; i++) {
                    color_axis[i] =
                        stk6bcx_mat_cal[cct_calc_i][(i*4)] *   (f_calc_rgb_data[STK6BCX_ALS_R]) +
                        stk6bcx_mat_cal[cct_calc_i][(i*4)+1] * (f_calc_rgb_data[STK6BCX_ALS_G]) +
                        stk6bcx_mat_cal[cct_calc_i][(i*4)+2] * (f_calc_rgb_data[STK6BCX_ALS_B]) +
                        stk6bcx_mat_cal[cct_calc_i][(i*4)+3] * (f_calc_rgb_data[STK6BCX_ALS_W]);
                    color_axis[i] *= 1;
                }
                cct_calc_j = cct_calc_i;
                cct_calc_i = cct_size+1;
            }
        }

        //protect cct start
        cct_calc = (float)stk6bcx_mat_cal[0][12]; //default
        if(((STK6BCX_MAX_GAIN*65535) == als_data[STK6BCX_ALS_CH0])&&
            ((STK6BCX_MAX_GAIN*65535) == als_data[STK6BCX_ALS_CH1])&&
            ((STK6BCX_MAX_GAIN*65535) == als_data[STK6BCX_ALS_CH4])&&
            ((STK6BCX_MAX_GAIN*65535) == als_data[STK6BCX_ALS_CH5]))
            {
            //avoid channel saturation

            cct_calc = (float)STK6BCX_ALS_MAX_LUX_CCT_LIMIT;
            alpha_v = 0.31352;
            beta_v  = 0.32363;
#ifdef STK6BCX_ENABLE_DEBUG_MSG
            STK_LOG( "stk6bcx_als_compensation:: channel data: saturation!!r=%d, g=%d, b=%d, c=%d",
                    als_data[STK6BCX_ALS_CH4],
                    als_data[STK6BCX_ALS_CH0],
                    als_data[STK6BCX_ALS_CH5],
                    als_data[STK6BCX_ALS_CH1]);
#endif
        } else {
#ifdef STK6BCX_ENABLE_DEBUG_MSG
            STK_LOG( "stk6bcx_als_compensation:: X=%d, Y=%d, Z=%d (*1000000)",
                    (int32_t)(color_axis[0] * 1000000),
                    (int32_t)(color_axis[1] * 1000000),
                    (int32_t)(color_axis[2] * 1000000));
#endif
            //normal
            if((0 ==color_axis[0])&&
                (0 ==color_axis[1])&&
                (0 ==color_axis[2])) {

                cct_calc = (float)stk6bcx_mat_cal[cct_calc_j][12];
                alpha_v = 0.3804;
                beta_v  = 0.3828;

            } else {
                alpha_v = color_axis[0] / (color_axis[0] + color_axis[1] + color_axis[2]);
                beta_v  = color_axis[1] / (color_axis[0] + color_axis[1] + color_axis[2]);
                cct_modify_flag = true;
                //protect
                if(alpha_v > 0.73) {
                    alpha_v = 0.72;
                } else if(alpha_v < 0.260) {
                    alpha_v = 0.260;
                }

                //y
                if(beta_v > 0.71) {
                    beta_v = 0.7;
                } else if(beta_v < 0.253) {
                    beta_v = 0.263;
                }

                    n_v = (alpha_v - 0.332) / (0.1858 - beta_v);
                    cct_calc = (437 * (n_v*n_v*n_v)) +
                                (3601 * (n_v*n_v)) +
                                (6861 * n_v) +
                                5517;

                if(cct_modify_flag)
                {
                    cct_calc += stk6bcx_mat_cal[cct_calc_j][14];
                }
            }

            //low lux (<N lux)
            if(lux_calc <= STK6BCX_ALS_MIN_LUX_THD_LIMIT)
            {
                cct_modify_flag = false;
                cct_calc = (float)stk6bcx_mat_cal[0][12];
                alpha_v = 0.3451;
                beta_v  = 0.35161;
#ifdef STK6BCX_ENABLE_DEBUG_MSG
                STK_LOG( "stk6bcx_als_compensation:: under MIN_Lux (limit)!!, lux = %d",
                        (uint32_t)lux_calc);
#endif
            }
        }

    stk6bcx_als.cct = cct_calc;
//#endif
/************calc CCT end****************/
    stk6bcx_als.lux = lux_calc;
    STK_LOG( ":: lux-cct-rgbw = %d %d %d %d %d %d",
         (uint32_t)lux_calc, (uint32_t)cct_calc, als_data[0],als_data[1],als_data[2],als_data[3]);
    //state->als_info.als_cct = cct_calc;

}

#endif
static int stk6bcx_light_report_abs_value(struct input_dev *input, int data)
{
    input_report_abs(input, ABS_MISC, data);
    input_sync(input);
    return data;
}

static void stk6bcx_als_get_data(struct i2c_client *client)
{
    int rv = 0;
//    uint32_t xfer_bytes;
    uint8_t  raw_data[30] = {0};
    int      loop_count;

    raw_data[0] = STK6BCX_REG_ALS_FIFO_OUT;
    rv = sensor_rx_data(client,  raw_data, stk6bcx_als.frame_byte);

    //STK_LOG("get raw %d %d %d %d %d", raw_data[0], raw_data[1], raw_data[2], raw_data[3], stk6bcx_als.frame_byte);
    if (stk6bcx_als.data_type == STK6BCX_FIFO_DATA_SEL_ALS012345)
    {
        for (loop_count = 0; loop_count < STK6BCX_ALS_CNT; loop_count++)
        {
            stk6bcx_als.last_raw_data[loop_count] = ((raw_data[2 * loop_count] << 8) | raw_data[(2 * loop_count) + 1]);
        }
    }
    else if (stk6bcx_als.data_type == STK6BCX_FIFO_DATA_SEL_STA012345_ALS012345)
    {
        for (loop_count = 0; loop_count < STK6BCX_ALS_CNT; loop_count++)
        {
            stk6bcx_als.last_raw_data[loop_count] = ((raw_data[(3 * loop_count) + 1] << 8) | raw_data[(3 * loop_count) + 2]);
        }
    }
    else if (stk6bcx_als.data_type == STK6BCX_FIFO_DATA_SEL_STAR012345_ALSR012345)
    {
        for (loop_count = 0; loop_count < STK6BCX_ALS_CNT; loop_count++)
        {
            stk6bcx_als.last_raw_data[loop_count] = ((raw_data[(4 * loop_count) + 1] << 16) | (raw_data[(4 * loop_count) + 2] << 8) | (raw_data[(4 * loop_count) + 3] << 0));
        }
    }
    else
    {
        STK_LOG("stk6bcx_als_get_data:: fifo type not support yet.");
    }
}

static int stk6bcx_sensor_get_data(struct i2c_client *client)
{
    struct sensor_private_data *sensor =
    (struct sensor_private_data *) i2c_get_clientdata(client);
    int ret = 0;
    char buf[2] = {0};
//    uint8_t i;
    uint32_t luxdata_flt;
    uint32_t channel_data[STK6BCX_ALS_CNT] = {0};
//    struct sensor_event sensor_event_data;
#ifdef STK_ALGO_ENABLE
    PixelData DisplayData; //input display info
    ChannelData RawData;
    ChannelData AmbientData;
    ChannelData DisplayNoiseData;
#endif

    if(++stk6bcx_als.als_dbg_cnt == STK6BCX_ALS_DUMP_CNT) {
        stk6bcx_dump_reg(client);
        stk6bcx_als.als_dbg_cnt = 0;
    }
    if(stk6bcx_als.fifo_enable){
        stk6bcx_get_fifo_data_polling(client);
        if (!stk6bcx_als.fifo_is_ready)
            return 0;
    } else {
        buf[0] = STK6BCX_REG_ALS_FLAG;
        ret = sensor_rx_data(client, buf, 1);
        stk6bcx_als.als_is_ready =((buf[0] & STK6BCX_FLG_ALS_DR_MASK) ? true : false);
        if(stk6bcx_als.als_is_ready)
            stk6bcx_als_get_data(client);
        else
            STK_LOG("::ALS is not ready");
    }

    if (stk6bcx_als.als_is_ready)
    {
        stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH0] = MAX_GAIN / (stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH0] * stk6bcx_als.als_cur_again[STK6BCX_ALS_CH0]);
        stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH1] = MAX_GAIN / (stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH1] * stk6bcx_als.als_cur_again[STK6BCX_ALS_CH1]);
        stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH2] = MAX_GAIN / (stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH2] * stk6bcx_als.als_cur_again[STK6BCX_ALS_CH2]);
        stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH3] = MAX_GAIN / (stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH3] * stk6bcx_als.als_cur_again[STK6BCX_ALS_CH3]);
        stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH4] = MAX_GAIN / (stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH4] * stk6bcx_als.als_cur_again[STK6BCX_ALS_CH4]);
        stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH5] = MAX_GAIN / (stk6bcx_als.als_cur_dgain[STK6BCX_ALS_CH5] * stk6bcx_als.als_cur_again[STK6BCX_ALS_CH5]);

        stk6bcx_als.last_raw_data[STK6BCX_ALS_CH0] *= stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH0];
        stk6bcx_als.last_raw_data[STK6BCX_ALS_CH1] *= stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH1];
        stk6bcx_als.last_raw_data[STK6BCX_ALS_CH2] *= stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH2];
        stk6bcx_als.last_raw_data[STK6BCX_ALS_CH3] *= stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH3];
        stk6bcx_als.last_raw_data[STK6BCX_ALS_CH4] *= stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH4];
        stk6bcx_als.last_raw_data[STK6BCX_ALS_CH5] *= stk6bcx_als.als_cur_ratio[STK6BCX_ALS_CH5];
#if (defined(CONFIG_STK_ALS_DRI) || defined(CONFIG_STK_RGB_DRI))
       if ((state->publish_sensors & (STK6BCX_ALS_OC | STK6BCX_RGB_OC)))
        {
            // [Warning] if using RGBC fifo, nust using G ch
            stk6bcx_als_set_thd(client, stk6bcx_als.last_raw_data[2]);
        }
#endif

#ifdef STK_ALS_MID_FIR
        stk6bcx_als_recalculate_filter(client, stk6bcx_als.last_raw_data);
#endif
    }

    memcpy(channel_data, stk6bcx_als.last_raw_data, sizeof(stk6bcx_als.last_raw_data));
    stk6bcx_als.lux = channel_data[0];// * stk6bcx_als.g_scale;
#ifdef STK6BCX_LSC
    kernel_neon_begin();
    stk6bcx_als_compensation(channel_data);
    kernel_neon_end();
#endif
    stk_power(3, 2);//debug
    luxdata_flt = (uint32_t)stk6bcx_als.lux + stk6bcx_als.als_dbg_cnt % 3;
    STK_LOG("lux-CCT-rgbw raw = %d %d %d %d %d %d",
        (uint32_t)stk6bcx_als.lux, (uint32_t)stk6bcx_als.cct,
        stk6bcx_als.last_raw_data[4], stk6bcx_als.last_raw_data[0],
        stk6bcx_als.last_raw_data[5], stk6bcx_als.last_raw_data[1]);


#ifdef STK_ALGO_ENABLE
    DisplayData.PixelR = stk6bcx_als.pixeldata.PixelR ;
    DisplayData.PixelG = stk6bcx_als.pixeldata.PixelG ;
    DisplayData.PixelB = stk6bcx_als.pixeldata.PixelB ;
    DisplayData.Brightness = stk6bcx_als.pixeldata.Brightness;
    memset(&DisplayNoiseData, 0, sizeof(ChannelData));
    STK_calcDisplayNoise(&DisplayNoiseData, &DisplayData);
    //STK_LOG("DisplayNoiseData.ChannelG : %d\n", DisplayNoiseData.ChannelG);
    memset(&AmbientData, 0, sizeof(ChannelData));
    /*
    STK_LOG("raw ALS: %d, G:%d , C:%d, gain =%d , G * gain =%d\n", \
    stk6bcx_als.als_raw_data[0], stk6bcx_als.als_raw_data[1], stk6bcx_als.als_raw_data[2], gain, \
    stk6bcx_als.als_raw_data_u32[1]);
    */
    RawData.ChannelF = stk6bcx_als.als_raw_data_u32[3]; // Channel F
    RawData.ChannelG = stk6bcx_als.als_raw_data_u32[1]; // CHannel G
    STK_calcAmbientInfo(&RawData, &AmbientData, &DisplayData, false);
    luxdata_flt = (float)AmbientData.ChannelG;
    STK_LOG("Ambient ALS: %u, C1: %u, lux =%d, als_scale : %f, calibrated =%d\n", \
                    AmbientData.ChannelF, AmbientData.ChannelG, luxdata_flt,  \
                    stk6bcx_als.als_scale, stk6bcx_als.calibrated);
#endif

#if 0
#ifdef STK_GPIO_ALS
    stk6bcx_gpio_lost_handle(client);
#endif
#endif
    ret = stk6bcx_light_report_abs_value(sensor->input_dev, luxdata_flt);

    return ret;
}

struct sensor_operate stk6bcx_ops = {
    .name                = "ls_stk6bcx",
    .type                = SENSOR_TYPE_LIGHT,    //sensor type and it should be correct
    .id_i2c              = LIGHT_ID_STK6BCX,        //i2c id number
    .read_reg            = STK6BCX_REG_ALS_FIFO_OUT,            //read data
    .read_len            = 2,                //data length
    .id_reg              = STK6BCX_REG_PID,//SENSOR_UNKNOW_DATA,        //read device id from this register
    .id_data             = STK6BCX_PID,//SENSOR_UNKNOW_DATA,        //device id
    .precision           = 16,                //16 bits
    .ctrl_reg            = STK6BCX_REG_ALS_FLAG,            //enable or disable 
    .int_status_reg      = 0x00,            //intterupt status register
    .range               = {2,65535},        //range
    .brightness          = {5,255},     //brightness    
    .trig                = IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,        
    .active              = stk6bcx_sensor_enable,    
    .init                = stk6bcx_sensor_init,
    .report              = stk6bcx_sensor_get_data,
};

static struct sensor_operate *stk6bcx_light_get_ops(void)
{
    return &stk6bcx_ops;
}

static int __init stk6bcx_init(void)
{
    struct sensor_operate *ops = stk6bcx_light_get_ops();
    int result = 0;
    int type = ops->type;
    result = sensor_register_slave(type, NULL, NULL, stk6bcx_light_get_ops);
    return result;
}

static void __exit stk6bcx_exit(void)
{
    struct sensor_operate *ops = stk6bcx_light_get_ops();
    int type = ops->type;
    sensor_unregister_slave(type, NULL, NULL, stk6bcx_light_get_ops);
}


module_init(stk6bcx_init);
module_exit(stk6bcx_exit);
MODULE_AUTHOR("David <David@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk6bcx Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION_STK6BCX);

