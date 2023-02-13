/*
 *
 * $Id: stk3x3x.h
 *
 * Copyright (C) 2012~2013 Lex Hsieh     <lex_hsieh@sensortek.com.tw> 
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef __STK3X3X_H__
#define __STK3X3X_H__

#define DRIVER_VERSION  "3.10.0_0429"

/* Define Register Map */
#define STK_STATE_REG               0x00
#define STK_PSCTRL_REG              0x01
#define STK_ALSCTRL_REG             0x02
#define STK_LEDCTRL_REG             0x03
#define STK_INT_REG                 0x04
#define STK_WAIT_REG                0x05
#define STK_THDH1_PS_REG            0x06
#define STK_THDH2_PS_REG            0x07
#define STK_THDL1_PS_REG            0x08
#define STK_THDL2_PS_REG            0x09
#define STK_THDH1_ALS_REG           0x0A
#define STK_THDH2_ALS_REG           0x0B
#define STK_THDL1_ALS_REG           0x0C
#define STK_THDL2_ALS_REG           0x0D
#define STK_FLAG_REG                0x10
#define STK_DATA1_PS_REG            0x11
#define STK_DATA2_PS_REG            0x12
#define STK_DATA1_ALS_REG           0x13
#define STK_DATA2_ALS_REG           0x14
#define STK_PSOFF_REG               0x24
#define STK_PDT_ID_REG              0x3E
#define STK_RSRVD_REG               0x3F
#define STK_GAINCTRL_REG            0x4E
#define STK_SW_RESET_REG            0x80
#define STK_PD_CHOICE               0xA1
#define STK_AGAIN_K                 0xDB

#define STK_STATE_EN_WAIT_MASK      0x04
#define STK_STATE_EN_ALS_MASK       0x02
#define STK_STATE_EN_PS_MASK        0x01

#define STK_FLG_ALSDR_MASK          0x80
#define STK_FLG_PSDR_MASK           0x40
#define STK_FLG_ALSINT_MASK         0x20
#define STK_FLG_PSINT_MASK          0x10
#define STK_FLG_NF_MASK             0x01

#define STK_INT_ALS                 0x08

/*****************************************************************************/
#define STK_MAX_MIN_DIFF            80
#define STK_HT_N_CT                 500
#define STK_LT_N_CT                 400
#define STK_PS_THD_HAND             2500
#define STK_PS_SMG_H                900
#define STK_PS_SMG_L                700
#define STK_THD_SL                  1000

/*****************************************************************************/
#define STK3338_PID                 0x58
#define STK335XX_PID                0x51
/*****************************************************************************/
#define STK_ALS_GAIN1               0x00
#define STK_ALS_GAIN4               0x10
#define STK_ALS_GAIN16              0x20
#define STK_ALS_GAIN64              0x30
/*****************************************************************************/
#define STK_DEBUG_PRINTF
/* platform data */
struct stk3x3x_platform_data
{
    uint8_t state_reg;
    uint8_t psctrl_reg;
    uint8_t alsctrl_reg;
    uint8_t ledctrl_reg;
    uint8_t wait_reg;
    uint8_t pd_choice_reg;
    uint8_t Again_reg;
    uint16_t ps_thd_h;
    uint16_t ps_thd_l;
    //int int_pin;
    uint32_t transmittance;
    uint16_t ps_max_min_diff;
    uint16_t ps_lt_n_ct;
    uint16_t ps_ht_n_ct;
    uint16_t ps_pocket;
    uint16_t ps_hand_h;
    uint16_t ps_smg_h;
    uint16_t ps_smg_l;

    uint16_t stk_max_min_diff;
    uint16_t stk_lt_n_ct;
    uint16_t stk_ht_n_ct;
    };
    
typedef struct stk3x3x_register_table
{
    unsigned   char address;
    unsigned   char value;
} stk3x3x_register_table;

static struct stk3x3x_register_table stk3x3x_config_table[] =
{
    {0x00,  0x00},
    {0x01,  0x32},
    {0x02,  0x30},
    {0x03,  0x80},
    {0x04,  0x01},
    {0x05,  0x1f},
    {0x4E,  0x00},
    {0xA0,  0x10},
    {0xAA,  0x64},
    {0xA1,  0x7F},
    {0xDB,  0x01},
    {0xF6,  0x82},
    {0xFA,  0x01},
    {0x81,  0x60},
};

#endif // __STK3X3X_H__
