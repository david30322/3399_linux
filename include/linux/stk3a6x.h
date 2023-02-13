/*
 *
 * $Id: stk3a6.h
 *
 * Copyright (C) 2012~2013 Lex Hsieh     <lex_hsieh@sensortek.com.tw> 
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef __STK3A6X_H__
#define __STK3A6X_H__

#define DRIVER_VERSION  "1.03.2022"

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
#define STK_PSOFF_REG               0x24
#define STK_PDT_ID_REG             0x3E
#define STK_RSRVD_REG             0x3F
#define STK_GAINCTRL_REG        0x4E
#define STK_PDCTRL_REG           0xA1

#define STK_FIFOCTRL1_REG               0x60
#define STK_FIFOCTRL1_VAL               0x34
#define STK_THD1_FIFO_FCNT_REG          0x61
#define STK_THD2_FIFO_FCNT_REG          0x62
#define STK_FIFOCTRL2_REG               0x63
#define STK_FIFOFCNT1_REG               0x64
#define STK_FIFOFCNT2_REG               0x65
#define STK_FIFO_OUT_REG                0x66
#define STK_FIFO_FLAG_REG               0x67
#define STK_ALSCTRL2_REG                0x6F
#define STK_SW_RESET_REG                0x80

#define STK_FIFO_MAX_FRAME              128//1024
#define STK_FIFO_MAX_LEN                2048

#define STK_STATE_EN_WAIT_MASK    0x04
#define STK_STATE_EN_ALS_MASK    0x02
#define STK_STATE_EN_PS_MASK    0x01

#define STK_FLG_ALSDR_MASK        0x80
#define STK_FLG_PSDR_MASK        0x40
#define STK_FLG_ALSINT_MASK        0x20
#define STK_FLG_PSINT_MASK        0x10
#define STK_FLG_OUI_MASK            0x04
#define STK_FLG_NF_MASK            0x01

#define STK_INT_ALS                0x08

#define STK_ALS_GAIN1                   0x00
#define STK_ALS_GAIN4                   0x10
#define STK_ALS_GAIN16                  0x20
#define STK_ALS_GAIN64                  0x30

#define STK3A6X_PID 0x21

#define STK_FIFO

/*****************************************************************************/
#define STK_MAX_MIN_DIFF            80
#define STK_HT_N_CT                 500
#define STK_LT_N_CT                 400
#define STK_PS_THD_HAND             2500
#define STK_PS_SMG_H                900
#define STK_PS_SMG_L                700
#define STK_THD_SL                  1000

typedef struct stk3a6x_register_table
{
    unsigned   char address;
    unsigned   char value;
} stk3a6x_register_table;

static struct stk3a6x_register_table stk3a6x_config_table[] =
{
    {0x00,  0x00},  
    {0x01,  0x24},
#ifdef STK_FIFO
    {0x02,  0x35}, //
    {STK_FIFOCTRL1_REG,  STK_FIFOCTRL1_VAL},
    {0x61,  0x07},
    {0x62,  0x07},
    {0x6F,  0x82}, //bit7 :0 long als_it; 1:short als_it,;bit4~bit0: short als_it select
#else
    {0x02,  0x04}, //ALS_IT 50ms
    {STK_FIFOCTRL1_REG,  0x00},
    {0x6F,  0x02},
#endif
    {0x03,  0x80},  
    {0x04,  0x00},
    {0x05,  0x1f},
    {0x46,  0x70},
    {0x4E,  0x06}, //128 Gain ALS+C1+C2
     //bit5~bit4:00:FIFO mode off;01: bypass mode;10:FIFO mode;11:stream mode;
                   //bit2~bit0:000:ps only 001:ALS+C1 010:ALS+C2 011:Reserved 100:ALS+C1+C2+PS
    {0xA0,  0x10},
    {0xA1,  0x7F}, 
    {0xDB,  0x01},  
    {0xF6,  0x09}, 
    {0xF1,  0x00},  
};

/* platform data */
struct stk3a6x_platform_data
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
};

#endif 
