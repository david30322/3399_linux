/*
 *
 * $Id: stk3x1x.h
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

/* platform data */
struct stk3x3x_platform_data
{
	uint8_t state_reg;
	uint8_t psctrl_reg;
	uint8_t alsctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t	wait_reg;	
	uint8_t pd_choice_reg;
	uint8_t Again_reg;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	//int int_pin;
	uint32_t transmittance;
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
    {0x01,  0x30}, 
    {0x02,  0x31},
    {0x03,  0x80},  
    {0x04,  0x00},
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
