/*
 *
 * $Id: w1160.h
 *
 * Copyright (C) 2012~2013 Lex Hsieh     <lex_hsieh@sensortek.com.tw> 
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef __W1160_H__
#define __W1160_H__

/* platform data */
struct w1160_platform_data
{
    uint8_t state_reg;
    uint8_t alsctrl_reg;
    uint8_t Again_reg;
};
    
typedef struct w1160_register_table
{
    unsigned   char address;
    unsigned   char value;
} w1160_register_table;

#endif // __STK3X3X_H__
