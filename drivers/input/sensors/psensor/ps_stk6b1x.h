#ifndef _PROXIMITY_STK6B1X_SENSOR_DRIVER_H_
#define _PROXIMITY_STK6B1X_SENSOR_DRIVER_H_

#define STK6B1X_GPIO_PS
#define STK_TAG                  "[PS] "
#define STK_FUN(f)               printk(STK_TAG" %s\n", __FUNCTION__)
//#define STK_ERR(fmt, args...)    printf(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define STK_LOG(fmt, args...)    printk(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
//#define STK_DBG(fmt, args...)    printf(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define NO_ERROR 0
#define FAIL (-1)

/*Reg*/
#define STK6B1X_REG_ENABLE                  0x00
#define STK6B1X_REG_FSM_CTRL                0x01
#define STK6B1X_REG_SWRST                   0x02
#define STK6B1X_REG_PS_DGAIN                0x03
#define STK6B1X_REG_PS_AGAIN                0x04
#define STK6B1X_REG_PS_IT                   0x05
#define STK6B1X_REG_PS_WAIT_SEL             0x06
#define STK6B1X_REG_PS_WAIT1                0x07
#define STK6B1X_REG_PS_WAIT2                0x08
#define STK6B1X_REG_PS_LED_SET              0x09
#define STK6B1X_REG_PS_DATA1_OFFSET         0x0D
#define STK6B1X_REG_PS_DATA2_OFFSET         0x0E
#define STK6B1X_REG_PS_PRST                 0x0F
#define STK6B1X_REG_PS_COMP                 0x50
#define STK6B1X_REG_PS_THDH1                0x20
#define STK6B1X_REG_PS_THDH2                0x21
#define STK6B1X_REG_PS_THDL1                0x22
#define STK6B1X_REG_PS_THDL2                0x23
#define STK6B1X_REG_PS_OFF_THDH1            0x28
#define STK6B1X_REG_PS_OFF_THDH2            0x29
#define STK6B1X_REG_PS_BGIR_THDH1           0x2A
#define STK6B1X_REG_PS_BGIR_THDH2           0x2B
#define STK6B1X_REG_INTCTRL1                0x2C
#define STK6B1X_REG_INTCTRL2                0x2D
#define STK6B1X_REG_ALS_FLAG                0x30
#define STK6B1X_REG_PID                     0x3E
#define STK6B1X_REG_PS_FLAG1                0x40
#define STK6B1X_REG_PS_FLAG2                0x41
#define STK6B1X_REG_PS_DATA1                0x42
#define STK6B1X_REG_PS_DATA2                0x43
#define STK6B1X_REG_PS_OFF_DATA1            0x44
#define STK6B1X_REG_PS_OFF_DATA2            0x45
#define STK6B1X_REG_PS_BGIR_DATA1           0x4A
#define STK6B1X_REG_PS_BGIR_DATA2           0x4B
#define STK6B1X_REG_PS_BGIR_SET0            0x53
#define STK6B1X_REG_PS_BGIR_SET1            0x54
#define STK6B1X_REG_PS_SUM_MODE_SET0        0x59
#define STK6B1X_REG_PS_SUM_MODE_SET1        0x5A
#define STK6B1X_REG_FIFO_SET0               0x63
#define STK6B1X_REG_FIFO_SET1               0x64
#define STK6B1X_REG_FIFO_SET2               0x65
#define STK6B1X_REG_FIFO_SET3               0x66
#define STK6B1X_REG_FIFO_SET4               0x67
#define STK6B1X_REG_GPIO_SET0               0x68
#define STK6B1X_REG_GPIO_SET1               0x6A
#define STK6B1X_REG_GPIO_SET2               0x6B
#define STK6B1X_REG_GPIO_SET3               0x6C
#define STK6B1X_REG_GPIO_SET4               0x6F
#define STK6B1X_REG_GPIO_SET5               0x70
#define STK6B1X_REG_GPIO_SET6               0x71
#define STK6B1X_REG_GPIO_SET7               0x72
#define STK6B1X_REG_GPIO_SET8               0x73
#define STK6B1X_REG_GPIO_SET9               0x74
#define STK6B1X_REG_GPIO_SET10              0x78
#define STK6B1X_REG_GPIO_SET11              0x79
#define STK6B1X_REG_GPIO_SET12              0x7A
#define STK6B1X_REG_GPIO_SET13              0x7B
#define STK6B1X_REG_GPIO_SET14              0x7C
#define STK6B1X_REG_GPIO_SET15              0x7D
#define STK6B1X_REG_GPIO_SET16              0x7E
#define STK6B1X_REG_GPIO_SET17              0x81
#define STK6B1X_REG_GPIO_SET18              0x82
#define STK6B1X_REG_GPIO_SET19              0x83
#define STK6B1X_REG_GPIO_SET20              0x84
#define STK6B1X_REG_GPIO_SET21              0x85
#define STK6B1X_REG_GPIO_SET22              0x86
#define STK6B1X_REG_GPIO_SET23              0x87
#define STK6B1X_REG_GPIO_SET24              0x89
#define STK6B1X_REG_GPIO_SET25              0x8A
#define STK6B1X_REG_GPIO_SET26              0x8B
#define STK6B1X_REG_GPIO_SET27              0x8D
#define STK6B1X_REG_GPIO_SET28              0x8E
#define STK6B1X_REG_FSM_MODE                0xB0
#define STK6B1X_REG_PD_SEL                  0xB1

/* Define FSM Ctrl reg*/
#define STK6B1X_FSM_FLG_RST_SHIFT           7
#define STK6B1X_FSM_PS_PAUSE_SHIFT          4
#define STK6B1X_FSM_PS_FUNC_RESTART_SHIFT   1
#define STK6B1X_FSM_FLG_RST_MASK            0x80
#define STK6B1X_FSM_PS_PAUSE_MASK           0x10
#define STK6B1X_FSM_PS_FUNC_RESTART_MASK    0x04

/* Define state reg */
#define STK6B1X_STATE_EN_PS_WAIT_MASK       0x04
#define STK6B1X_STATE_EN_PS_MASK            0x01

/* Define PS flag1 reg[0x40] */
#define STK6B1X_FLG_PSDR_MASK               0x80
#define STK6B1X_FLG_PS_INT_MASK             0x40
#define STK6B1X_FLG_INVALID_PS_MASK         0x10

/* Define Pag2 reg */
#define STK6B1X_FLG_NF_MASK                 0x01

/* Define PS DG reg */
#define STK6B1X_PS_DGAIN_SHIFT              0
#define STK6B1X_PS_DGAIN_MASK               0x0F

#define STK6B1X_PS_GAIN1                    0x00
#define STK6B1X_PS_GAIN2                    0x01
#define STK6B1X_PS_GAIN4                    0x02
#define STK6B1X_PS_GAIN8                    0x03
#define STK6B1X_PS_GAIN16                   0x04
#define STK6B1X_PS_GAIN32                   0x05
#define STK6B1X_PS_GAIN64                   0x06
#define STK6B1X_PS_GAIN128                  0x07
#define STK6B1X_PS_GAIN256                  0x08

/* Define PS LED SET reg[0x09] */
#define STK6B1X_PS_IRDR_MAKS                0x1F
#define STK6B1X_PS_IRDR_5_46mA              0x06
#define STK6B1X_PS_IRDR_12_5mA              0x0F

/* Define PS AG reg */
#define STK6B1X_PS_AGAIN_SHIFT              0
#define STK6B1X_PS_AGAIN_MASK               0x03

#define STK6B1X_PS_AGAIN_2_0                0x00
#define STK6B1X_PS_AGAIN_1_0                0x01
#define STK6B1X_PS_AGAIN_0_5                0x02
#define STK6B1X_PS_AGAIN_0_25               0x03

/* Define PS IT reg*/
#define STK6B1X_PS_IT_SHIFT                 0
#define STK6B1X_PS_IT_MASK                  0x0F

#define STK6B1X_PS_IT12                     0x00
#define STK6B1X_PS_IT24                     0x01
#define STK6B1X_PS_IT48                     0x02
#define STK6B1X_PS_IT96                     0x03
#define STK6B1X_PS_IT192                    0x04
#define STK6B1X_PS_IT384                    0x05
#define STK6B1X_PS_IT768                    0x06
#define STK6B1X_PS_IT1536                   0x07
#define STK6B1X_PS_IT3072                   0x08
#define STK6B1X_PS_IT6144                   0x09

#define STK6B1X_IRDR_0_78mA                 0x00
#define STK6B1X_IRDR_3_9mA                  0x01
#define STK6B1X_IRDR_6_25mA                 0x07
#define STK6B1X_IRDR_9_38mA                 0x0B

/* Define PS PRST reg*/
#define STK6B1X_PS_PRST_MASK                0x7
#define STK6B1X_PS_PRST1                    0x0
#define STK6B1X_PS_PRST2                    0x1
#define STK6B1X_PS_PRST4                    0x2
#define STK6B1X_PS_PRST8                    0x3
#define STK6B1X_PS_PRST16                   0x4

/* Define PS Wait reg*/
#define STK6B1X_PS_WAIT_SEL_SHIFT           7
#define STK6B1X_PS_WAIT_SEL_21US            (0x00 << STK6B1X_PS_WAIT_SEL_SHIFT) //dedault
#define STK6B1X_PS_WAIT_SEL_3US             (0x01 << STK6B1X_PS_WAIT_SEL_SHIFT)

#define STK6B1X_PS_WAIT20                   0x3B8
#define STK6B1X_PS_WAIT50                   0x94D
#define STK6B1X_PS_WAIT100                  0x129A

#define STK6B1X_WAIT_H(X)                   ((X >> 8) & 0xFF)
#define STK6B1X_WAIT_L(X)                   (X & 0xFF)

#define STK6B1X_PS_BGIR_THRESHOLD           0x64

#define PROX_STATE_NEAR                 0
#define PROX_STATE_FAR                  1
#define STK6B1X_PS_CALI_DATA_NUM            3

#define STK6B1X_LT_N_CT                     1700
#define STK6B1X_HT_N_CT                     2200
#define STK6B1X_DEFAULT_CT                  6000
#define STK6B1X_PS_BOOT_THD_RATIO           2
#define STK6B1X_PS_SMUDGE_RATIO             3
#define STK6B1X_TRACKING_QUANTI             4
#define STK6B1X_QUANTI_RANGE                10
#define STK6B1X_SMUDGE_DIFF                 300
#define STK6B1X_SMUDGE_NT                   3000
#define STK6B1X_SMUDGE_FT                   2500
#define STK6B1X_MAX_MIN_DIFF                200

#define STK6B1X_TC_TRACKING_TIME            2000    // ms, Set 0 to tune off.
#define STK6B1X_CT_FIR_LEN                  6       // >=6, 6, 8, 10 ...
#define STK6B1X_TC_MAX_MIN_DIFF             50
#define STK6B1X_TC_SLOPE_THD                15      // THD for (last_avg - pre_avg)
#define STK6B1X_TC_CT_DIFF                  5       // Compensation THD for CT

#ifdef STK6B1X_GPIO_PS
    /* Define GPIO SET0 reg */
    #define STK6B1X_GPIO_EDGE_SEL_SHIFT                     7
    #define STK6B1X_GPIO_EDGE_SEL_N_EDGE                    (0x00 << STK6B1X_GPIO_EDGE_SEL_SHIFT)
    #define STK6B1X_GPIO_EDGE_SEL_P_EDGE                    (0x01 << STK6B1X_GPIO_EDGE_SEL_SHIFT)

    #define STK6B1X_GPIO_TD_MODE_SHIFT                      4
    #define STK6B1X_GPIO_TD_MODE_ABS                        (0x00 << STK6B1X_GPIO_TD_MODE_SHIFT)
    #define STK6B1X_GPIO_TD_MODE_RATIO                      (0x01 << STK6B1X_GPIO_TD_MODE_SHIFT)

    #define STK6B1X_GPIO_PS_SEL_MASK                        0x04
    #define STK6B1X_GPIO_ALS_SEL_MASK                       0x01

    /* Define GPIO SET1 reg */
    #define STK6B1X_GPIO_EN_PSEUDO_MASK                     0x08
    #define STK6B1X_GPIO_EN_MEASURE_MASK                    0x04

    /* Define GPIO SET2 reg */
    #define STK6B1X_GPIO_EN_FREQ_LOST_INT_MASK              0x04
    #define STK6B1X_GPIO_EN_TIMER_INT_MASK                  0x01

    /* Define GPIO SET3 reg */
    #define STK6B1X_GPIO_DEB_SEL_BOTH                       0x00
    #define STK6B1X_GPIO_DEB_SEL_RISING                     0x01
    #define STK6B1X_GPIO_DEB_SEL_FALLING                    0x02
    #define STK6B1X_GPIO_DEB_SEL_DISABLE                    0x03

    /* Define FIFO SET26 reg */
    #define STK6B1X_FLG_GPIO_FREQ_LOST_INT_MASK             0x80
    #define STK6B1X_FLG_GPIO_TIMER_INT_MASK                 0x40
    #define STK6B1X_FLG_GPIO_FREQ_LOST_MASK                 0x20
    #define STK6B1X_FLG_GPIO_TIMER_MASK                     0x10

    #define STK6B1X_GPIO_FREQ_LOST_THD_1_PERCENT            0x00
    #define STK6B1X_GPIO_FREQ_LOST_THD_5_PERCENT            0x06
    #define STK6B1X_GPIO_FREQ_LOST_THD_10_PERCENT           0x0C

    #define STK6B1X_GPIO_TIMER_10MS                         0x34
    #define STK6B1X_GPIO_TIMER_15MS                         0x4E
    #define STK6B1X_GPIO_TIMER_20MS                         0x68

    #define STK6B1X_GPIO_TD_TIMER(US)                       ((US * 4) / 3) // /0.75
#endif

typedef struct
{
    uint16_t raw[STK6B1X_CT_FIR_LEN];
    //uint16_t number;
    uint16_t idx;
    uint16_t max;
    uint16_t min;
}stk_ps_data_filter;

enum PROX_STATE{
    PS_UNKNOWN = -1,
    PS_NEAR = 0,
    PS_FAR = 1,
};

struct stk6b1x_ps_data
{
    uint16_t ps_last_raw_data;
    int      ps_last_status;
    uint16_t ps_thd_h;
    uint16_t ps_thd_l;
    uint16_t fac_ct;
    uint16_t psa;
    uint16_t psi;
    uint16_t last_ps_psi;
    uint16_t psi_set;
    uint32_t ps_stat_data[3];
    uint16_t ht_n_ct;
    uint16_t lt_n_ct;
    uint16_t smudge_update;
    uint8_t  set_thd;
    uint16_t data_count;
    int ps_distance_last;
    bool ps_need_report;
    bool first_report；
    uint16_t tracking_time; // tune0 period time
    
    uint16_t compensation_target;
    uint16_t compensation_cnt;
    stk_ps_data_filter ps_data_tc_filter;
#ifdef STK6B1X_GPIO_PS
    uint8_t display_freq;
    bool gpio_enable;
#endif
};

typedef struct stk6b1x_register_table
{
    uint8_t address;
    uint8_t value;
    uint8_t mask;
} stk6b1x_register_table;

#endif //_PROXIMITY_STK6B1X_SENSOR_DRIVER_H_

