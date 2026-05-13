#ifndef _PROXIMITY_STK6BCX_SENSOR_DRIVER_H_
#define _PROXIMITY_STK6BCX_SENSOR_DRIVER_H_

#define STK6BCX_GPIO_PS
#define STK_TAG                  "[PS] "
#define STK_FUN(f)               printk(STK_TAG" %s\n", __FUNCTION__)
//#define STK_ERR(fmt, args...)    printf(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define STK_LOG(fmt, args...)    printk(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
//#define STK_DBG(fmt, args...)    printf(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define NO_ERROR 0
#define FAIL (-1)

/*Reg*/
#ifndef STK6BCX_ENABLE_TEST_CODE
    #define STK6BCX_ENABLE_TEST_CODE                   1
#endif

// Enable when Timer, Registry, ACP dependencies are available
#ifndef STK6BCX_ENABLE_DEPENDENCY
    #define STK6BCX_ENABLE_DEPENDENCY                  0
#endif

#ifndef STK6BCX_USE_DEFAULTS
    #define STK6BCX_USE_DEFAULTS                       1
#endif

/**
 *  Address registers
 */
#define STK6BCX_REG_ENABLE                             0x00
#define STK6BCX_REG_FSM_CTRL                           0x01
#define STK6BCX_REG_SWRST                              0x02
#define STK6BCX_REG_PS_DGAIN                           0x03
#define STK6BCX_REG_PS_AGAIN                           0x04
#define STK6BCX_REG_PS_IT1                             0x05
#define STK6BCX_REG_PS_IT2                             0x06
#define STK6BCX_REG_PS_WAIT1                           0x07
#define STK6BCX_REG_PS_WAIT2                           0x08
#define STK6BCX_REG_PS_LED_SET                         0x09
#define STK6BCX_REG_PS_DATA_OFFSET1                    0x0D
#define STK6BCX_REG_PS_DATA_OFFSET2                    0x0E
#define STK6BCX_REG_PS_PRST                            0x0F
#define STK6BCX_REG_ALS_DGAIN                          0x10
#define STK6BCX_REG_ALS_DGAIN1                         0x11
#define STK6BCX_REG_ALS_DGAIN2                         0x12
#define STK6BCX_REG_ALS_AGAIN                          0x13
#define STK6BCX_REG_ALS_AGAIN1                         0x14
#define STK6BCX_REG_ALS_AGAIN2                         0x15
#define STK6BCX_REG_ALS_IT_SET0                        0x17
#define STK6BCX_REG_ALS_IT_SET1                        0x18
#define STK6BCX_REG_ALS_IT_SET2                        0x19
#define STK6BCX_REG_ALS_WAIT1                          0x1A
#define STK6BCX_REG_ALS_WAIT2                          0x1B
#define STK6BCX_REG_NPST_SET1                          0x1D
#define STK6BCX_REG_NPST_SET2                          0x1E
#define STK6BCX_REG_ALS_PRST                           0x1F
#define STK6BCX_REG_PS_THDH1                           0x20
#define STK6BCX_REG_PS_THDH2                           0x21
#define STK6BCX_REG_PS_THDL1                           0x22
#define STK6BCX_REG_PS_THDL2                           0x23
#define STK6BCX_REG_ALS_THDH1                          0x24
#define STK6BCX_REG_ALS_THDH2                          0x25
#define STK6BCX_REG_ALS_THDL1                          0x26
#define STK6BCX_REG_ALS_THDL2                          0x27
#define STK6BCX_REG_PS_OFF_THDH1                       0x28
#define STK6BCX_REG_PS_OFF_THDH2                       0x29
#define STK6BCX_REG_PS_BGIR_THDH1                      0x2A
#define STK6BCX_REG_PS_BGIR_THDH2                      0x2B
#define STK6BCX_REG_INT_CTRL1                          0x2C
#define STK6BCX_REG_INT_CTRL2                          0x2D
#define STK6BCX_REG_ALS_FLAG                           0x30
#define STK6BCX_REG_ALS_FIFO_FLAG                      0x37
#define STK6BCX_REG_ALS_FIFO_CNT1                      0x38
#define STK6BCX_REG_ALS_FIFO_CNT2                      0x39
#define STK6BCX_REG_ALS_FIFO_OUT                       0x3C
#define STK6BCX_REG_FLK_FIFO_OUT                       0x3D
#define STK6BCX_REG_PS_FLAG1                           0x40
#define STK6BCX_REG_PS_FLAG2                           0x41
#define STK6BCX_REG_PS_DATA1                           0x42
#define STK6BCX_REG_PS_DATA2                           0x43
#define STK6BCX_REG_PS_OFF_DATA1                       0x44
#define STK6BCX_REG_PS_OFF_DATA2                       0x45
#define STK6BCX_REG_PS_BGIR_DATA1                      0x4A
#define STK6BCX_REG_PS_BGIR_DATA2                      0x4B
#define STK6BCX_REG_PS_BGIR_SET                        0x53
#define STK6BCX_REG_PS_SUM_MODE_SET0                   0x59
#define STK6BCX_REG_PS_SUM_MODE_SET1                   0x5A
#define STK6BCX_REG_FIFO_SET0                          0x63
#define STK6BCX_REG_FIFO_SET1                          0x64
#define STK6BCX_REG_FIFO_SET2                          0x65
#define STK6BCX_REG_FIFO_SET3                          0x66
#define STK6BCX_REG_FIFO_SET4                          0x67
#define STK6BCX_REG_GPIO_SET0                          0x68
#define STK6BCX_REG_GPIO_SET1                          0x6A
#define STK6BCX_REG_GPIO_SET2                          0x6B
#define STK6BCX_REG_GPIO_SET3                          0x6C
#define STK6BCX_REG_GPIO_SET4                          0x6F
#define STK6BCX_REG_GPIO_SET5                          0x70
#define STK6BCX_REG_GPIO_SET6                          0x71
#define STK6BCX_REG_GPIO_SET7                          0x72
#define STK6BCX_REG_GPIO_SET8                          0x73
#define STK6BCX_REG_GPIO_SET9                          0x74
#define STK6BCX_REG_GPIO_SET10                         0x75
#define STK6BCX_REG_GPIO_SET11                         0x76
#define STK6BCX_REG_GPIO_SET12                         0x77
#define STK6BCX_REG_GPIO_SET13                         0x78
#define STK6BCX_REG_GPIO_SET14                         0x79
#define STK6BCX_REG_GPIO_SET15                         0x7A
#define STK6BCX_REG_GPIO_SET16                         0x7B
#define STK6BCX_REG_GPIO_SET17                         0x7C
#define STK6BCX_REG_GPIO_SET18                         0x7D
#define STK6BCX_REG_GPIO_SET19                         0x7E
#define STK6BCX_REG_GPIO_SET20                         0x81
#define STK6BCX_REG_GPIO_SET21                         0x82
#define STK6BCX_REG_GPIO_SET22                         0x83
#define STK6BCX_REG_GPIO_SET23                         0x84
#define STK6BCX_REG_GPIO_SET24                         0x85
#define STK6BCX_REG_GPIO_SET25                         0x86
#define STK6BCX_REG_GPIO_SET26                         0x87
#define STK6BCX_REG_GPIO_SET27                         0x89
#define STK6BCX_REG_GPIO_SET28                         0x8A
#define STK6BCX_REG_GPIO_SET29                         0x8B
#define STK6BCX_REG_GPIO_SET30                         0x8D
#define STK6BCX_REG_GPIO_SET31                         0x8E
#define STK6BCX_REG_FSM_MODE                           0xB0
#define STK6BCX_REG_PD_SEL                             0xB1
#define STK6BCX_REG_PS_IRDR_DIV                        0xC3
#define STK6BCX_REG_PS_IRDR_DIS                        0xC4
#define STK6BCX_REG_FIFO_FLK_SET0                      0xB2
#define STK6BCX_REG_FIFO_FLK_SET1                      0xB3
#define STK6BCX_REG_FIFO_FLK_SET2                      0xB4
#define STK6BCX_REG_FIFO_FLK_SET3                      0xB5
#define STK6BCX_REG_FIFO_FLK_SET4                      0xB6
#define STK6BCX_REG_FIFO_FLK_SET5                      0xB7
#define STK6BCX_REG_FIFO_FLK_SET6                      0xB8
#define STK6BCX_REG_PID                                0xFF



/* Define ENABLE reg */
#define  STK6BCX_STATE_EN_PWR_SHIFT                    7
#define  STK6BCX_STATE_EN_FLK_SHIFT                    4
#define  STK6BCX_STATE_EN_ALS_WAIT_SHIFT               3
#define  STK6BCX_STATE_EN_PS_WAIT_SHIFT                2
#define  STK6BCX_STATE_EN_ALS_SHIFT                    1
#define  STK6BCX_STATE_EN_PS_SHIFT                     0
#define  STK6BCX_STATE_EN_PWR_MASK                     0x80
#define  STK6BCX_STATE_EN_FLK_MASK                     0x10
#define  STK6BCX_STATE_EN_ALS_WAIT_MASK                0x08
#define  STK6BCX_STATE_EN_PS_WAIT_MASK                 0x04
#define  STK6BCX_STATE_EN_ALS_MASK                     0x02
#define  STK6BCX_STATE_EN_PS_MASK                      0x01

/* Define FSM ctrl reg */
#define  STK6BCX_FLG_RST_SHIFT                         7
#define  STK6BCX_ALS_FSM_PAUSE_SHIFT                   5
#define  STK6BCX_PS_FSM_PAUSE_SHIFT                    4
#define  STK6BCX_PS_FUNC_RESTART_SHIFT                 2
#define  STK6BCX_ALS_FUNC_RESTART_SHIFT                1
#define  STK6BCX_FLG_RST_MASK                          0x80
#define  STK6BCX_ALS_FSM_PAUSE_MASK                    0x20
#define  STK6BCX_PS_FSM_PAUSE_MASK                     0x10
#define  STK6BCX_PS_FUNC_RESTART_MASK                  0x40
#define  STK6BCX_ALS_FUNC_RESTART_MASK                 0x20

#define  STK6BCX_ALPS_REG_H(X,MASK)                    ((X >> 8) & MASK)
#define  STK6BCX_ALPS_REG_L(X)                         (X & 0xFF)

/* Define PS parameters */
#define  STK6BCX_PS_GAIN1                              0x00
#define  STK6BCX_PS_GAIN2                              0x01
#define  STK6BCX_PS_GAIN4                              0x02
#define  STK6BCX_PS_GAIN8                              0x03
#define  STK6BCX_PS_GAIN16                             0x04
#define  STK6BCX_PS_GAIN32                             0x05
#define  STK6BCX_PS_GAIN64                             0x06
#define  STK6BCX_PS_GAIN128                            0x07
#define  STK6BCX_PS_GAIN256                            0x08
#define  STK6BCX_PS_GAIN_MASK                          0x0F

#define  STK6BCX_PS_CI4                                0x00
#define  STK6BCX_PS_CI2                                0x01
#define  STK6BCX_PS_CI_MASK                            0x01

#define  STK6BCX_PS_IT95                               0x5F
#define  STK6BCX_PS_IT100                              0x63
#define  STK6BCX_PS_IT200                              0xC7
#define  STK6BCX_PS_IT400                              0x18F
#define  STK6BCX_PS_IT800                              0x31F
#define  STK6BCX_PS_IT2_H_MAKS                         0x03

#define  STK6BCX_WAIT_0                                0x0
#define  STK6BCX_WAIT_1                              0x2F
#define  STK6BCX_WAIT_20                               0x3B8  //20ms
#define  STK6BCX_WAIT_50                               0x94D
#define  STK6BCX_WAIT_100                              0x129A
#define  STK6BCX_WAIT2_H_MAKS                          0xFF

#define  STK6BCX_LED_0_5mA                             0x00
#define  STK6BCX_LED_1mA                               0x01
#define  STK6BCX_LED_1_5mA                             0x02
#define  STK6BCX_LED_2mA                               0x03
#define  STK6BCX_LED_2_5mA                             0x04
#define  STK6BCX_LED_3mA                               0x05
#define  STK6BCX_LED_3_5mA                             0x06
#define  STK6BCX_LED_4mA                               0x07
#define  STK6BCX_LED_4_5mA                             0x08
#define  STK6BCX_LED_5mA                               0x09
#define  STK6BCX_LED_9mA                               0x11
#define  STK6BCX_LED_10mA                              0x13
#define  STK6BCX_LED_12_5mA                            0x18
#define  STK6BCX_LED_15mA                              0x1D
#define  STK6BCX_LED_20mA                              0x27
#define  STK6BCX_LED_MASK                              0x3F

#define  STK6BCX_PS_PRST1                              0x00
#define  STK6BCX_PS_PRST2                              0x01
#define  STK6BCX_PS_PRST4                              0x02
#define  STK6BCX_PS_PRST8                              0x03
#define  STK6BCX_PS_PRST16                             0x04
#define  STK6BCX_PS_PRST_MASK                          0x07

/* Define ALS parameters */
#define  STK6BCX_ALS_GAIN1                             0x00
#define  STK6BCX_ALS_GAIN2                             0x01
#define  STK6BCX_ALS_GAIN4                             0x02
#define  STK6BCX_ALS_GAIN8                             0x03
#define  STK6BCX_ALS_GAIN16                            0x04
#define  STK6BCX_ALS_GAIN32                            0x05
#define  STK6BCX_ALS_GAIN64                            0x06
#define  STK6BCX_ALS_GAIN128                           0x07
#define  STK6BCX_ALS_GAIN256                           0x08
#define  STK6BCX_ALS_GAIN512                           0x09
#define  STK6BCX_ALS_GAIN1024                          0x0A0
#define  STK6BCX_ALS_GAIN2048                          0xB0

#define  STK6BCX_ALS0_GAIN_MASK                        0xF0
#define  STK6BCX_ALS0_GAIN_SHIFT                       4
#define  STK6BCX_ALS1_GAIN_MASK                        0x0F
#define  STK6BCX_ALS1_GAIN_SHIFT                       0
#define  STK6BCX_ALS2_GAIN_MASK                        0xF0
#define  STK6BCX_ALS2_GAIN_SHIFT                       4
#define  STK6BCX_ALS3_GAIN_MASK                        0x0F
#define  STK6BCX_ALS3_GAIN_SHIFT                       0
#define  STK6BCX_ALS4_GAIN_MASK                        0xF0
#define  STK6BCX_ALS4_GAIN_SHIFT                       4
#define  STK6BCX_ALS5_GAIN_MASK                        0x0F
#define  STK6BCX_ALS5_GAIN_SHIFT                       0

#define  STK6BCX_ALS_CI_4                              0x00
#define  STK6BCX_ALS_CI_2                              0x01
#define  STK6BCX_ALS_CI_1                              0x02
#define  STK6BCX_ALS_CI_0_5                            0x03
#define  STK6BCX_ALS_CI_0_25                           0x04

#define  STK6BCX_ALS0_CI_MAKS                          0x70
#define  STK6BCX_ALS0_CI_SHIFT                         4
#define  STK6BCX_ALS1_CI_MAKS                          0x07
#define  STK6BCX_ALS1_CI_SHIFT                         0
#define  STK6BCX_ALS2_CI_MAKS                          0x70
#define  STK6BCX_ALS2_CI_SHIFT                         4
#define  STK6BCX_ALS3_CI_MAKS                          0x07
#define  STK6BCX_ALS3_CI_SHIFT                         0
#define  STK6BCX_ALS4_CI_MAKS                          0x70
#define  STK6BCX_ALS4_CI_SHIFT                         4
#define  STK6BCX_ALS5_CI_MAKS                          0x07
#define  STK6BCX_ALS5_CI_SHIFT                         0

#define  STK6BCX_ALS_IT_SEL_1US                        0x00
#define  STK6BCX_ALS_IT_SEL_8US                        0x01
#define  STK6BCX_ALS_IT_SEL_MASK                       0x01

#define  STK6BCX_ALS_IT_BASE_SEL                       STK6BCX_ALS_IT_SEL_8US
#define  STK6BCX_ALS_GET_IT_BASE_UNIT                  ((STK6BCX_ALS_IT_BASE_SEL & STK6BCX_ALS_IT_SEL_8US) ? 8 : 1)
//for ALS IT SEL 1us
#define  STK6BCX_ALS_IT_25US                           ((25 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_50US                           ((50 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_100US                          ((100 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_150US                          ((150 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_200US                          ((200 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_250US                          ((250 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_300US                          ((300 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_350US                          ((350 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_400US                          ((400 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)

#define  STK6BCX_ALS_IT_10MS                           ((10000 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_25MS                           ((25000 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_50MS                           ((50000 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_100MS                          ((100000 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_150MS                          ((150000 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)
#define  STK6BCX_ALS_IT_200MS                          ((200000 / STK6BCX_ALS_GET_IT_BASE_UNIT) - 1)

#define  STK6BCX_ALS_PRST1                             0x00
#define  STK6BCX_ALS_PRST2                             0x01
#define  STK6BCX_ALS_PRST4                             0x02
#define  STK6BCX_ALS_PRST8                             0x03
#define  STK6BCX_ALS_PRST_MASK                         0xC0
#define  STK6BCX_ALS_PRST_SHIFT                        6

/* Define INT ctrl reg */
#define  STK6BCX_ALS_DR_INT_EN                         0x02
#define  STK6BCX_PS_DR_INT_EN                          0x01
#define  STK6BCX_PS_INVALID_INT_EN                     0x10
#define  STK6BCX_PS_INT_MODE                           0x08
#define  STK6BCX_PS_NF_MODE                            0x04
#define  STK6BCX_ALS_INT_EN                            0x02
#define  STK6BCX_PS_INT_EN                             0x01

/* Define ALS FLAG reg */
#define  STK6BCX_FLG_ALS_DR_MASK                       0x80
#define  STK6BCX_FLG_ALS_INT_MASK                      0x40
#define  STK6BCX_FLG_ALS_SAT_MAKS                      0x20

/* Define ALS FIFO FLAG reg*/
#define  STK6BCX_FLG_FIFO_OVR_MAKS                     0x40
#define  STK6BCX_FLG_FIFO_WM_MAKS                      0x20
#define  STK6BCX_FLG_FIFO_FULL_MAKS                    0x10

/* Define PS FLAG reg*/
#define  STK6BCX_FLG_PS_DR_MASK                        0x80
#define  STK6BCX_FLG_PS_INT_MASK                       0x40
#define  STK6BCX_FLG_PS_INVALID_INT_MASK               0x20
#define  STK6BCX_FLG_PS_INVALID_MASK                   0x10
#define  STK6BCX_FLG_PS_NF_MASK                        0x01

/* Define PS BGIR reg */
#define  STK6BCX_PS_BGIR_EN_MASK                       0x01

/* Define PS SUM mode */
#define  STK6BCX_PS_SIM_EN_MASK                        0x01
#define  STK6BCX_PS_SUM_MASK                           0xC0
#define  STK6BCX_PS_SUM1                               0x00
#define  STK6BCX_PS_SUM2                               0x40
#define  STK6BCX_PS_SUM4                               0x80
#define  STK6BCX_PS_AVG_MASK                           0x30
#define  STK6BCX_PS_AVG1                               0x00
#define  STK6BCX_PS_AVG2                               0x10
#define  STK6BCX_PS_AVG4                               0x20

/* Define FIFO reg */
#define  STK6BCX_FIFO_DATA_SEL_ALS012345               0x00
#define  STK6BCX_FIFO_DATA_SEL_STA012345_ALS012345     0x10
#define  STK6BCX_FIFO_DATA_SEL_STAR012345_ALSR012345   0x20
#define  STK6BCX_FIFO_DATA_SEL_TS_SI_ALSr012345        0x30
#define  STK6BCX_FIFO_DATA_SEL_MASK                    0x70

#define  STK6BCX_FIFO_MODE_OFF                         0x00
#define  STK6BCX_FIFO_MODE_BYPASS                      0x01
#define  STK6BCX_FIFO_MODE_FIFO                        0x02
#define  STK6BCX_FIFO_MODE_STREAM                      0x03
#define  STK6BCX_FIFO_MODE_MASK                        0x03

#define  STK6BCX_FLG_FIFO_FLUSH                        0x02
#define  STK6BCX_FLG_FIFO_PAUSE                        0x01

#define  STK6BCX_FIFO_FOVR_EN                          0x04
#define  STK6BCX_FIFO_FWM_EN                           0x02
#define  STK6BCX_FIFO_FFULL_EN                         0x01

/* Define GPIO reg */
#define  STK6BCX_GPIO_SEEK_EN                          0x40

#define  STK6BCX_GPIO_TD_MODE_MASK                     0x10
#define  STK6BCX_GPIO_TD_MODE_ABS                      0x00
#define  STK6BCX_GPIO_TD_MODE_RATIO                    0x10

#define  STK6BCX_GPIO_PS_EN                            0x04
#define  STK6BCX_GPIO_ALS_EN                           0x01

#define  STK6BCX_GPIO_PSEUDO_EN                        0x08
#define  STK6BCX_GPIO_MEASURE_EN                       0x04
#define  STK6BCX_GPIO_FREQ_LOST_INT_EN                 0x04
#define  STK6BCX_GPIO_TIMER_INT_EN                     0x01

#define  STK6BCX_GPIO_DEB_TIME_MASK                    0xF8
#define  STK6BCX_GPIO_DEB_SEL_MASK                     0x04
#define  STK6BCX_GPIO_DEB_SEL_BOTH                     0x00
#define  STK6BCX_GPIO_DEB_SEL_RISGING                  0x01
#define  STK6BCX_GPIO_DEB_SEL_FALLING                  0x02
#define  STK6BCX_GPIO_DEB_SEL_DISABLE                  0x03

#define  STK6BCX_GPIO_FREQ_LOST_THD_96US               0x1F
#define  STK6BCX_GPIO_FREQ_LOST_THD_MASK               0x1F

#define  STK6BCX_GPIO_ALS_IGNORE_HIGH_MAKS             0xFF
#define  STK6BCX_GPIO_PS_IGNORE_HIGH_MAKS              0xFF

#define  STK6BCX_GPIO_TIMER_DIABLE                     0x00
#define  STK6BCX_GPIO_TIMER_20MS                       0x68
#define  STK6BCX_GPIO_TIMER_50MS                       0x104

#define  STK6BCX_GPIO_TIMER_HIGH_MASK                  0x3F

#define  STK6BCX_GPIO_FLAG_FREQ_LOST_INT               0x80
#define  STK6BCX_GPIO_FLAG_TIMER_INT                   0x40
#define  STK6BCX_GPIO_FLAG_FREQ_LOST                   0x20
#define  STK6BCX_GPIO_FLAG_TIMER                       0x10

#define  STK6BCX_GPIO_ALS_DUTY_1                       0x00
#define  STK6BCX_GPIO_ALS_DUTY_2                       0x01
#define  STK6BCX_GPIO_ALS_DUTY_4                       0x03
#define  STK6BCX_GPIO_ALS_DUTY_10                      0x09
#define  STK6BCX_GPIO_ALS_DUTY_16                      0x0F

#define  STK6BCX_GPIO_PS_DUTY_1                        0x00
#define  STK6BCX_GPIO_PS_DUTY_2                        0x01
#define  STK6BCX_GPIO_PS_DUTY_4                        0x03
#define  STK6BCX_GPIO_PS_DUTY_10                       0x09
#define  STK6BCX_GPIO_PS_DUTY_16                       0x0F

#define  STK6BCX_FSM_MODE_ALTERNATIVE                  0x00
#define  STK6BCX_FSM_MODE_SEQUENTIAL                   0x01


#define  STK6BCX_PS_PD_SEL                             0x80
#define  STK6BCX_PS_PD_MASK                            0x80

/* Define FIFO FLK reg */
#define  STK6BCX_FIFO_FLK_MODE_OFF                     0x00
#define  STK6BCX_FIFO_FLK_MODE_BYPASS                  0x01
#define  STK6BCX_FIFO_FLK_MODE_FIFO                    0x02
#define  STK6BCX_FIFO_FLK_MODE_STREAM                  0x03
#define  STK6BCX_FIFO_FLK_MODE_MASK                    0x03

#define  STK6BCX_FIFO_FLK_FLUSH                        0x02
#define  STK6BCX_FIFO_FLK_PAUSE                        0x01

#define  STK6BCX_FIFO_FLK_FOVR_EN                      0x04
#define  STK6BCX_FIFO_FLK_FWM_EN                       0x02
#define  STK6BCX_FIFO_FLK_FFULL_EN                     0x01

/* Define IRDR reg */
#define  STK6BCX_IRDR_EN_DIV4                          0x01
#define  STK6BCX_IRDR_EN_IRDR                          0x00
#define  STK6BCX_IRDR_DISABLE_IRDR                     0x01

/* PID */
#define STK6BCX_PID_LIST_NUM                           5

/** sw reset value */
#define STK_STK6BCX_SWRESET                            0x00

/** Off to idle time */
#define STK6BCX_OFF_TO_IDLE_MS                         10  //ms

/** ALS threshold */

//#ifdef STK_PS_TUNE0
    #define STK6BCX_SMUDGE_DIFF                        300
    #define STK6BCX_TRACKING_QUANTI                    4
    #define STK6BCX_QUANTI_RANGE                       10
//#endif

#ifdef STK_PS_CALI
    #define STK6BCX_PS_CALI_TIMES                      2
    #define STK6BCX_PS_CALI_ERROR_TIMES                2
    #define STK6BCX_PS_CALI_MAX_CROSSTALK              3000
    #define STK6BCX_PS_CALI_DIFF                       40
#endif
#define STK6BCX_NUM_RGB                                6
#ifdef STK_RGB_CALI
    #define STK6BCX_RGB_CALI_TARGET_R                  420.0
    #define STK6BCX_RGB_CALI_TARGET_G                  284.0
    #define STK6BCX_RGB_CALI_TARGET_B                  181.0
    #define STK6BCX_RGB_CALI_PERIOD                    120
#endif

#define STK6BCX_PS_BGIR_THRESHOLD                      0x64
#define STK6BCX_BOOT_CALI_ERROR_TIMES                  2
#define STK6BCX_BOOT_CALI_GETDATA_TIMES                2

#define STK6BCX_WATCH_DOG_LIMIT                        10
#define STK6BCX_LOG4(x)                                (uint8_t)(logf(x)/logf(4))
#define STK6BCX_LOG2(x)                                (uint8_t)(logf(x)/logf(2))
#ifdef STK_RGB_ENABLE
    #define CCTR_MATRIX_ROW                            3
    #define CCTR_MATRIX_COL                            4
#endif

#define MAX_GAIN                                       (STK6BCX_ALS_DGAIN_MULTI512 * STK6BCX_ALS_AGAIN_MULTI16)
#define MAX_AG                                         STK6BCX_ALS_AGAIN_MULTI16
#define STK6BCX_GPIO_IGNORE(US)                        ((uint16_t)(US/24))

#define STK6BCX_TC_SLOPE_THD        40
#define STK6BCX_TC_MAX_MIN_DIFF     15
#define STK6BCX_TC_CT_DIFF          20
#define STK6BCX_TC_TRACKING_TIME    105

#define STK6BCX_CT_FIR_LEN 5

#define STK6BCX_MAX_MIN_DIFF                           200
#define STK6BCX_LT_N_CT                                300
#define STK6BCX_HT_N_CT                                500
#define STK6BCX_DEFAULT_CT                             6000
#define STK6BCX_PS_BOOT_THD_RATIO                      2
#define STK6BCX_PS_SMUDGE_RATIO                        3

#define STK6BCX_PRX_THD_NEAR                           1000
#define STK6BCX_PRX_THD_FAR                            900
#define STK6BCX_PRX_DATA_READY_TIME                    10
#define STK6BCX_PRX_TUNE0_TIME                         75

#define  STK6BCX_PS_DUMP_CNT    500

#define PROX_STATE_NEAR                 0
#define PROX_STATE_FAR                  1

typedef struct
{
    uint16_t raw[STK6BCX_CT_FIR_LEN];
    //uint16_t number;
    uint16_t idx;
    uint16_t max;
    uint16_t min;
}stk_ps_data_filter;

typedef struct stk6bcx_gpio_config
{
    uint8_t screen_hz;
    uint32_t target_timer;
    uint8_t ps_duty;
    uint32_t ps_td;
    uint32_t ps_ignore;
} stk6bcx_gpio_config;


enum PROX_STATE{
    PS_UNKNOWN = -1,
    PS_NEAR = 0,
    PS_FAR = 1,
};

struct stk6bcx_ps_data
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
    uint16_t ps_dbg_cnt;
    int ps_distance_last;
    bool ps_need_report;
    bool ps_enable;
    uint16_t tracking_time; // tune0 period time
    
    uint16_t compensation_target;
    uint16_t compensation_cnt;
    stk_ps_data_filter ps_data_tc_filter;
#ifdef STK6BCX_GPIO_PS
    uint8_t display_freq;
    bool gpio_enable;
#endif
};

typedef struct stk6bcx_register_table
{
    uint8_t address;
    uint8_t value;
    uint8_t mask;
} stk6bcx_register_table;

#endif //_PROXIMITY_STK6BCX_SENSOR_DRIVER_H_

