#ifndef _LIGHT_STK6BCX_SENSOR_DRIVER_H_
#define _LIGHT_STK6BCX_SENSOR_DRIVER_H_

//#include "sensor_driver_common.h"

/** sw reset value */

#define STK_ALS_HAGC
#define STK_ALS_IT1_SHORT
//#define STK_GPIO_ALS

#define NO_ERROR 0
#define FAIL (-1)

#define STK_TAG                  "[ALS] "
#define STK_FUN(f)               printk(STK_TAG" %s\n", __FUNCTION__)
//#define STK_ERR(fmt, args...)    printf(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define STK_LOG(fmt, args...)    printk(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
//#define STK_DBG(fmt, args...)    printf(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define STK_ABS(x) ((x<0)? -(x):(x))

//#define STK_ALGO_ENABLE
#ifdef STK_ALGO_ENABLE
    #include "stk_alps_lib.h" //enable algorithm
#endif

// #define STK6B1X_RGB_ENABLE
#ifdef STK_ALGO_ENABLE
    #define MAX_BACKGROUND_NUM 10
    #define TARGET_LUX 1000
#endif

#define STK6BCX_PID 0x12
#define FLOAT_EPS   1e-6
#define STK6BCX_ALS_MIN_LUX_THD_LIMIT   50
#define STK6BCX_ALS_MAX_LUX_CCT_LIMIT   6500
#define STK6BCX_ALS_MIN_LUX_CCT_LIMIT   5001
#define STK6BCX_MAX_GAIN    (STK6BCX_ALS_DGAIN_MULTI512)

#define STK_FIFO_I2C_READ_FRAME             32     //I2C bus get fifo data every cycle
#define STK_FIFO_I2C_READ_FRAME_TARGET      128

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
#define STK6BCX_REG_ALS_THDH1                          0x24
#define STK6BCX_REG_ALS_THDH2                          0x25
#define STK6BCX_REG_ALS_THDL1                          0x26
#define STK6BCX_REG_ALS_THDL2                          0x27
#define STK6BCX_REG_INT_CTRL1                          0x2C
#define STK6BCX_REG_INT_CTRL2                          0x2D
#define STK6BCX_REG_ALS_FLAG                           0x30
#define STK6BCX_REG_ALS_FIFO_FLAG                      0x37
#define STK6BCX_REG_ALS_FIFO_CNT1                      0x38
#define STK6BCX_REG_ALS_FIFO_CNT2                      0x39
#define STK6BCX_REG_ALS_FIFO_OUT                       0x3C
#define STK6BCX_REG_FLK_FIFO_OUT                       0x3D
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

#define  STK6BCX_WAIT_0                                0x0
#define  STK6BCX_WAIT_1                              0x2F
#define  STK6BCX_WAIT_20                               0x3B8  //20ms
#define  STK6BCX_WAIT_50                               0x94D
#define  STK6BCX_WAIT_100                              0x129A
#define  STK6BCX_WAIT2_H_MAKS                          0xFF

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
#define  STK6BCX_ALS_INT_EN                            0x02

/* Define ALS FLAG reg */
#define  STK6BCX_FLG_ALS_DR_MASK                       0x80
#define  STK6BCX_FLG_ALS_INT_MASK                      0x40
#define  STK6BCX_FLG_ALS_SAT_MAKS                      0x20

/* Define ALS FIFO FLAG reg*/
#define  STK6BCX_FLG_FIFO_OVR_MAKS                     0x40
#define  STK6BCX_FLG_FIFO_WM_MAKS                      0x20
#define  STK6BCX_FLG_FIFO_FULL_MAKS                    0x10

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

#define  STK6BCX_FSM_MODE_ALTERNATIVE                  0x00
#define  STK6BCX_FSM_MODE_SEQUENTIAL                   0x01

#define  STK6BCX_ALS_PD_SEL0                           0x01
#define  STK6BCX_ALS_PD_SEL1                           0x02
#define  STK6BCX_ALS_PD_SEL2                           0x04
#define  STK6BCX_ALS_PD_SEL3                           0x08
#define  STK6BCX_ALS_PD_SEL4                           0x10
#define  STK6BCX_ALS_PD_SEL5                           0x20
#define  STK6BCX_ALS_PD_MASK                           0X3F


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

/* PID */
#define STK6BCX_PID_LIST_NUM                           5

/** sw reset value */
#define STK_STK6BCX_SWRESET                            0x00

/** Off to idle time */
#define STK6BCX_OFF_TO_IDLE_MS                         10  //ms

/** ALS threshold */
#define STK6BCX_ALS_THD_ADJ                            0.05
#define STK6BCX_NUM_AXES                               3
#define STK6BCX_ALS_DATA_READY_TIME                    60
#define STK6BCX_ALS_THRESHOLD                          30

#ifdef STK_PS_TUNE0
    #define STK6BCX_SMUDGE_DIFF                        300
    #define STK6BCX_TRACKING_QUANTI                    4
    #define STK6BCX_QUANTI_RANGE                       10
#endif

#ifdef STK_ALS_CALI
    #define STK6BCX_ALS_CALI_DATA_READY_US             55000000
    #define STK6BCX_ALS0_CALI_TARGET                  500.0
    #define STK6BCX_ALS1_CALI_TARGET                  400.0
    #define STK6BCX_ALS2_CALI_TARGET                  300.0
    #define STK6BCX_ALS3_CALI_TARGET                  200.0
    #define STK6BCX_ALS4_CALI_TARGET                  100.0
    #define STK6BCX_ALS5_CALI_TARGET                  150.0

    #define STK6BCX_ALS_CALI_DATA_CNT_TARGET           5
    #define STK6BCX_RGB_CALI_DATA_CNT_TARGET           5
#endif

#define STK6BCX_NUM_RGB                                6
#ifdef STK_RGB_CALI
    #define STK6BCX_RGB_CALI_TARGET_R                  420.0
    #define STK6BCX_RGB_CALI_TARGET_G                  284.0
    #define STK6BCX_RGB_CALI_TARGET_B                  181.0
    #define STK6BCX_RGB_CALI_PERIOD                    120
#endif

#ifdef STK_ALS_SW_AGC
    #define STK6BCX_SW_AGC_ALS_IT_HTD                  64000
    #define STK6BCX_SW_AGC_ALS_IT_LTD                  3000
#endif

#define STK6BCX_ALS_DUMP_CNT    300

#define STK6BCX_WATCH_DOG_LIMIT                        10
#define STK6BCX_LOG4(x)                                (uint8_t)(logf(x)/logf(4))
#define STK6BCX_LOG2(x)                                (uint8_t)(logf(x)/logf(2))
#ifdef STK_RGB_ENABLE
    #define CCTR_MATRIX_ROW                            3
    #define CCTR_MATRIX_COL                            4
#endif

#define STK_FIFO_I2C_READ_BYTE              1024

#define MAX_GAIN                                       (STK6BCX_ALS_DGAIN_MULTI512 * STK6BCX_ALS_AGAIN_MULTI16)

typedef enum
{
    STK6BCX_CALI_IDLE,
    STK6BCX_CALI_RUNNING,
    STK6BCX_CALI_FAILED,
    STK6BCX_CALI_DONE
} stk6bcx_calibration_status;

typedef enum
{
    STK6BCX_ALS_DGAIN_MULTI1   = 1,
    STK6BCX_ALS_DGAIN_MULTI4   = 4,
    STK6BCX_ALS_DGAIN_MULTI16  = 16,
    STK6BCX_ALS_DGAIN_MULTI64  = 64,
    STK6BCX_ALS_DGAIN_MULTI128 = 128,
    STK6BCX_ALS_DGAIN_MULTI256 = 256,
    STK6BCX_ALS_DGAIN_MULTI512 = 512,
} stk6bcx_als_dgain_multi;

typedef enum
{
    STK6BCX_ALS_AGAIN_MULTI1   = 1,
    STK6BCX_ALS_AGAIN_MULTI2   = 2,
    STK6BCX_ALS_AGAIN_MULTI4   = 4,
    STK6BCX_ALS_AGAIN_MULTI8   = 8,
    STK6BCX_ALS_AGAIN_MULTI16  = 16,
} stk6bcx_als_again_multi;

typedef enum
{
    STK6BCX_RGB_CALI_SW,
    STK6BCX_RGB_CALI_HW,
    STK6BCX_RGB_CALI_FACTORY,
    STK6BCX_RGB_CALI_COM,
    STK6BCX_RGB_CALI_NONE
} stk6bcx_rgb_cali_type;

typedef enum
{
    STK6BCX_ALS_CH0                           = 0,
    STK6BCX_ALS_CH1                           = 1,
    STK6BCX_ALS_CH2                           = 2,
    STK6BCX_ALS_CH3                           = 3,
    STK6BCX_ALS_CH4                           = 4,
    STK6BCX_ALS_CH5                           = 5,
    STK6BCX_ALS_CNT                           = 6,
}stk6bcx_rgb_ch_repos1;


typedef enum
{
    STK6BCX_ALS_R                           = 0,
    STK6BCX_ALS_G                           = 1,
    STK6BCX_ALS_B                           = 2,
    STK6BCX_ALS_W                           = 3,
    STK6BCX_ALS_RF1                         = 4,
    STK6BCX_ALS_RF2                          = 5,
}stk6bcx_rgb_ch_repos;

typedef struct light_pare {
    uint8_t group_sel;
    float group_rule;
    float group_rule_mat;
    float group_rule_mat_2;
    float param_gc_ratio;
    float param_c_scale;
    float param_r_scale;
    float param_g_scale;
    float param_b_scale;
    float param_g2_scale;
    float param_r2_scale;
    float param_b2_scale;
    float param_limit;
    float param_lower_thd;
    uint8_t  param_lower_sel;
} light_pare;

struct NCSDataColorSensor
{
    float cct;
    float r;
    float g;
    float b;
    float c;
    float w;
    float ir_ratio; //(2.7W-C)/(2.7W)
    float x;
    float y;
    float lux;
};

//#ifdef STK6BCX_RGB_ENABLE

enum stk6bcx_fac_cali
{
    R_CALI = 0,
    G_CALI,
    B_CALI,
    C_CALI,
    W_CALI,
    F_CALI,
    STK6B1X_CALI_SIZE
};

typedef struct stk6bcx_register_table
{
    uint8_t address;
    uint8_t value;
    uint8_t mask;
} stk6bcx_register_table;

#ifdef STK_ALGO_ENABLE
typedef struct
{
    uint16_t fac_cali_F_data[MAX_BACKGROUND_NUM];
    uint16_t fac_cali_G_data[MAX_BACKGROUND_NUM];
    uint16_t fac_cali_C_data[MAX_BACKGROUND_NUM];
    uint16_t fac_cali_other_data[MAX_BACKGROUND_NUM];
} stk6bcx_als_fac_cali_data;

struct stk6bcx_als_fac_cali_data_type
{
    uint16_t fac_cali_F;
    uint16_t fac_cali_G;
    uint16_t fac_cali_C;
    uint16_t fac_cali_other;
    uint8_t fgc_count;
};

typedef struct PixelData_m
{
    uint8_t PixelR;
    uint8_t PixelG;
    uint8_t PixelB;
    //    uint8_t Fresh_rate;
    uint16_t Brightness;
} PixelData_m;

struct display_info
{
    uint32_t brightness;
    uint32_t pixel_rgb;
};

struct under_screen_als_info
{
    uint8_t send_flag;
    union
    {
        uint32_t underscreen_als[MAX_BACKGROUND_NUM];
        struct display_info display_info_t;
    };
};
#endif

#ifdef STK_ALS_MID_FIR
#define STK_ALS_MID_FIR_LEN              5
#define MAX_ALS_FIR_LEN                  32
typedef struct
{
    uint32_t raw[STK_ALS_MID_FIR_LEN];
    uint32_t number;
    uint32_t index;
} stk6bcx_data_filter;
#endif

struct stk6bcx_data
{
    uint16_t als_count;
    uint8_t als_enable;
    float r_scale;
    float g_scale;
    float b_scale;
    float w_scale;
    //float lux;
    //float cct;
    uint32_t lux;
    uint32_t cct;
    uint32_t als_dbg_cnt;
    uint8_t data_type;
    bool fifo_enable;
    bool als_is_ready;
    bool fifo_is_ready;
    uint8_t frame_byte;
    uint8_t fifo_channel_byte;
    uint8_t xFlag_pos;
    uint8_t xflag;
    uint16_t last_frame_count;
    uint8_t read_frame;
    uint8_t target_frame_count;
    uint16_t read_max_byte;

    uint8_t raw_data[STK_FIFO_I2C_READ_BYTE];
    uint32_t fifo_data0[STK_FIFO_I2C_READ_FRAME_TARGET];
    uint32_t fifo_data1[STK_FIFO_I2C_READ_FRAME_TARGET];
    uint32_t fifo_data2[STK_FIFO_I2C_READ_FRAME_TARGET];
    uint32_t fifo_data3[STK_FIFO_I2C_READ_FRAME_TARGET];
    uint32_t fifo_data4[STK_FIFO_I2C_READ_FRAME_TARGET];

    uint32_t fifo_data5[STK_FIFO_I2C_READ_FRAME_TARGET];

    stk6bcx_als_dgain_multi     als_cur_dgain[STK6BCX_ALS_CNT];
    stk6bcx_als_again_multi     als_cur_again[STK6BCX_ALS_CNT];
#ifdef STK_ALS_MID_FIR
    stk6bcx_data_filter         als_data_filter;
#endif

    uint32_t als_cur_ratio[STK6BCX_ALS_CNT];
    uint32_t als_last_raw_data[8];
//    int32_t ch_scale[STK6BCX_ALS_CNT];
    //    struct NCSDataColorSensor als_sample;
    struct NCSDataColorSensor rgb_sample;
#ifdef STK_ALGO_ENABLE
    uint8_t calibrated;
    PixelData_m pixeldata;
    ChannelData ChannelData[MAX_BACKGROUND_NUM];
#endif
    uint32_t last_raw_data[STK6BCX_ALS_CNT];
//#ifdef STK_GPIO_ALS
    uint8_t display_freq;
    bool gpio_enable;
//#endif
};
//struct sensor_driver* light_stk6bcx_sensor_init() ;

#endif //_LIGHT_STK6B1X_SENSOR_DRIVER_H_

