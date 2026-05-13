#ifndef _LIGHT_STK6B1X_SENSOR_DRIVER_H_
#define _LIGHT_STK6B1X_SENSOR_DRIVER_H_

//#include "sensor_driver_common.h"

/** sw reset value */
#define STK_STK6B1X_SWRESET             0x01

#define STK_ALS_HAGC
#define STK_ALS_IT1_SHORT
#define STK_GPIO_ALS
#define STK6B1X_PID 0xA1

#define NO_ERROR 0
#define FAIL (-1)

#define STK_TAG                  "[ALS] "
#define STK_FUN(f)               printk(STK_TAG" %s\n", __FUNCTION__)
//#define STK_ERR(fmt, args...)    printf(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define STK_LOG(fmt, args...)    printk(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
//#define STK_DBG(fmt, args...)    printf(STK_TAG" %s %4d: "fmt"\n", __FUNCTION__, __LINE__, ##args)


//#define STK_ALGO_ENABLE
#if STK_ALGO_ENABLE
    #include "stk_alps_lib.h" //enable algorithm
#endif

// #define STK6B1X_RGB_ENABLE

/*ALSPS REGS*/
#define STK6B1X_REG_ENABLE                  0x00
#define STK6B1X_REG_FSM_CTRL                0x01
#define STK6B1X_REG_SWRST                   0x02
#define STK6B1X_REG_ALS_DGAIN               0x10
#define STK6B1X_REG_ALS_DGAIN1              0x11
#define STK6B1X_REG_ALS_DGAIN2              0x12
#define STK6B1X_REG_ALS_AGAIN               0x13
#define STK6B1X_REG_ALS_AGAIN1              0x14
#define STK6B1X_REG_ALS_AGAIN2              0x15
#define STK6B1X_REG_ALS_IT_SET0             0x16
#define STK6B1X_REG_ALS_IT_SET1             0x18
#define STK6B1X_REG_ALS_WAIT1               0x1A
#define STK6B1X_REG_ALS_WAIT2               0x1B
#define STK6B1X_REG_ALS_PRST                0x1E
#define STK6B1X_REG_ALS_THDH1               0x24
#define STK6B1X_REG_ALS_THDH2               0x25
#define STK6B1X_REG_ALS_THDL1               0x26
#define STK6B1X_REG_ALS_THDL2               0x27
#define STK6B1X_REG_INTCTRL1                0x2C
#define STK6B1X_REG_INTCTRL2                0x2D
#define STK6B1X_REG_ALS_FLAG                0x30
#define STK6B1X_REG_ALS_FIFO_FLAG           0x39
#define STK6B1X_REG_ALS_FIFO_CNT1           0x3A
#define STK6B1X_REG_ALS_FIFO_CNT2           0x3B
#define STK6B1X_REG_ALS_FIFO_OUT            0x3C
#define STK6B1X_REG_PID                     0x3E
#define STK6B1X_REG_ALS_AGC_SET0            0x5B
#define STK6B1X_REG_ALS_AGC_SET1            0x5C
#define STK6B1X_REG_ALS_AGC_SET2            0x5D
#define STK6B1X_REG_ALS_AGC_SET3            0x5E
#define STK6B1X_REG_ALS_AGC_SET4            0x5F
#define STK6B1X_REG_ALS_AGC_SET5            0x60
#define STK6B1X_REG_ALS_AGC_SET6            0x62
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

/*********************************************
* Register control                           *
*********************************************/
/* Define state reg */
#define STK6B1X_STATE_EN_ALS_WAIT_SHIFT     3
#define STK6B1X_STATE_EN_ALS_SHIFT          1
#define STK6B1X_STATE_EN_ALS_WAIT_MASK      0x08
#define STK6B1X_STATE_EN_ALS_MASK           0x02

#define STK6B1X_FLG_ALS_DR_MASK             0x80
#define STK6B1X_FLG_ALS_INT_MASK            0x40
#define STK6B1X_FLG_ALS_SAT_MASK            0x20

/* Define FSM Ctrl reg*/
#define STK6B1X_FSM_FLG_RST_SHIFT           7
#define STK6B1X_FSM_ALS_PAUSE_SHIFT         5
#define STK6B1X_FSM_PS_PAUSE_SHIFT          4
#define STK6B1X_FSM_ALS_FUNC_RESTART_SHIFT  2
#define STK6B1X_FSM_PS_FUNC_RESTART_SHIFT   1
#define STK6B1X_FSM_FLG_RST_MASK            0x80
#define STK6B1X_FSM_ALS_PAUSE_MASK          0x20
#define STK6B1X_FSM_PS_PAUSE_MASK           0x10
#define STK6B1X_FSM_PS_FUNC_RESTART_MASK    0x04
#define STK6B1X_FSM_ALS_FUNC_RESTART_MASK   0x02
/*********************************************
* Parameters control                         *
*********************************************/
#define STK6B1X_ALS_IT_SEL_SHIFT            6
#define STK6B1X_ALS_IT_SEL_MASK             0x40
#define STK6B1X_ALS_IT_SEL_IT1              (0x00 << STK6B1X_ALS_IT_SEL_SHIFT)
#define STK6B1X_ALS_IT_SEL_IT1_SHORT        (0x01 << STK6B1X_ALS_IT_SEL_SHIFT)

#define STK6B1X_ALS_IT_MASK                 0x0F
#define STK6B1X_ALS_IT_3_125                0x0
#define STK6B1X_ALS_IT_6_25                 0x1
#define STK6B1X_ALS_IT_12_5                 0x2
#define STK6B1X_ALS_IT_25                   0x3
#define STK6B1X_ALS_IT_50                   0x4
#define STK6B1X_ALS_IT_100                  0x5
#define STK6B1X_ALS_IT_200                  0x6
#define STK6B1X_ALS_IT_400                  0x7

#define STK6B1X_ALS_IT_SHORT_MASK           0x7F
#define STK6B1X_ALS_IT_SHORT_96             0x03
#define STK6B1X_ALS_IT_SHORT_288            0x0B
#define STK6B1X_ALS_IT_SHORT_384            0x0F
#define STK6B1X_ALS_IT_SHORT_480            0x13
#define STK6B1X_ALS_IT_SHORT_576            0x17
#define STK6B1X_ALS_IT_SHORT_672            0x1B
#define STK6B1X_ALS_IT_SHORT_768            0x1F
#define STK6B1X_ALS_IT_SHORT_864            0x23
#define STK6B1X_ALS_IT_SHORT_960            0x27
#define STK6B1X_ALS_IT_SHORT_1056           0x2B

/* Define ALS AGC SET4/5 reg */
#define STK6B1X_ALS0_AGC_SEL_SHIFT          6
#define STK6B1X_ALS1_AGC_SEL_SHIFT          4
#define STK6B1X_ALS2_AGC_SEL_SHIFT          2
#define STK6B1X_ALS3_AGC_SEL_SHIFT          0
#define STK6B1X_ALS4_AGC_SEL_SHIFT          6

#define STK6B1X_ALS0_NO_AGC                 (0x00 << STK6B1X_ALS0_AGC_SEL_SHIFT)
#define STK6B1X_ALS0_AGC1                   (0x01 << STK6B1X_ALS0_AGC_SEL_SHIFT)
#define STK6B1X_ALS0_AGC2                   (0x02 << STK6B1X_ALS0_AGC_SEL_SHIFT)

#define STK6B1X_ALS1_NO_AGC                 (0x00 << STK6B1X_ALS1_AGC_SEL_SHIFT)
#define STK6B1X_ALS1_AGC1                   (0x01 << STK6B1X_ALS1_AGC_SEL_SHIFT)
#define STK6B1X_ALS1_AGC2                   (0x02 << STK6B1X_ALS1_AGC_SEL_SHIFT)

#define STK6B1X_ALS2_NO_AGC                 (0x00 << STK6B1X_ALS2_AGC_SEL_SHIFT)
#define STK6B1X_ALS2_AGC1                   (0x01 << STK6B1X_ALS2_AGC_SEL_SHIFT)
#define STK6B1X_ALS2_AGC2                   (0x02 << STK6B1X_ALS2_AGC_SEL_SHIFT)

#define STK6B1X_ALS3_NO_AGC                 (0x00 << STK6B1X_ALS3_AGC_SEL_SHIFT)
#define STK6B1X_ALS3_AGC1                   (0x01 << STK6B1X_ALS3_AGC_SEL_SHIFT)
#define STK6B1X_ALS3_AGC2                   (0x02 << STK6B1X_ALS3_AGC_SEL_SHIFT)

#define STK6B1X_ALS4_NO_AGC                 (0x00 << STK6B1X_ALS4_AGC_SEL_SHIFT)
#define STK6B1X_ALS4_AGC1                   (0x01 << STK6B1X_ALS4_AGC_SEL_SHIFT)
#define STK6B1X_ALS4_AGC2                   (0x02 << STK6B1X_ALS4_AGC_SEL_SHIFT)

/* Define ALS DG reg */
#define STK6B1X_ALS0_DGAIN_SHIFT            4
#define STK6B1X_ALS1_DGAIN_SHIFT            0
#define STK6B1X_ALS2_DGAIN_SHIFT            4
#define STK6B1X_ALS3_DGAIN_SHIFT            0
#define STK6B1X_ALS4_DGAIN_SHIFT            4

#define STK6B1X_ALS0_DGAIN_MASK             0xF0
#define STK6B1X_ALS1_DGAIN_MASK             0x0F
#define STK6B1X_ALS2_DGAIN_MASK             0xF0
#define STK6B1X_ALS3_DGAIN_MASK             0x0F
#define STK6B1X_ALS4_DGAIN_MASK             0xF0

#define STK6B1X_ALS_DGAIN1                  0x00
#define STK6B1X_ALS_DGAIN2                  0x01
#define STK6B1X_ALS_DGAIN4                  0x02
#define STK6B1X_ALS_DGAIN8                  0x03
#define STK6B1X_ALS_DGAIN16                 0x04
#define STK6B1X_ALS_DGAIN32                 0x05
#define STK6B1X_ALS_DGAIN64                 0x06
#define STK6B1X_ALS_DGAIN128                0x07
#define STK6B1X_ALS_DGAIN256                0x08
#define STK6B1X_ALS_DGAIN512                0x09
#define STK6B1X_ALS_DGAIN1024               0x0A
#define STK6B1X_ALS_DGAIN2048               0x0B

/* Define ALS AG reg*/
#define STK6B1X_ALS0_AGAIN_SHIFT            4
#define STK6B1X_ALS1_AGAIN_SHIFT            0
#define STK6B1X_ALS2_AGAIN_SHIFT            4
#define STK6B1X_ALS3_AGAIN_SHIFT            0
#define STK6B1X_ALS4_AGAIN_SHIFT            4

#define STK6B1X_ALS0_AGAIN_MASK             0x70
#define STK6B1X_ALS1_AGAIN_MASK             0x07
#define STK6B1X_ALS2_AGAIN_MASK             0x70
#define STK6B1X_ALS3_AGAIN_MASK             0x07
#define STK6B1X_ALS4_AGAIN_MASK             0x70

#define STK6B1X_ALS_AGAIN2_0                0x00
#define STK6B1X_ALS_AGAIN1_0                0x01
#define STK6B1X_ALS_AGAIN0_5                0x02
#define STK6B1X_ALS_AGAIN0_25               0x03
#define STK6B1X_ALS_AGAIN0_125              0x04

/* Define FIFO SET0 reg */
#define STK6B1X_FIFO_SEL_SHIFT                              3
#define STK6B1X_FIFO_SEL_MASK                               0xF8
#define STK6B1X_FIFO_SEL_ALS01234                           0x00
#define STK6B1X_FIFO_SEL_SALS0_SALS1_SALS2_SALS3_SALS4      0x06
#define STK6B1X_FIFO_SEL_SALSR0_SALSR1_SALSR2_SALSR3_SALSR4 0x0C

#define STK6B1X_FIFO_MODE_MASK                              0x03
#define STK6B1X_FIFO_MODE_OFF                               0x00
#define STK6B1X_FIFO_MODE_BYPASS                            0x01
#define STK6B1X_FIFO_MODE_NORMAL                            0x02
#define STK6B1X_FIFO_MODE_STREAM                            0x03

/* Define FIFO SET4 reg */
#define STK6B1X_FIFO_EN_SRAM_PEW_MASK                       0x08
#define STK6B1X_FIFO_EN_FOVR_MASK                           0x04
#define STK6B1X_FIFO_EN_FWM_MASK                            0x02
#define STK6B1X_FIFO_EN_FFULL_MASK                          0x01

#define STK6B1X_H_BYTE(x)                  ((x & 0xFF00) >> 8)
#define STK6B1X_L_BYTE(x)                  (x  & 0x00FF)

#ifdef STK_GPIO_ALS
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

#define STK_FIFO_I2C_READ_FRAME             32     //I2C bus get fifo data every cycle
#ifdef STK_ALS_IT1_SHORT
    #define STK_FIFO_I2C_READ_FRAME_TARGET  128
#else
    #define STK_FIFO_I2C_READ_FRAME_TARGET  1
#endif
#define STK_FIFO_I2C_READ_BYTE              1024

#define STK6B1X_ALS_DGAIN_MASK              0x0F
#define STK6B1X_ALS_AGAIN_MASK              0x70
#define STK6B1X_ALS_AGAIN_SHIFT             4

#define STK6B1X_MAX_GAIN                    STK6B1X_ALS_DG_MULTI2048 * STK6B1X_ALS_AG_MULTI16

#define STK6B1X_ALS_GET_STA_DG(state)       (state & STK6B1X_ALS_DGAIN_MASK)
#define STK6B1X_ALS_GET_STA_AG(state)       (state & STK6B1X_ALS_AGAIN_MASK) >> STK6B1X_ALS_AGAIN_SHIFT
#define STK6B1X_GET_GAIN_RATIO(DG,AG)       (STK6B1X_MAX_GAIN / (DG * AG))

#define STK6B1X_CH_CNT                  5
#define CCTR_COEF_ROW                   3
#define CCTR_COEF_COL                   4
#define RAW_NUM                         4

#define STK_STK6B1X_SWRESET             0x01

#if STK_ALGO_ENABLE
    #define MAX_BACKGROUND_NUM 10
    #define TARGET_LUX 1000
#endif

typedef enum
{
    STK6B1X_INIT       = 0x00,
    STK6B1X_PS         = 0x01,
    STK6B1X_ALS        = 0x02,
    STK6B1X_RGB        = 0x04,
} stk6b1x_sensor_type;

typedef enum
{
    STK6B1X_ALS_DG_MULTI1       = 1 << 0,
    STK6B1X_ALS_DG_MULTI2       = 1 << 1,
    STK6B1X_ALS_DG_MULTI4       = 1 << 2,
    STK6B1X_ALS_DG_MULTI8       = 1 << 3,
    STK6B1X_ALS_DG_MULTI16      = 1 << 4,
    STK6B1X_ALS_DG_MULTI32      = 1 << 5,
    STK6B1X_ALS_DG_MULTI64      = 1 << 6,
    STK6B1X_ALS_DG_MULTI128     = 1 << 7,
    STK6B1X_ALS_DG_MULTI256     = 1 << 8,
    STK6B1X_ALS_DG_MULTI512     = 1 << 9,
    STK6B1X_ALS_DG_MULTI1024    = 1 << 10,
    STK6B1X_ALS_DG_MULTI2048    = 1 << 11,
} stk6b1x_als_dgain_multi;
typedef enum
{
    STK6B1X_ALS_AG_MULTI1       = 1 << 0,
    STK6B1X_ALS_AG_MULTI2       = 1 << 1,
    STK6B1X_ALS_AG_MULTI4       = 1 << 2,
    STK6B1X_ALS_AG_MULTI8       = 1 << 3,
    STK6B1X_ALS_AG_MULTI16      = 1 << 4,
} stk6b1x_als_again_multi;

typedef enum
{
    ALS0_CALI = 0,
    ALS1_CALI,
} stk6b1x_fac_cali;

typedef enum
{
    STK6B1X_CALI_IDLE,
    STK6B1X_CALI_RUNNING,
    STK6B1X_CALI_FAILED,
    STK6B1X_CALI_DONE
} stk6b1x_calibration_status;

typedef enum
{
    STK6B1X_AGC_OFF    = 0x00,
    STK6B1X_AGC1       = 0x01,
    STK6B1X_AGC2       = 0x02,
} stk6b1x_als_agc_type;

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

#ifdef STK6B1X_RGB_ENABLE
typedef struct stk6b1x_cluster_mean
{
    float mean_r;
    float mean_g;
    float mean_b;
    float mean_c;
} stk6b1x_cluster_mean;

typedef struct stk6b1x_cct_cluster
{
    float cct_coef[12];
} stk6b1x_cct_cluster;
#endif

enum stk6b1x_fac_cali
{
    R_CALI = 0,
    G_CALI,
    B_CALI,
    C_CALI,
    W_CALI,
    F_CALI,
    STK6B1X_CALI_SIZE
};

typedef struct stk6b1x_register_table
{
    uint8_t address;
    uint8_t value;
    uint8_t mask;
} stk6b1x_register_table;

#if STK_ALGO_ENABLE
typedef struct
{
    uint16_t fac_cali_F_data[MAX_BACKGROUND_NUM];
    uint16_t fac_cali_G_data[MAX_BACKGROUND_NUM];
    uint16_t fac_cali_C_data[MAX_BACKGROUND_NUM];
    uint16_t fac_cali_other_data[MAX_BACKGROUND_NUM];
} stk6b1x_als_fac_cali_data;

struct stk6b1x_als_fac_cali_data_type
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

struct stk_data
{
    uint16_t als_count;
    uint8_t als_enable;
    float als_scale;
    uint8_t data_type;
    bool fifo_enable;
    bool als_is_ready;
    bool fifo_is_ready;
    uint8_t frame_byte;
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

    uint32_t als_last_raw_data[8];
    int32_t ch_scale[STK6B1X_CALI_SIZE];
    //    struct NCSDataColorSensor als_sample;
    struct NCSDataColorSensor rgb_sample;
#if STK_ALGO_ENABLE
    uint8_t calibrated;
    PixelData_m pixeldata;
    ChannelData ChannelData[MAX_BACKGROUND_NUM];
#endif
    uint32_t als_raw_data_u32[5];
#ifdef STK_GPIO_ALS
    uint8_t display_freq;
    bool gpio_enable;
#endif
};

//struct sensor_driver* light_stk6b1x_sensor_init() ;

#endif //_LIGHT_STK6B1X_SENSOR_DRIVER_H_

