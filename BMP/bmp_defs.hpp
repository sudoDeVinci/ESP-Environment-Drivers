#pragma once

#include <cstdint>

/**
 * Definitions taken from the BMP3xx datasheet and the bmp3_defs.h file from
 * The adafruit BMP3xx library @ https://github.com/adafruit/Adafruit_BMP3XX/blob/master/bmp3_defs.h
 */

 /**
 * BMP3_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BMP3_INTF_RET_TYPE
#define BMP3_INTF_RET_TYPE                      int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BMP3_INTF_RET_SUCCESS
#define BMP3_INTF_RET_SUCCESS                   int8_t(0)
#endif

/**Primary i2c address */
#define BMP3_ADDR_PRIM                          uint8_t(0x76)
/**Secondary i2c address */
#define BMP3_ADDR_SEC                           uint8_t(0x77)

/**\name BMP3 chip identifier */
#define BMP3_CHIP_ID                            uint8_t(0x50)
#define BMP390_CHIP_ID                          uint8_t(0x60)

/**\name BMP3 pressure settling time (micro secs)*/
#define BMP3_SETTLE_TIME_PRESS                  uint16_t(392)
/**\name BMP3 temperature settling time (micro secs) */
#define BMP3_SETTLE_TIME_TEMP                   uint16_t(313)

/**\name BMP3 adc conversion time (micro secs) */
#define BMP3_ADC_CONV_TIME                      uint16_t(2000)
#define BMP3_ENABLE                             uint8_t(0x01)
#define BMP3_DISABLE                            uint8_t(0x00)

/**\name I2c watch dog timer period selection */
#define BMP3_I2C_WDT_SHORT_1_25_MS              uint8_t(0x00)
#define BMP3_I2C_WDT_LONG_40_MS                 uint8_t(0x01)

/**\name Soft reset command */
#define BMP3_SOFT_RESET                         uint8_t(0xB6)

/**\name FIFO flush command */
#define BMP3_FIFO_FLUSH                         uint8_t(0xB0)

/**\name API success code */
#define BMP3_OK                                 int8_t(0)

/**\name API warning codes */
#define BMP3_W_SENSOR_NOT_ENABLED               uint8_t(1)
#define BMP3_W_INVALID_FIFO_REQ_FRAME_CNT       uint8_t(2)


/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.*/
#define BMP3_PRESS                              uint8_t(1)
#define BMP3_TEMP                               uint8_t(1 << 1)
#define BMP3_ALL                                uint8_t(0x03)

/*! Power control settings */
#define BMP3_POWER_CNTL                         uint16_t(0x0006)

/*! Odr and filter settings */
#define BMP3_ODR_FILTER                         uint16_t(0x00F0)

/*! Interrupt control settings */
#define BMP3_INT_CTRL                           uint16_t(0x0708)

/*! Advance settings */
#define BMP3_ADV_SETT                           uint16_t(0x1800)

/*! Mask for fifo_mode, fifo_stop_on_full, fifo_time_en, fifo_press_en and
 * fifo_temp_en settings */
#define BMP3_FIFO_CONFIG_1                      uint16_t(0x003E)

/*! Mask for fifo_sub_sampling and data_select settings */
#define BMP3_FIFO_CONFIG_2                      uint16_t(0x00C0)

/*! Mask for fwtm_en and ffull_en settings */
#define BMP3_FIFO_INT_CTRL                      uint16_t(0x0300)

/**\name    UTILITY MACROS  */
#define BMP3_SET_LOW_BYTE                       uint16_t(0x00FF)
#define BMP3_SET_HIGH_BYTE                      uint16_t(0xFF00)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP3_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BMP3_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMP3_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                 (bitname##_POS))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

#define BMP3_GET_LSB(var)                       (uint8_t)(var & BMP3_SET_LOW_BYTE)
#define BMP3_GET_MSB(var)                       (uint8_t)((var & BMP3_SET_HIGH_BYTE) >> 8)


namespace bmp {

    enum class BMP_REGS : uint8_t {
        CHIP_ID = 0x00,
        ERR = 0x02,
        SENS_STATUS = 0x03,
        DATA = 0x04,
        EVENT = 0x10,
        INT_STATUS = 0x11,
        FIFO_LENGTH = 0x12,
        FIFO_DATA = 0x14,
        FIFO_WM = 0x15,
        FIFO_CONFIG_1 = 0x17,
        FIFO_CONFIG_2 = 0x18,
        INT_CTRL = 0x19,
        IF_CONF = 0x1A,
        POWER_CTRL = 0x1B,
        OSR = 0x1C,
        ODR = 0x1D,
        CONFIG = 0x1F,
        CALIB_DATA = 0x31,
        CMD = 0x7E
    };

    enum class BMP3_ERR : uint8_t {
        FATAL = 0x01,
        CMD = 0x02,
        CONF = 0x04,
    };

    enum class BMP3_DRDY : uint8_t {
        CMD = 0x10,
        PRESS = 0x20,
        TEMP = 0x40
    };

    enum class BMP3_MODE : uint8_t {
        SLEEP = 0x00,
        FORCED = 0x01,
        NORMAL = 0x03
    };

    enum class BMP3_INT_PIN : uint8_t {
        OPEN_DRAIN_MODE = 0x01,
        PUSH_PULL_MODE = 0x00,
        ACTIVE_HIGH = 0x01,
        ACTIVE_LOW = 0x00,
        LATCH_MODE = 0x01,
        NON_LATCH_MODE = 0x00
    };

    enum class BMP3_FIFO_SUBSAMPLING : uint8_t {
        NO_SUBSAMPLING = 0x00,
        SUBSAMPLING_2X = 0x01,
        SUBSAMPLING_4X = 0x02,
        SUBSAMPLING_8X = 0x03,
        SUBSAMPLING_16X = 0x04,
        SUBSAMPLING_32X = 0x05,
        SUBSAMPLING_64X = 0x06,
        SUBSAMPLING_128X = 0x07
    };

    enum class BMP3_OVERSAMPLING : uint8_t {
        NO_OVERSAMPLING = 0x00,
        OVERSAMPLING_2X = 0x01,
        OVERSAMPLING_4X = 0x02,
        OVERSAMPLING_8X = 0x03,
        OVERSAMPLING_16X = 0x04,
        OVERSAMPLING_32X = 0x05
    };

    enum class BMP3_IIR_FILTER : uint8_t {
        DISABLE = 0x00,
        COEFF_1 = 0x01,
        COEFF_3 = 0x02,
        COEFF_7 = 0x03,
        COEFF_15 = 0x04,
        COEFF_31 = 0x05,
        COEFF_63 = 0x06,
        COEFF_127 = 0x07
    };

    enum class BMP3_ODR : uint8_t {
        ODR_200_HZ = 0x00,
        ODR_100_HZ = 0x01,
        ODR_50_HZ = 0x02,
        ODR_25_HZ = 0x03,
        ODR_12_5_HZ = 0x04,
        ODR_6_25_HZ = 0x05,
        ODR_3_1_HZ = 0x06,
        ODR_1_5_HZ = 0x07,
        ODR_0_78_HZ = 0x08,
        ODR_0_39_HZ = 0x09,
        ODR_0_2_HZ = 0x0A,
        ODR_0_1_HZ = 0x0B,
        ODR_0_05_HZ = 0x0C,
        ODR_0_02_HZ = 0x0D,
        ODR_0_01_HZ = 0x0E,
        ODR_0_006_HZ = 0x0F,
        ODR_0_003_HZ = 0x10,
        ODR_0_001_HZ = 0x11
    };

    enum class BMP3_E : int8_t {
        NULL_PTR = -1,
        DEV_NOT_FOUND = -2,
        INVALID_ODR_OSR_SETTINGS = -3,
        CMD_EXEC_FAILED = -4,
        CONFIGURATION_ERR = -5,
        INVALID_LEN = -6,
        COMM_FAIL = -7,
        FIFO_WATERMARK_NOT_REACHED = -8
    };

    /**\name values to select the which sensor settings are to be set by the user.
     * These values are internal for API implementation. Don't relate this to
     * data sheet. */
    enum class BMP3_SEL : uint16_t {
        PRESS_EN = 1<<1,
        TEMP_EN = 1<<2,
        DRDY_EN = 1<<3,
        PRESS_OS = 1<<4,
        TEMP_OS = 1<<5,
        IIR_FILTER = 1<<6,
        ODR = 1<<7,
        OUTPUT_MODE = 1<<8,
        LEVEL = 1<<9,
        LATCH = 1<<10,
        I2C_WDT_EN = 1<<11,
        I2C_WDT = 1<<12,
        ALL = 0x7FF
    };

    /**\name values to select the which FIFO settings are to be set by the user
     * These values are internal for API implementation. Don't relate this to
     * data sheet.*/
    enum class BMP3_SEL_FIFO : uint16_t {
        MODE = 1<<1,
        STOP_ON_FULL_EN = 1<<2,
        TIME_EN = 1<<3,
        PRESS_EN = 1<<4,
        TEMP_EN = 1<<5,
        DOWN_SAMPLING = 1<<6,
        FILTER_EN_MODE = 1<<7,
        FWTM_EN = 1<<8,
        FULL_EN = 1<<9
    };

    enum class BMP3_MASKS : uint8_t {
        ERR_FATAL = 0x01,
        
        ERR_CMD = 0x02,
        ERR_CMD_POS = 0x01,
        
        ERR_CONF = 0x04,
        ERR_CONF_POS = 0x02,
        
        STATUS_CMD_RDY = 0x10,
        STATUS_CMD_RDY_POS = 0x04,
        
        STATUS_DRDY_PRESS = 0x20,
        STATUS_DRDY_PRESS_POS = 0x05,
        
        STATUS_DRDY_TEMP = 0x40,
        STATUS_DRDY_TEMP_POS = 0x06,
        
        OP_MODE = 0x30,
        OP_MODE_POS = 0x04,
        
        PRESS_EN = 0x01,
        
        TEMP_EN = 0x02,
        TEMP_EN_POS = 0x01,
        
        IIR_FILTER = 0x0E,
        IIR_FILTER_POS = 0x01,
        
        ODR = 0x1F,
        
        PRESS_OS = 0x07,
        
        TEMP_OS = 0x38,
        TEMP_OS_POS = 0x03,

        FIFO_MODE = 0x01,

        FIFO_STOP_ON_FULL = 0x02,
        FIFO_STOP_ON_FULL_POS = 0x01,

        FIFO_TIME_EN = 0x04,
        FIFO_TIME_EN_POS = 0x02,

        FIFO_PRESS_EN = 0x08,
        FIFO_PRESS_EN_POS = 0x03,

        FIFO_TEMP_EN = 0x10,
        FIFO_TEMP_EN_POS = 0x04,

        FIFO_FILTER_EN = 0x18,
        FIFO_FILTER_EN_POS = 0x03,

        FIFO_DOWN_SAMPLING = 0x07,

        FIFO_FWTM_EN = 0x08,
        FIFO_FWTM_EN_POS = 0x03,

        FIFO_FULL_EN = 0x10,
        FIFO_FULL_EN_POS = 0x04,

        INT_OUTPUT_MODE = 0x01,

        INT_LEVEL = 0x02,
        INT_LEVEL_POS = 0x01,

        INT_LATCH = 0x04,
        INT_LATCH_POS = 0x02,

        INT_DRDY_EN = 0x40,
        INT_DRDY_EN_POS = 0x06,

        I2C_WDT_EN = 0x02,
        I2C_WDT_EN_POS = 0x01,

        I2C_WDT_SEL = 0x04,
        I2C_WDT_SEL_POS = 0x02,

        INT_STATUS_FWTM = 0x01,

        INT_STATUS_FFULL = 0x02,
        INT_STATUS_FFULL_POS = 0x01,

        INT_STATUS_DRDY = 0x08,
        INT_STATUS_DRDY_POS = 0x03
    };

    enum class BMP3_LEN : uint8_t {
        CALIB_DATA = 21,
        P_AND_T_HEADER_DATA = 7,
        P_OR_T_HEADER_DATA = 4,
        P_T_DATA = 6,
        GEN_SETT = 7,
        P_DATA = 3,
        T_DATA = 3,
        SENSOR_TIME = 3,
        FIFO_MAX_FRAMES = 73
    };

    enum class BMP3_FIFO_FRAME : uint8_t {
        TEMP_PRESS = 0x94,
        TEMP = 0x90,
        PRESS = 0x84,
        TIME = 0xA0,
        ERROR = 0x44,
        CONFIG_CHANGE = 0x48
    };
}

/*!
 * @brief Interface selection Enums
 * Deprecated since we only use I2C.
 * This enum is kept for compatibility with existing code.
 */
enum class bmp3_intf {
    /*! SPI interface */
    BMP3_SPI_INTF,
    /*! I2C interface */
    BMP3_I2C_INTF
};

/*!
 * @brief Type definitions
 */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef BMP3_INTF_RET_TYPE (*bmp3_read_fptr_t)(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data to the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 *
 */
typedef BMP3_INTF_RET_TYPE (*bmp3_write_fptr_t)(uint8_t reg_addr, const uint8_t *read_data, uint32_t len,
                                                void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bmp3_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/*!
 * @brief Register Trim Variables
 */
struct bmp3_reg_calib_data {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    int64_t t_lin;
};

/*!
 * @brief bmp3 advance settings
 */
struct bmp3_adv_settings {
    /*! i2c watch dog enable */
    uint8_t i2c_wdt_en;
    /*! i2c watch dog select */
    uint8_t i2c_wdt_sel;
};

/*!
 * @brief bmp3 odr and filter settings
 */
struct bmp3_odr_filter_settings {
    /*! Pressure oversampling */
    uint8_t press_os;
    /*! Temperature oversampling */
    uint8_t temp_os;
    /*! IIR filter */
    uint8_t iir_filter;
    /*! Output data rate */
    uint8_t odr;
};

/*!
 * @brief bmp3 sensor status flags
 */
struct bmp3_sens_status {
    /*! Command ready status */
    uint8_t cmd_rdy;
    /*! Data ready for pressure */
    uint8_t drdy_press;
    /*! Data ready for temperature */
    uint8_t drdy_temp;
};

/*!
 * @brief bmp3 interrupt status flags
 */
struct bmp3_int_status {
    /*! fifo watermark interrupt */
    uint8_t fifo_wm;
    /*! fifo full interrupt */
    uint8_t fifo_full;
    /*! data ready interrupt */
    uint8_t drdy;
};

/*!
 * @brief bmp3 error status flags
 */
struct bmp3_err_status {
    /*! fatal error */
    uint8_t fatal;
    /*! command error */
    uint8_t cmd;
    /*! configuration error */
    uint8_t conf;
};

/*!
 * @brief bmp3 status flags
 */
struct bmp3_status {
    /*! Interrupt status */
    struct bmp3_int_status intr;
    /*! Sensor status */
    struct bmp3_sens_status sensor;
    /*! Error status */
    struct bmp3_err_status err;
    /*! power on reset status */
    uint8_t pwr_on_rst;
};

/*!
 * @brief bmp3 interrupt pin settings
 */
struct bmp3_int_ctrl_settings {
    /*! Output mode */
    uint8_t output_mode;
    /*! Active high/low */
    uint8_t level;
    /*! Latched or Non-latched */
    uint8_t latch;
    /*! Data ready interrupt */
    uint8_t drdy_en;
};

/*!
 * @brief bmp3 device settings
 */
struct bmp3_settings {
    /*! Power mode which user wants to set */
    uint8_t op_mode;
    /*! Enable/Disable pressure sensor */
    uint8_t press_en;
    /*! Enable/Disable temperature sensor */
    uint8_t temp_en;
    /*! ODR and filter configuration */
    struct bmp3_odr_filter_settings odr_filter;
    /*! Interrupt configuration */
    struct bmp3_int_ctrl_settings int_settings;
    /*! Advance settings */
    struct bmp3_adv_settings adv_settings;
};

/*!
 * @brief bmp3 fifo frame
 */
struct bmp3_fifo_data {
    /*! Data buffer of user defined length is to be mapped here
     * 512 + 4 */
    uint8_t *buffer;
    /*! Number of bytes of data read from the fifo */
    uint16_t byte_count;
    /*! Number of frames to be read as specified by the user */
    uint8_t req_frames;
    /*! Will be equal to length when no more frames are there to parse */
    uint16_t start_idx;
    /*! Will contain the no of parsed data frames from fifo */
    uint8_t parsed_frames;
    /*! Configuration error */
    uint8_t config_err;
    /*! Sensor time */
    uint32_t sensor_time;
    /*! FIFO input configuration change */
    uint8_t config_change;
    /*! All available frames are parsed */
    uint8_t frame_not_available;
};

/*!
 * @brief bmp3 fifo configuration
 */
struct bmp3_fifo_settings {
    /*! enable/disable */
    uint8_t mode;
    /*! stop on full enable/disable */
    uint8_t stop_on_full_en;
    /*! time enable/disable */
    uint8_t time_en;
    /*! pressure enable/disable */
    uint8_t press_en;
    /*! temperature enable/disable */
    uint8_t temp_en;
    /*! down sampling rate */
    uint8_t down_sampling;
    /*! filter enable/disable */
    uint8_t filter_en;
    /*! FIFO watermark enable/disable */
    uint8_t fwtm_en;
    /*! FIFO full enable/disable */
    uint8_t ffull_en;
};

/*!
 * @brief bmp3 bmp3 FIFO
 */
struct bmp3_fifo {
    /*! FIFO frame structure */
    struct bmp3_fifo_data data;
    /*! FIFO config structure */
    struct bmp3_fifo_settings settings;
};


/*!
 * @brief bmp3 sensor structure which comprises of temperature and pressure
 * data.
 */
struct bmp3_data {
    /*! Compensated temperature */
    int64_t temperature;
    /*! Compensated pressure */
    uint64_t pressure;
};

/*!
 * @brief Calibration data
 */
struct bmp3_calib_data {
    /*! Register data */
    struct bmp3_reg_calib_data reg_calib_data;
};


/*!
 * @brief bmp3 sensor structure which comprises of un-compensated temperature
 * and pressure data.
 */
struct bmp3_uncomp_data {
    /*! un-compensated pressure */
    uint32_t pressure;
    /*! un-compensated temperature */
    uint32_t temperature;
};

/*!
 * @brief bmp3 device structure
 */
struct bmp3_dev {
    /*! Chip Id */
    uint8_t chip_id;

    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *intf_ptr;

    /*! Interface Selection
     * For SPI, interface = BMP3_SPI_INTF
     * For I2C, interface = BMP3_I2C_INTF
     **/
    enum bmp3_intf intf;

    /*! To store interface pointer error */
    BMP3_INTF_RET_TYPE intf_rslt;

    /*! Decide SPI or I2C read mechanism */
    uint8_t dummy_byte;

    /*! Read function pointer */
    bmp3_read_fptr_t read;

    /*! Write function pointer */
    bmp3_write_fptr_t write;

    /*! Delay function pointer */
    bmp3_delay_us_fptr_t delay_us;

    /*! Trim data */
    struct bmp3_calib_data calib_data;

    /*! Sensor Settings */
    struct bmp3_settings settings;

    /*! Sensor and interrupt status flags */
    struct bmp3_status status;

    /*! FIFO data and settings structure */
    struct bmp3_fifo *fifo;
};

