#pragma once

/**
 * Definitions taken from the BMP3xx datasheet and the bmp3_defs.h file from
 * The adafruit BMP3xx library @ https://github.com/adafruit/Adafruit_BMP3XX/blob/master/bmp3_defs.h
 */

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

/**\name    UTILITY MACROS  */
#define BMP3_SET_LOW_BYTE                       UINT16_C(0x00FF)
#define BMP3_SET_HIGH_BYTE                      UINT16_C(0xFF00)

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


namespace BMP3 {

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
        P_AND_T_HEADER_DATA = 7
    };

}







