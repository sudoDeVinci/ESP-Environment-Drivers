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


enum BMP_REGS {
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

enum BMP3_ERR {
    FATAL = uint8_t(0x01),
    CMD = uint8_t(0x02),
    CONF = uint8_t(0x04),
};

enum BMP3_DRDY {
    CMD = uint8_t(0x10),
    PRESS = uint8_t(0x20),
    TEMP = uint8_t(0x40)
};

enum BMP3_MODE {
    SLEEP = uint8_t(0x00),
    FORCED = uint8_t(0x01),
    NORMAL = uint8_t(0x03)
};

enum BMP3_INT_PIN {
    OPEN_DRAIN = uint8_t(0x01),
    PUSH_PULL = uint8_t(0x00),
    ACTIVE_HIGH = uint8_t(0x01),
    ACTIVE_LOW = uint8_t(0x00),
    LATCH = uint8_t(0x01),
    NON_LATCH = uint8_t(0x00)
};

enum BMP3_FIFO_SUBSAMPLING {
    NO_SUBSAMPLING = uint8_t(0x00),
    SUBSAMPLING_2X = uint8_t(0x01),
    SUBSAMPLING_4X = uint8_t(0x02),
    SUBSAMPLING_8X = uint8_t(0x03),
    SUBSAMPLING_16X = uint8_t(0x04),
    SUBSAMPLING_32X = uint8_t(0x05),
    SUBSAMPLING_64X = uint8_t(0x06),
    SUBSAMPLING_128X = uint8_t(0x07)
};

enum BMP3_OVERSAMPLING {
    NO_OVERSAMPLING = uint8_t(0x00),
    OVERSAMPLING_2X = uint8_t(0x01),
    OVERSAMPLING_4X = uint8_t(0x02),
    OVERSAMPLING_8X = uint8_t(0x03),
    OVERSAMPLING_16X = uint8_t(0x04),
    OVERSAMPLING_32X = uint8_t(0x05),
};

enum BMP3_IIR_FILTER {
    DISABLE = uint8_t(0x00),
    COEFF_1 = uint8_t(0x01),
    COEFF_3 = uint8_t(0x02),
    COEFF_7 = uint8_t(0x03),
    COEFF_15 = uint8_t(0x04),
    COEFF_31 = uint8_t(0x05),
    COEFF_63 = uint8_t(0x06),
    COEFF_127 = uint8_t(0x07)
};

enum BMP3_ODR {
    ODR_200_HZ = uint8_t(0x00),
    ODR_100_HZ = uint8_t(0x01),
    ODR_50_HZ = uint8_t(0x02),
    ODR_25_HZ = uint8_t(0x03),
    ODR_12_5_HZ = uint8_t(0x04),
    ODR_6_25_HZ = uint8_t(0x05),
    ODR_3_1_HZ = uint8_t(0x06),
    ODR_1_5_HZ = uint8_t(0x07),
    ODR_0_78_HZ = uint8_t(0x08),
    ODR_0_39_HZ = uint8_t(0x09),
    ODR_0_2_HZ = uint8_t(0x0A),
    ODR_0_1_HZ = uint8_t(0x0B),
    ODR_0_05_HZ = uint8_t(0x0C),
    ODR_0_02_HZ = uint8_t(0x0D),
    ODR_0_01_HZ = uint8_t(0x0E),
    ODR_0_006_HZ = uint8_t(0x0F),
    ODR_0_003_HZ = uint8_t(0x10),
    ODR_0_001_HZ = uint8_t(0x11)
};

enum BMP3_E {
    NULL_PTR = int8_t(-1),
    DEV_NOT_FOUND = int8_t(-2),
    INVALID_ODR_OSR_SETTINGS = int8_t(-3),
    CMD_EXEC_FAILED = int8_t(-4),
    CONFIGURATION_ERR = int8_t(-5),
    INVALID_LEN = int8_t(-6),
    COMM_FAIL = int8_t(-7),
    FIFO_WATERMARK_NOT_REACHED = int8_t(-8),
};

/**\name Macros to select the which sensor settings are to be set by the user.
 * These values are internal for API implementation. Don't relate this to
 * data sheet. */
enum BMP3_SEL {
    PRESS_EN = uint16_t(1<<1),
    TEMP_EN = uint16_t(1<<2),
    DRDY_EN = uint16_t(1<<3),
    PRESS_OS = uint16_t(1<<4),
    TEMP_OS = uint16_t(1<<5),
    IIR_FILTER = uint16_t(1<<6),
    ODR = uint16_t(1<<7),
    OUTPUT_MODE = uint16_t(1<<8),
    LEVEL = uint16_t(1<<9),
    LATCH = uint16_t(1<<10),
    I2C_WDT_EN = uint16_t(1<<11),
    I2C_WDT = uint16_t(1<<12),
    ALL = uint16_t(0x7FF)
};

/**\name Macros to select the which FIFO settings are to be set by the user
 * These values are internal for API implementation. Don't relate this to
 * data sheet.*/
enum BMP3_SEL_FIFO {
    MODE = uint16_t(1<<1),
    STOP_ON_FULL_EN = uint16_t(1<<2),
    TIME_EN = uint16_t(1<<3),
    PRESS_EN = uint16_t(1<<4),
    TEMP_EN = uint16_t(1<<5),
    DOWN_SAMPLING = uint16_t(1<<6),
    FILTER_EN = uint16_t(1<<7),
    FWTM_EN = uint16_t(1<<8),
    FULL_EN = uint16_t(1<<9)
};

enum BMP3_MASKS {
    ERR_FATAL = uint8_t(0x01),
    
    ERR_CMD = uint8_t(0x02),
    ERR_CMD_POS = uint8_t(0x01),
    
    ERR_CONF = uint8_t(0x04),
    ERR_CONF_POS = uint8_t(0x02),
    
    STATUS_CMD_RDY = uint8_t(0x10),
    STATUS_CMD_RDY_POS = uint8_t(0x04),
    
    STATUS_DRDY_PRESS = uint8_t(0x20),
    STATUS_DRDY_PRESS_POS = uint8_t(0x05),
    
    STATUS_DRDY_TEMP = uint8_t(0x40),
    STATUS_DRDY_TEMP_POS = uint8_t(0x06),
    
    OP_MODE = uint8_t(0x30),
    OP_MODE_POS = uint8_t(0x04),
    
    PRESS_EN = uint8_t(0x01),
    
    TEMP_EN = uint8_t(0x02),
    TEMP_EN_POS = uint8_t(0x01),
    
    IIR_FILTER = uint8_t(0x0E),
    IIR_FILTER_POS = uint8_t(0x01),
    
    ODR = uint8_t(0x1F),
    
    PRESS_OS = uint8_t(0x07),
    
    TEMP_OS = uint8_t(0x38),
    TEMP_OS_POS = uint8_t(0x03),

    FIFO_MODE = uint8_t(0x01),

    FIFO_STOP_ON_FULL = uint8_t(0x02),
    FIFO_STOP_ON_FULL_POS = uint8_t(0x01),

    FIFO_TIME_EN = uint8_t(0x04),
    FIFO_TIME_EN_POS = uint8_t(0x02)
};












