#pragma once

#define BMP3_ADDR_PRIM  0x76
#define BMP3_ADDR_SEC   0x77

/**\name BMP3 pressure settling time (micro secs)*/
#define BMP3_SETTLE_TIME_PRESS  392

/**\name BMP3 temperature settling time (micro secs) */
#define BMP3_SETTLE_TIME_TEMP   313

/**\name BMP3 adc conversion time (micro secs) */
#define BMP3_ADC_CONV_TIME      2000

enum REGISTERS {
    CHIP_ID = 0x00,
    ERR = 0x02,
    SENS_STATUS = 0x03
};
