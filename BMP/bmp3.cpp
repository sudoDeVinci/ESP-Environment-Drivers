#include "BMP/bmp3.hpp"


/**
 * @brief This internal API reads the calibration data from the sensor, parse
 * it then compensates it and store in the device structure.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t get_calib_data(struct bmp3_dev *dev);


/**
 * @brief This internal API is used to parse the calibration data, compensates
 * it and store it in device structure.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[out] reg_data : Contains calibration data to be parsed.
 *
 */
static void parse_calib_data(const uint8_t *reg_data, struct bmp3_dev *dev);





































