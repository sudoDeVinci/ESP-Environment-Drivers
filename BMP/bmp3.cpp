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


/**
 * @brief This internal API gets the over sampling, ODR and filter settings
 * from the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t get_odr_filter_settings(struct bmp3_dev *dev);


/**
 * @brief This internal API is used to parse the pressure and temperature data
 * and store it in the bmp3_uncomp_data structure instance.
 *
 * @param[in] reg_data : Contains the register data which needs to be parsed.
 * @param[out] uncomp_data : Contains the uncompensated press and temp data.
 *
 */
static void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data);


/**
 * @brief This internal API is used to compensate the pressure or temperature
 * or both the data according to the component selected by the user.
 *
 * @param[in] sensor_comp : Used to select pressure or temperature.
 * @param[in] uncomp_data : Contains the uncompensated pressure and
 * temperature data.
 * @param[out] comp_data : Contains the compensated pressure and
 * temperature data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t compensate_data(uint8_t sensor_comp,
                              const struct bmp3_uncomp_data *uncomp_data,
                              struct bmp3_data *comp_data,
                              struct bmp3_calib_data *calib_data);


/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated temperature data.
 * @param[in] calib_data : Pointer to calibration data structure.
 *
 * @return Compensated temperature data.
 * @retval Compensated temperature data in integer.
 */
static int64_t compensate_temperature(const struct bmp3_uncomp_data *uncomp_data,
                                      struct bmp3_calib_data *calib_data);


/**
 * @brief This internal API is used to compensate the pressure data and return
 * the compensated pressure data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated pressure data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Compensated pressure data.
 * @retval Compensated pressure data in integer.
 */
static uint64_t compensate_pressure(const struct bmp3_uncomp_data *uncomp_data,
                                    const struct bmp3_calib_data *calib_data);


/**
 * @brief This internal API is used to calculate the power functionality.
 *
 * @param[in] base : Contains the base value.
 * @param[in] power : Contains the power value.
 *
 * @return Output of power function.
 * @retval Calculated power function output in integer.
 */
static uint32_t pow_bmp3(uint8_t base, uint8_t power);


/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 *
 * @param[in] sub_settings : Contains the settings subset to identify particular
 * group of settings which the user is interested to change.
 * @param[in] settings : Contains the user specified settings.
 *
 * @return Indicates whether user is interested to modify the settings which
 * are related to sub_settings.
 * @retval True -> User wants to modify this group of settings
 * @retval False -> User does not want to modify this group of settings
 */
static uint8_t are_settings_changed(uint32_t sub_settings, uint32_t settings);


/**
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 *
 * @param[in] reg_addr : Contains the register address array.
 * @param[out] temp_buff : Contains the temporary buffer to store the
 * register data and register address.
 * @param[in] reg_data : Contains the register data to be written in the
 * temporary buffer.
 * @param[in] len : No of bytes of data to be written for burst write.
 *
 */
static void interleave_reg_addr(const uint8_t *reg_addr,
                                uint8_t *temp_buff,
                                const uint8_t *reg_data, uint32_t len);


/**
 * @brief This internal API sets the pressure enable and
 * temperature enable settings of the sensor.
 *
 * @param[in] desired_settings : Contains the settings which user wants to
 * change.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t set_pwr_ctrl_settings(uint32_t desired_settings, struct bmp3_dev *dev);


/**
 * @brief This internal API sets the over sampling, ODR and filter settings of
 * the sensor based on the settings selected by the user.
 *
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t set_odr_filter_settings(uint32_t desired_settings, struct bmp3_dev *dev);


/**
 * @brief This internal API sets the interrupt control (output mode, level,
 * latch and data ready) settings of the sensor based on the settings
 * selected by the user.
 *
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t set_int_ctrl_settings(uint32_t desired_settings, struct bmp3_dev *dev);


/**
 * @brief This internal API sets the advance (i2c_wdt_en, i2c_wdt_sel)
 * settings of the sensor based on the settings selected by the user.
 *
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t set_advance_settings(uint32_t desired_settings, struct bmp3_dev *dev);


/**
 * @brief This internal API fills the register address and register data of the
 * the over sampling settings for burst write operation.
 *
 * @param[in] desired_settings : Variable which specifies the settings which
 * are to be set in the sensor.
 * @param[out] addr : To store the address to fill in register buffer.
 * @param[out] reg_data : To store the osr register data.
 * @param[out] len : To store the len for burst write.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 */
static void fill_osr_data(uint32_t desired_settings,
                          uint8_t *addr,
                          uint8_t *reg_data,
                          uint8_t *len,
                          const struct bmp3_dev *dev);
















