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


/**
 * @brief This internal API fills the register address and register data of the
 * the ODR settings for burst write operation.
 *
 * @param[out] addr : To store the address to fill in register buffer.
 * @param[out] reg_data : To store the register data to set the odr data.
 * @param[out] len : To store the len for burst write.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 */
static void fill_odr_data(uint8_t *addr, uint8_t *reg_data, uint8_t *len, struct bmp3_dev *dev);


/**
 * @brief This internal API fills the register address and register data of the
 * the filter settings for burst write operation.
 *
 * @param[out] addr : To store the address to fill in register buffer.
 * @param[out] reg_data : To store the register data to set the filter.
 * @param[out] len : To store the len for burst write.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 */
static void fill_filter_data(uint8_t *addr, uint8_t *reg_data, uint8_t *len, const struct bmp3_dev *dev);


/**
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t null_ptr_check(const struct bmp3_dev *dev);


/**
 * @brief This internal API parse the power control(power mode, pressure enable
 * and temperature enable), over sampling, ODR, filter and interrupt control
 * settings and store in the device structure.
 *
 * @param[in] reg_data : Register data to be parsed.
 * @param[out] dev : Structure instance of bmp3_dev.
 */
static void parse_sett_data(const uint8_t *reg_data, struct bmp3_dev *dev);


/**
 * @brief This internal API parse the power control(power mode, pressure enable
 * and temperature enable) settings and store in the device structure.
 *
 * @param[in] reg_data : Pointer variable which stores the register data to
 * parse.
 * @param[out] settings : Structure instance of bmp3_settings.
 */
static void parse_pwr_ctrl_settings(const uint8_t *reg_data, struct bmp3_settings *settings);


/**
 * @brief This internal API parse the over sampling, ODR and filter
 * settings and store in the device structure.
 *
 * @param[in] reg_data : Pointer variable which stores the register data to
 * parse.
 * @param[out] settings : Structure instance of bmp3_odr_filter_settings.
 */
static void parse_odr_filter_settings(const uint8_t *reg_data, struct bmp3_odr_filter_settings *settings);


/**
 * @brief This internal API parse the interrupt control(output mode, level,
 * latch and data ready) settings and store in the device structure.
 *
 * @param[in] reg_data : Pointer variable which stores the register data to
 * parse.
 * @param[out] settings : Structure instance of bmp3_int_ctrl_settings.
 */
static void parse_int_ctrl_settings(const uint8_t *reg_data, struct bmp3_int_ctrl_settings *settings);


/**
 * @brief This internal API parse the advance (i2c_wdt_en, i2c_wdt_sel)
 * settings and store in the device structure.
 *
 * @param[in] reg_data : Pointer variable which stores the register data to
 * parse.
 * @param[out] settings : Structure instance of bmp3_adv_settings.
 */
static void parse_advance_settings(const uint8_t *reg_data, struct bmp3_adv_settings *settings);


/**
 * @brief This internal API validate the normal mode settings of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t validate_normal_mode_settings(struct bmp3_dev *dev);


/**
 * @brief This internal API validate the over sampling, ODR settings of the
 * sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Indicates whether ODR and OSR settings are valid or not.
 * @retval 0 -> Success
 * @retval <0 -> Fail
 */
static int8_t validate_osr_and_odr_settings(const struct bmp3_dev *dev);


/**
 * @brief This internal API calculates the pressure measurement duration of the
 * sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Pressure measurement time
 * @retval Pressure measurement time in microseconds
 */
static uint32_t calculate_press_meas_time(const struct bmp3_dev *dev);


/**
 * @brief This internal API calculates the temperature measurement duration of
 * the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Temperature measurement time
 * @retval Temperature measurement time in microseconds
 */
static uint32_t calculate_temp_meas_time(const struct bmp3_dev *dev);


/**
 * @brief This internal API checks whether the measurement time and ODR duration
 * of the sensor are proper.
 *
 * @param[in] meas_t : Pressure and temperature measurement time in microseconds.
 * @param[in] odr_duration : Duration in microseconds corresponding to the ODR
 *                           value.
 *
 * @return Indicates whether ODR and OSR settings are valid or not.
 * @retval 0 -> Success
 * @retval >0 -> Warning
 */
static int8_t verify_meas_time_and_odr_duration(uint32_t meas_t, uint32_t odr_duration);


/**
 * @brief This internal API puts the device to sleep mode.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status.
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t put_device_to_sleep(struct bmp3_dev *dev);


/**
 * @brief This internal API sets the normal mode in the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status.
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t set_normal_mode(struct bmp3_dev *dev);


/**
 * @brief This internal API writes the power mode in the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status.
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t write_power_mode(struct bmp3_dev *dev);


/**
 * @brief This internal API fills the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en) settings in the
 * reg_data variable so as to burst write in the sensor.
 *
 * @param[in] desired_settings : Variable which specifies the settings which
 * are to be set in the sensor.
 * @param[out] reg_data : Pointer variable where the fifo_config_1 register
 * data will be stored so as to burst write in the register.
 * @param[in] dev_fifo : Structure instance of bmp3_fifo_settings which
 * contains the fifo_config_1 values set by the user.
 */
static int8_t fill_fifo_config_1(uint16_t desired_settings, struct bmp3_dev *dev);


/**
 * @brief This internal API fills the fifo_config_2(fifo_sub_sampling,
 * data_select) settings in the reg_data variable so as to burst write
 * in the sensor.
 *
 * @param[in] desired_settings : Variable which specifies the settings which
 * are to be set in the sensor.
 * @param[out] reg_data : Pointer variable where the fifo_config_2 register
 * data will be stored so as to burst write in the register.
 * @param[in] dev_fifo : Structure instance of bmp3_fifo_settings which
 * contains the fifo_config_2 values set by the user.
 */
static int8_t fill_fifo_config_2(uint16_t desired_settings, struct bmp3_dev *dev);


/**
 * @brief This internal API fills the fifo interrupt control(fwtm_en, ffull_en)
 * settings in the reg_data variable so as to burst write in the sensor.
 *
 * @param[in] desired_settings : Variable which specifies the settings which
 * are to be set in the sensor.
 * @param[out] reg_data : Pointer variable where the fifo interrupt control
 * register data will be stored so as to burst write in the register.
 * @param[in] dev_fifo : Structure instance of bmp3_fifo_settings which
 * contains the fifo interrupt control values set by the user.
 */
static int8_t fill_fifo_int_ctrl(uint16_t desired_settings, struct bmp3_dev *dev);


/**
 * @brief This internal API is used to parse the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings and store it in device structure
 *
 * @param[in] reg_data : Pointer variable which stores the fifo settings data
 * read from the sensor.
 * @param[out] dev_fifo : Structure instance of bmp3_fifo_settings which
 * contains the fifo settings after parsing.
 */
static void parse_fifo_settings(const uint8_t *reg_data, struct bmp3_fifo_settings *dev_fifo);


/**
 * @brief This internal API parse the FIFO data frame from the fifo buffer and
 * fills the byte count, uncompensated pressure and/or temperature data and no
 * of parsed frames.
 *
 * @param[in] header : Pointer variable which stores the fifo settings data
 * read from the sensor.
 * @param[in,out] fifo : Structure instance of bmp3_fifo which stores the
 * read fifo data.
 * @param[out] byte_index : Byte count which is incremented according to the
 * of data.
 * @param[out] uncomp_data : Uncompensated pressure and/or temperature data
 * which is stored after parsing fifo buffer data.
 * @param[out] parsed_frames : Total number of parsed frames.
 *
 * @return Result of API execution status.
 * @retval 0 -> Success
 * @retval <0 -> Fail
 */
static uint8_t parse_fifo_data_frame(uint8_t header,
                                     struct bmp3_fifo *fifo,
                                     uint16_t *byte_index,
                                     struct bmp3_uncomp_data *uncomp_data,
                                     uint8_t *parsed_frames);


/**
 * @brief This internal API unpacks the FIFO data frame from the fifo buffer and
 * fills the byte count, uncompensated pressure and/or temperature data.
 *
 * @param[out] byte_index : Byte count of fifo buffer.
 * @param[in] fifo_buffer : FIFO buffer from where the temperature and pressure
 * frames are unpacked.
 * @param[out] uncomp_data : Uncompensated temperature and pressure data after
 * unpacking from fifo buffer.
 */
static void unpack_temp_press_frame(uint16_t *byte_index,
                                    const uint8_t *fifo_buffer,
                                    struct bmp3_uncomp_data *uncomp_data);


/**
 * @brief This internal API unpacks the FIFO data frame from the fifo buffer and
 * fills the byte count and uncompensated pressure data.
 *
 * @param[out] byte_index : Byte count of fifo buffer.
 * @param[in] fifo_buffer : FIFO buffer from where the pressure frames are
 * unpacked.
 * @param[out] uncomp_data : Uncompensated pressure data after unpacking from
 * fifo buffer.
 */
static void unpack_press_frame(uint16_t *byte_index, const uint8_t *fifo_buffer, struct bmp3_uncomp_data *uncomp_data);


/**
 * @brief This internal API unpacks the FIFO data frame from the fifo buffer and
 * fills the byte count and uncompensated temperature data.
 *
 * @param[out] byte_index : Byte count of fifo buffer.
 * @param[in] fifo_buffer : FIFO buffer from where the temperature frames
 * are unpacked.
 * @param[out] uncomp_data : Uncompensated temperature data after unpacking from
 * fifo buffer.
 */
static void unpack_temp_frame(uint16_t *byte_index, const uint8_t *fifo_buffer, struct bmp3_uncomp_data *uncomp_data);


/**
 * @brief This internal API unpacks the time frame from the fifo data buffer and
 * fills the byte count and update the sensor time variable.
 *
 * @param[out] byte_index : Byte count of fifo buffer.
 * @param[in] fifo_buffer : FIFO buffer from where the sensor time frames
 * are unpacked.
 * @param[out] sensor_time : Variable used to store the sensor time.
 */
static void unpack_time_frame(uint16_t *byte_index, const uint8_t *fifo_buffer, uint32_t *sensor_time);


/**
 * @brief This internal API parses the FIFO buffer and gets the header info.
 *
 * @param[out] header : Variable used to store the fifo header data.
 * @param[in] fifo_buffer : FIFO buffer from where the header data is retrieved.
 * @param[out] byte_index : Byte count of fifo buffer.
 */
static void get_header_info(uint8_t *header, const uint8_t *fifo_buffer, uint16_t *byte_index);


/**
 * @brief This internal API parses the FIFO data frame from the fifo buffer and
 * fills uncompensated temperature and/or pressure data.
 *
 * @param[in] sensor_comp : Variable used to select either temperature or
 * pressure or both while parsing the fifo frames.
 * @param[in] fifo_buffer : FIFO buffer where the temperature or pressure or
 * both the data to be parsed.
 * @param[out] uncomp_data : Uncompensated temperature or pressure or both the
 * data after unpacking from fifo buffer.
 */
static void parse_fifo_sensor_data(uint8_t sensor_comp, const uint8_t *fifo_buffer,
                                   struct bmp3_uncomp_data *uncomp_data);


/**
 * @brief This internal API resets the FIFO buffer, start index,
 * parsed frame count, configuration change, configuration error and
 * frame_not_available variables.
 *
 * @param[out] fifo : FIFO structure instance where the fifo related variables
 * are reset.
 */
static void reset_fifo_index(struct bmp3_fifo *fifo);


/**
 * @brief This API gets the command ready, data ready for pressure and
 * temperature, power on reset status from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval 0 -> Success
 * @retval <0 -> Fail
 */
static int8_t get_sensor_status(struct bmp3_dev *dev);


/**
 * @brief This API gets the interrupt (fifo watermark, fifo full, data ready)
 * status from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval 0 -> Success
 * @retval <0 -> Fail
 */
static int8_t get_int_status(struct bmp3_dev *dev);


/**
 * @brief This API gets the fatal, command and configuration error
 * from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval 0 -> Success
 * @retval <0 -> Fail
 */
static int8_t get_err_status(struct bmp3_dev *dev);


/**
 * @brief This internal API converts the no. of frames required by the user to
 * bytes so as to write in the watermark length register.
 *
 * @param[in] dev : Structure instance of bmp3_dev
 * @param[out] watermark_len : Pointer variable which contains the watermark
 * length.
 *
 * @return Result of API execution status.
 * @retval 0 -> Success
 * @retval <0 -> Fail
 */
static int8_t convert_frames_to_bytes(uint16_t *watermark_len, const struct bmp3_dev *dev);



/**
 * @brief This API is the entry point.
 * It performs the selection of I2C/SPI read mechanism according to the
 * selected interface and reads the chip-id and calibration data of the sensor.
 * @param[in,out] dev : Structure instance of bmp3_dev.
 * @return Result of API execution status
 * 
 */
int8_t bmp3_init(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Early exit if null check failed */
    if (rslt != BMP3_OK) return rslt;

    /* Read the chip-id of bmp3 sensor */
    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::CHIP_ID), &chip_id, 1, dev);

    /* Early return if chip id isn't for generic BMP or BMP390*/
    if ((chip_id != BMP3_CHIP_ID) && (chip_id != BMP390_CHIP_ID)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::DEV_NOT_FOUND);
        return rslt;
    }

    dev->chip_id = chip_id;

    /* Reset the sensor and read calibration data if all is good*/
    rslt = bmp3_soft_reset(dev);
    if (rslt == BMP3_OK) rslt = get_calib_data(dev);

    return rslt;
}


/**
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t bmp3_get_regs(
    uint8_t reg_addr,
    uint8_t *reg_data,
    uint32_t len,
    struct bmp3_dev *dev
) {
    int8_t rslt;
    uint32_t idx;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt != BMP3_OK) || (reg_data == NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }

    uint32_t temp_len = len + dev->dummy_byte;
    uint8_t temp_buff[len + dev->dummy_byte];

    /* Read the data using I2C */
    dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);

    /* Check for communication error */
    if (dev->intf_rslt != BMP3_INTF_RET_SUCCESS) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::COMM_FAIL);
    }

    return rslt;
}


/**
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t bmp3_set_regs(
    uint8_t *reg_addr,
    const uint8_t *reg_data,
    uint32_t len,
    struct bmp3_dev *dev
) {
    int8_t rslt;
    uint8_t temp_buff[len * 2];
    uint32_t temp_len;
    uint8_t reg_addr_cnt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if ((rslt != BMP3_OK) || (reg_addr == NULL) || (reg_data == NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }


    if (len == 0) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::INVALID_LEN);
        return rslt;
    }

    temp_buff[0] = reg_data[0];

    /* Burst write mode */
    if (len > 1) {
        /* 
         * Interleave register address w.r.t data for
         * burst write
         */
        interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
        temp_len = len * 2;
    } else {
        temp_len = len;
    }

    dev->intf_rslt = dev->write(reg_addr[0], temp_buff, temp_len, dev->intf_ptr);

    /* Check for communication error */
    if (dev->intf_rslt != BMP3_INTF_RET_SUCCESS) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::COMM_FAIL);
    }


    return rslt;
}


/**
 * @brief This API sets the power control(pressure enable and
 * temperature enable), over sampling, ODR and filter
 * settings in the sensor.
 */
int8_t bmp3_set_sensor_settings(uint32_t desired_settings, struct bmp3_dev *dev) {
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt != BMP3_OK) return rslt;

    if (are_settings_changed(BMP3_POWER_CNTL, desired_settings)) {
        /* Set the power control settings */
        rslt = set_pwr_ctrl_settings(desired_settings, dev);
    }

    if (are_settings_changed(BMP3_ODR_FILTER, desired_settings) && (!rslt)) {
        /* Set the over sampling, ODR and filter settings*/
        rslt = set_odr_filter_settings(desired_settings, dev);
    }

    if (are_settings_changed(BMP3_INT_CTRL, desired_settings) && (!rslt)) {
        /* Set the interrupt control settings */
        rslt = set_int_ctrl_settings(desired_settings, dev);
    }

    if (are_settings_changed(BMP3_ADV_SETT, desired_settings) && (!rslt)) {
        /* Set the advance settings */
        rslt = set_advance_settings(desired_settings, dev);
    }

    return rslt;
}


/**
 * @brief This API gets the power control(power mode, pressure enable and
 * temperature enable), over sampling, ODR, filter, interrupt control and
 * advance settings from the sensor.
 */
int8_t bmp3_get_sensor_settings(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t settings_data[static_cast<int8_t>(bmp::BMP3_LEN::GEN_SETT)];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt != BMP3_OK) return rslt;


    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::INT_CTRL), settings_data, static_cast<uint32_t>(bmp::BMP3_LEN::GEN_SETT), dev);

    if (rslt == BMP3_OK) {
        /* Parse the settings data */
        parse_sett_data(settings_data, dev);
    }

    return rslt;
}


/**
 * @brief This API sets the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings in the sensor.
 */
int8_t bmp3_set_fifo_settings(uint16_t desired_settings, struct bmp3_dev *dev) {
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt != BMP3_OK) || (dev->fifo == NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }

    if (are_settings_changed(BMP3_FIFO_CONFIG_1, desired_settings)) {
        /* Fill the FIFO config 1 register data */
        rslt = fill_fifo_config_1(desired_settings, dev);
    }

    if (are_settings_changed(desired_settings, BMP3_FIFO_CONFIG_2)) {
        /* Fill the FIFO config 2 register data */
        rslt = fill_fifo_config_2(desired_settings, dev);
    }

    if (are_settings_changed(desired_settings, BMP3_FIFO_INT_CTRL)) {
        /* Fill the FIFO interrupt ctrl register data */
        rslt = fill_fifo_int_ctrl(desired_settings, dev);
    }


    return rslt;
}


/**
 * @brief This API gets the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings from the sensor.
 */
int8_t bmp3_get_fifo_settings(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t fifo_sett[3];
    uint8_t len = 3;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt != BMP3_OK) || (dev->fifo == NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }

    /* Read the fifo settings */
    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::FIFO_CONFIG_1), fifo_sett, len, dev);

    /* Parse the fifo settings */
    parse_fifo_settings(fifo_sett, &dev->fifo->settings);

    return rslt;
}


/**
 * @brief This API gets the fifo data from the sensor.
 */
int8_t bmp3_get_fifo_data(struct bmp3_dev *dev) {
    int8_t rslt;
    uint16_t fifo_len;

    rslt = null_ptr_check(dev);

    if ((rslt != BMP3_OK) || (dev->fifo == NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }


    struct bmp3_fifo *fifo = dev->fifo;

    reset_fifo_index(dev->fifo);

    /* Get the total no of bytes available in FIFO */
    rslt = bmp3_get_fifo_length(&fifo_len, dev);

    /* For sensor time frame */
    if (dev->fifo->settings.time_en == TRUE) {
        fifo_len = fifo_len + 4;
    }

    /* Update the fifo length in the fifo structure */
    dev->fifo->data.byte_count = fifo_len;

    if (rslt == BMP3_OK) {
        /* Read the fifo data */
        rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::FIFO_DATA), fifo->data.buffer, fifo_len, dev);
    }

    return rslt;
}


/**
 * @brief This API sets the fifo watermark length according to the frames count
 * set by the user in the device structure. Refer below for usage.
 */
int8_t bmp3_set_fifo_watermark(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t reg_data[2];
    uint8_t reg_addr[2] = {
        static_cast<uint8_t>(bmp::BMP_REGS::FIFO_WM),
        static_cast<uint8_t>(bmp::BMP_REGS::FIFO_WM) + 1
    };
    uint16_t watermark_len;

    rslt = null_ptr_check(dev);

    if ((rslt != BMP3_OK) || (dev->fifo == NULL)) return rslt;


    rslt = convert_frames_to_bytes(&watermark_len, dev);

    if (rslt != BMP3_OK) return rslt;

    reg_data[0] = BMP3_GET_LSB(watermark_len);
    reg_data[1] = BMP3_GET_MSB(watermark_len) & 0x01;
    rslt = bmp3_set_regs(reg_addr, reg_data, 2, dev);

    return rslt;
}



/**
 * @brief This API extracts the temperature and/or pressure data from the FIFO
 * data which is already read from the fifo.
 */
int8_t bmp3_extract_fifo_data(struct bmp3_data *data, struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t header;
    uint8_t parsed_frames = 0;
    uint8_t t_p_frame;
    struct bmp3_uncomp_data uncomp_data;

    rslt = null_ptr_check(dev);

    if ((rslt != BMP3_OK) || (dev->fifo == NULL) || (data == NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }

    uint16_t byte_index = dev->fifo->data.start_idx;

    while ((parsed_frames < (dev->fifo->data.req_frames)) && (byte_index < dev->fifo->data.byte_count)) {
        get_header_info(&header, dev->fifo->data.buffer, &byte_index);
        t_p_frame = parse_fifo_data_frame(header, dev->fifo, &byte_index, &uncomp_data, &parsed_frames);

        /* If the frame is pressure and/or temperature data */
        if (t_p_frame != FALSE) {
            /* Compensate temperature and pressure data */
            rslt = compensate_data(t_p_frame, &uncomp_data, &data[parsed_frames - 1], &dev->calib_data);
        }
    }

    /* Check if any frames are parsed in FIFO */
    if (parsed_frames != 0) {
        /* Update the bytes parsed in the device structure */
        dev->fifo->data.start_idx = byte_index;
        dev->fifo->data.parsed_frames += parsed_frames;
    }
    else {
        /* No frames are there to parse. It is time to
         * read the FIFO, if more frames are needed */
        dev->fifo->data.frame_not_available = TRUE;
    }


    return rslt;
}


/**
 * @brief This API gets the command ready, data ready for pressure and
 * temperature and interrupt (fifo watermark, fifo full, data ready) and
 * error status from the sensor.
 */
int8_t bmp3_get_status(struct bmp3_dev *dev) {
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    if (rslt != BMP3_OK) return rslt;

    rslt = get_sensor_status(dev);

    /* Proceed further if the earlier operation is fine */
    if (rslt != BMP3_OK) return rslt;

    rslt = get_int_status(dev);

    /* Proceed further if the earlier operation is fine */
    if (rslt == BMP3_OK) rslt = get_err_status(dev);

    return rslt;
}


/**
 * @brief This API gets the fifo length from the sensor.
 */
int8_t bmp3_get_fifo_length(uint16_t *fifo_length, struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t reg_data[2];

    rslt = null_ptr_check(dev);

    if ((rslt != BMP3_OK) || (fifo_length == NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }

    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::FIFO_LENGTH), reg_data, 2, dev);

    /* Proceed if read from register is fine */
    if (rslt == BMP3_OK) {
        /* Update the fifo length */
        *fifo_length = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
    }

    return rslt;
}


/**
 * @brief This API performs the soft reset of the sensor.
 */
int8_t bmp3_soft_reset(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t reg_addr = static_cast<uint8_t>(bmp::BMP_REGS::CMD);

    /* 0xB6 is the soft reset command */
    uint8_t soft_rst_cmd = BMP3_SOFT_RESET;
    uint8_t cmd_rdy_status;
    uint8_t cmd_err_status;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    if (rslt != BMP3_OK) return rslt;

    /* Check for command ready status */
    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::SENS_STATUS), &cmd_rdy_status, 1, dev);

    /* Device is ready to accept new command */
    if (!(cmd_rdy_status & static_cast<uint8_t>(bmp::BMP3_DRDY::CMD)) || (rslt != BMP3_OK)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::CMD_EXEC_FAILED);
        return rslt;
    }
    
    /* Write the soft reset command in the sensor */
    rslt = bmp3_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);

    /* Proceed if everything is fine until now */
    if (rslt != BMP3_OK) return rslt;

    /* Wait for 2 ms */
    dev->delay_us(2000, dev->intf_ptr);

    /* Read for command error status */
    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::ERR), &cmd_err_status, 1, dev);

    /* check for command error status */
    if ((cmd_err_status & static_cast<uint8_t>(bmp::BMP_REGS::CMD)) || (rslt != BMP3_OK)) {
        /* Command not written hence return error */
        rslt = static_cast<int8_t>(bmp::BMP3_E::CMD_EXEC_FAILED);
    }

    return rslt;
}


/**
 * @brief This API performs the soft reset of the sensor.
 */
int8_t bmp3_fifo_flush(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t reg_addr = static_cast<uint8_t>(bmp::BMP_REGS::CMD);

    uint8_t fifo_flush_cmd = BMP3_FIFO_FLUSH;
    uint8_t cmd_rdy_status;
    uint8_t cmd_err_status;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt != BMP3_OK) return rslt;

    /* Check for command ready status */
    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::SENS_STATUS), &cmd_rdy_status, 1, dev);

    /* Device is ready to accept new command */
    if (!(cmd_rdy_status & static_cast<uint8_t>(bmp::BMP3_DRDY::CMD)) || (rslt != BMP3_OK)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::CMD_EXEC_FAILED);
        return rslt;
    }

    /* Write the soft reset command in the sensor */
    rslt = bmp3_set_regs(&reg_addr, &fifo_flush_cmd, 1, dev);

    /* Proceed if everything is fine until now */
    if (rslt != BMP3_OK) return rslt;
    
    /* Wait for 2 ms */
    dev->delay_us(2000, dev->intf_ptr);

    /* Read for command error status */
    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::ERR), &cmd_err_status, 1, dev);

    /* check for command error status */
    if ((cmd_err_status & static_cast<uint8_t>(bmp::BMP_REGS::CMD)) || (rslt != BMP3_OK)) {
        /* Command not written hence return error */
        rslt = static_cast<int8_t>(bmp::BMP3_E::CMD_EXEC_FAILED);
    }

    return rslt;
}


/**
 * @brief This API sets the power mode of the sensor.
 */
int8_t bmp3_set_op_mode(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t last_set_mode;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    if (rslt != BMP3_OK) return rslt;
    
    uint8_t curr_mode = dev->settings.op_mode;

    rslt = bmp3_get_op_mode(&last_set_mode, dev);

    /* If the sensor is not in sleep mode put the device to sleep mode */
    if (last_set_mode != static_cast<uint8_t>(bmp::BMP3_MODE::SLEEP)) {
        /* Device should be put to sleep before transitioning to
         * forced mode or normal mode
         */
        rslt = put_device_to_sleep(dev);

        /* Give some time for device to go into sleep mode */
        dev->delay_us(5000, dev->intf_ptr);
    }

    if (rslt != BMP3_OK) return rslt;

    if (curr_mode == static_cast<uint8_t>(bmp::BMP3_MODE::NORMAL)) {
        /* Set normal mode and validate necessary settings */
        rslt = set_normal_mode(dev);
    } else if (curr_mode == static_cast<uint8_t>(bmp::BMP3_MODE::FORCED)) {
        /* Set forced mode */
        rslt = write_power_mode(dev);
    }

    return rslt;
}



/**
 * @brief This API gets the power mode of the sensor.
 */
int8_t bmp3_get_op_mode(uint8_t *op_mode, struct bmp3_dev *dev) {
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    if ((rslt == BMP3_OK) && (op_mode != NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }
    
    /* Read the power mode register */
    rslt = bmp3_get_regs(static_cast<uint8_t>(bmp::BMP_REGS::POWER_CTRL), op_mode, 1, dev);

    /* Assign the power mode in the device structure */
    *op_mode = ((*op_mode & static_cast<uint8_t>(bmp::BMP3_MASKS::OP_MODE)) >> static_cast<uint8_t>(bmp::BMP3_MASKS::OP_MODE_POS));

    return rslt;
}


/**
 * @brief This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the bmp3_data structure
 * instance passed by the user.
 */
int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data, struct bmp3_dev *dev) {
    int8_t rslt;

    /* Array to store the pressure and temperature data read from
     * the sensor */
    const uint32_t BMP3_LEN_P_T_DATA = static_cast<uint32_t>(bmp::BMP3_LEN::P_T_DATA);
    uint8_t reg_data[BMP3_LEN_P_T_DATA] = { 0 };
    struct bmp3_uncomp_data uncomp_data = { 0 };

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    if ((rslt != BMP3_OK) || (comp_data == NULL)) {
        rslt = static_cast<int8_t>(bmp::BMP3_E::NULL_PTR);
        return rslt;
    }
    
    /* Read the pressure and temperature data from the sensor */
    rslt = bmp3_get_regs(
        static_cast<uint8_t>(bmp::BMP_REGS::DATA),
        reg_data,
        BMP3_LEN_P_T_DATA,
        dev
    );

    if (rslt == BMP3_OK) {
        /* Parse the read data from the sensor */
        parse_sensor_data(reg_data, &uncomp_data);

        /* Compensate the pressure/temperature/both data read
            * from the sensor */
        rslt = compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
    }

    return rslt;
}


