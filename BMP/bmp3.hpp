#pragma once

#include "BMP/bmpdefs.hpp"

/**
 * @details This is the API entry point for initializing the BMP3 sensor.
 * Performs the selection of I2C/SPI read mechanism according to the
 * selected interface and reads the chip-id and calibration data of the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_init(struct bmp3_dev *dev);


/**
 * @details This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_soft_reset(struct bmp3_dev *dev);


 /** 
 * @details This API sets the power control(pressure enable and
 * temperature enable), over sampling, odr and filter
 * settings in the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set in the sensor.
 *
 * @note : Below are the enum values to be used by the user for selecting the
 * desired settings. User can do OR operation of these values for configuring
 * multiple settings.
 *
 * @verbatim
 * Enum values                |   Functionality
 * ---------------------------|----------------------------------------------
 * bmp::BMP3_SEL::PRESS_EN    |   Enable/Disable pressure.
 * bmp::BMP3_SEL::TEMP_EN     |   Enable/Disable temperature.
 * bmp::BMP3_SEL::PRESS_OS    |   Set pressure oversampling.
 * bmp::BMP3_SEL::TEMP_OS     |   Set temperature oversampling.
 * bmp::BMP3_SEL::IIR_FILTER  |   Set IIR filter.
 * bmp::BMP3_SEL::ODR         |   Set ODR.
 * bmp::BMP3_SEL::OUTPUT_MODE |   Set either open drain or push pull
 * bmp::BMP3_SEL::LEVEL       |   Set interrupt pad to be active high or low
 * bmp::BMP3_SEL::LATCH       |   Set interrupt pad to be latched or nonlatched.
 * bmp::BMP3_SEL::DRDY_EN     |   Map/Unmap the drdy interrupt to interrupt pad.
 * bmp::BMP3_SEL::I2C_WDT_EN  |   Enable/Disable I2C internal watch dog.
 * bmp::BMP3_SEL::I2C_WDT     |   Set I2C watch dog timeout delay.
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_set_sensor_settings(uint32_t desired_settings, struct bmp3_dev *dev);


/**
 * @details This API gets the power control(power mode, pressure enable and
 * temperature enable), over sampling, odr, filter, interrupt control and
 * advance settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_sensor_settings(struct bmp3_dev *dev);


/**
 * @details This API gets the power control(power mode, pressure enable and
 * temperature enable), over sampling, odr, filter, interrupt control and
 * advance settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_sensor_settings(struct bmp3_dev *dev);


/**
 * @details This API sets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 *@verbatim
 * ------------------------------------------
 * dev->settings.op_mode |   Values
 * ----------------------|-------------------
 *     0                 | bmp::BMP3_MODE::SLEEP
 *     1                 | bmp::BMP3_MODE::FORCED
 *     3                 | bmp::BMP3_MODE::NORMAL
 * ------------------------------------------
 *@endverbatim
 *
 * @note : Before setting normal mode, valid odr and osr settings should be set
 * in the sensor by using 'bmp3_set_sensor_settings' function.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_set_op_mode(struct bmp3_dev *dev);


/**
 * @details This API gets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[out] op_mode : Pointer variable to store the op-mode.
 *
 *@verbatim
 * ------------------------------------------
 *   op_mode             |   Values
 * ----------------------|-------------------
 *     0                 | bmp::BMP3_MODE::SLEEP
 *     1                 | bmp::BMP3_MODE::FORCED
 *     3                 | bmp::BMP3_MODE::NORMAL
 * ------------------------------------------
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_op_mode(uint8_t *op_mode, struct bmp3_dev *dev);


/**
 * @details This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the bmp3_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 *@verbatim
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BMP3_PRESS
 *     2       | BMP3_TEMP
 *     3       | BMP3_ALL
 *@endverbatim
 *
 * @param[out] data : Structure instance of bmp3_data.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @note : for fixed point the compensated temperature and pressure has a multiplication factor of 100.
 *          units are degree celsius and Pascal respectively.
 *          ie if temp is 2426 then temp is 24.26 deg C
 *          if press is 9528709 it is 95287.09 Pascal.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *data, struct bmp3_dev *dev);


/**
 * @details This API writes the given data to the register address
 * of the sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *                         in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bmp3_dev *dev);


/**
 * @details This API reads the data from the given register address of the sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmp3_dev *dev);


/**
 * @details This API sets the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings in the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[in] desired_settings : Variable used to select the FIFO settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 *@verbatim
 * --------------------------------------|-------------------------------------------
 * Macros                                |  Functionality
 * --------------------------------------|-------------------------------------------
 * bmp::BMP3_SEL_FIFO::MODE              |  Enable/Disable FIFO
 * bmp::BMP3_SEL_FIFO::STOP_ON_FULL_EN   |  Set FIFO stop on full interrupt
 * bmp::BMP3_SEL_FIFO::TIME_EN           |  Enable/Disable FIFO time
 * bmp::BMP3_SEL_FIFO::PRESS_EN          |  Enable/Disable pressure
 * bmp::BMP3_SEL_FIFO::TEMP_EN           |  Enable/Disable temperature
 * bmp::BMP3_SEL_FIFO::DOWN_SAMPLING     |  Set FIFO downsampling
 * bmp::BMP3_SEL_FIFO::FILTER_EN         |  Enable/Disable FIFO filter
 * bmp::BMP3_SEL_FIFO::FWTM_EN           |  Enable/Disable FIFO watermark interrupt
 * bmp::BMP3_SEL_FIFO::FULL_EN           |  Enable/Disable FIFO full interrupt
 * ----------------------------------------------------------------------------------
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_set_fifo_settings(uint16_t desired_settings, struct bmp3_dev *dev);


/**
 * @details This API gets the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_fifo_settings(struct bmp3_dev *dev);


/**
 * @details This API gets the fifo data from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3 device, where the fifo
 * data will be stored in fifo buffer.
 *
 * @return Result of API execution status.
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_fifo_data(struct bmp3_dev *dev);


/**
 * @details This API gets the fifo length from the sensor.
 *
 * @param[out] fifo_length : Variable used to store the fifo length.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status.
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_fifo_length(uint16_t *fifo_length, struct bmp3_dev *dev);


/**
 * @details This API extracts the temperature and/or pressure data from the FIFO
 * data which is already read from the fifo.
 *
 * @param[out] data : Array of bmp3_data structures where the temperature
 * and pressure frames will be stored.
 * @param[in,out] dev : Structure instance of bmp3_dev which contains the
 * fifo buffer to parse the temperature and pressure frames.
 *
 * @return Result of API execution status.
 * @retval 0  -> Success
 * @retval <0 -> Error
 */
int8_t bmp3_extract_fifo_data(struct bmp3_data *data, struct bmp3_dev *dev);


/**
 * @details This API sets the fifo watermark length according to the frames count
 * set by the user in the device structure. Refer below for usage.
 *
 * @note: dev->fifo->data.req_frames = 50;
 *
 * @param[in] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval 0  -> Success
 * @retval <0 -> Error
 */
int8_t bmp3_set_fifo_watermark(struct bmp3_dev *dev);


/**
 * @details This API performs fifo flush
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_fifo_flush(struct bmp3_dev *dev);


/**
 * @details This API gets the command ready, data ready for pressure and
 * temperature and interrupt (fifo watermark, fifo full, data ready) and
 * error status from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval 0  -> Success
 * @retval <0 -> Error
 */
int8_t bmp3_get_status(struct bmp3_dev *dev);













