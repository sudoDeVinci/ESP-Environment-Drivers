#ifdef EPOXY_DUINO

#include <AUnit.h>
#include "../I2CManager.hpp"
#include "../MPU6050/MPU6050.hpp"

test(MPU6050_Construction) {
    MPU6050 sensor(0, 21, 22);
    
    assertEqual(sensor.getAddress(), MPU6050_ADDR);
    assertEqual(sensor.getBusNum(), 0);
    assertEqual(sensor.getSdaPin(), 21);
    assertEqual(sensor.getSclPin(), 22);
    assertFalse(sensor.isInitialized());
}

test(MPU6050_EnumValues) {
    // Test DLPF_CFG enum values
    assertEqual(static_cast<uint8_t>(mpu::DLPF_CFG::DLPF_256HZ), 0x00);
    assertEqual(static_cast<uint8_t>(mpu::DLPF_CFG::DLPF_188HZ), 0x01);
    assertEqual(static_cast<uint8_t>(mpu::DLPF_CFG::DLPF_5HZ), 0x06);
    
    // Test LSB_SENSITIVITY enum values
    assertEqual(static_cast<uint8_t>(mpu::LSB_SENSITIVITY::LSB_131P0), 0x00);
    assertEqual(static_cast<uint8_t>(mpu::LSB_SENSITIVITY::LSB_65P5), 0x08);
    assertEqual(static_cast<uint8_t>(mpu::LSB_SENSITIVITY::LSB_16P4), 0x18);
}

test(MPU6050_LSBMapping) {
    // Test that LSB_MAP contains correct sensitivity values
    assertEqual(MPU6050::LSB_MAP.at(mpu::LSB_SENSITIVITY::LSB_131P0), 131.0f);
    assertEqual(MPU6050::LSB_MAP.at(mpu::LSB_SENSITIVITY::LSB_65P5), 65.5f);
    assertEqual(MPU6050::LSB_MAP.at(mpu::LSB_SENSITIVITY::LSB_32P8), 32.8f);
    assertEqual(MPU6050::LSB_MAP.at(mpu::LSB_SENSITIVITY::LSB_16P4), 16.4f);
}

test(MPU6050_ReadGyro_NotInitialized) {
    I2CManager& manager = I2CManager::getInstance();
    manager.clear();
    
    MPU6050 mpusensor(0, 21, 22);
    
    // Reading gyro on uninitialized sensor should return zeros
    MPU_XYZ reading = mpusensor.readGyro();
    assertEqual(reading[0], 0.0f);
    assertEqual(reading[1], 0.0f);
    assertEqual(reading[2], 0.0f);
}

#endif
