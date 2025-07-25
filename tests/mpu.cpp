#include <AUnit.h>
#include "../MPU6050.hpp"

test(MPU6050_Construction) {
    MPU6050 mpu(0, 21, 22);
    
    assertEqual(mpu.getAddress(), MPU6050_ADDR);
    assertEqual(mpu.getBusNum(), 0);
    assertEqual(mpu.getSdaPin(), 21);
    assertEqual(mpu.getSclPin(), 22);
    assertFalse(mpu.isInitialized());
}

test(MPU6050_EnumValues) {
    // Test DLPF_CFG enum values
    assertEqual(static_cast<uint8_t>(DLPF_256HZ), 0x00);
    assertEqual(static_cast<uint8_t>(DLPF_188HZ), 0x01);
    assertEqual(static_cast<uint8_t>(DLPF_5HZ), 0x06);
    
    // Test LSB_SENSITIVITY enum values
    assertEqual(static_cast<uint8_t>(LSB_131P0), 0x00);
    assertEqual(static_cast<uint8_t>(LSB_65P5), 0x08);
    assertEqual(static_cast<uint8_t>(LSB_16P4), 0x18);
}

test(MPU6050_LSBMapping) {
    // Test that LSB_MAP contains correct sensitivity values
    assertEqual(LSB_MAP.at(LSB_131P0), 131.0f);
    assertEqual(LSB_MAP.at(LSB_65P5), 65.5f);
    assertEqual(LSB_MAP.at(LSB_32P8), 32.8f);
    assertEqual(LSB_MAP.at(LSB_16P4), 16.4f);
}

test(MPU6050_ReadGyro_NotInitialized) {
    MPU6050 mpu(0, 21, 22);
    
    // Reading gyro on uninitialized sensor should return zeros
    MPU_XYZ reading = mpu.readGyro();
    assertEqual(reading[0], 0.0f);
    assertEqual(reading[1], 0.0f);
    assertEqual(reading[2], 0.0f);
}