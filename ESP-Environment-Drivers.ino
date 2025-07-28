#include "MPU6050.hpp"
#include "SHT31D.hpp"
#include "BMP3xx.hpp"


#define I2C_BUS_0_SDA 21
#define I2C_BUS_0_SCL 22

// Instances of your sensors would usually live in a global scope
MPU_XYZ gyroreading;
MPU6050 mpu(0, I2C_BUS_0_SDA, I2C_BUS_0_SCL);

void setup() {
    Serial.begin(115200);
    
    if (!mpu.init()) {
        Serial.println("Failed to initialize MPU6050!");
        while (1) { delay(1000); }
    } else {
        Serial.println("MPU6050 initialized successfully.");
    }
}

void loop() {
    gyroreading = mpu.readGyro();
    Serial.printf("Gyro X: %.2f, Y: %.2f, Z: %.2f\n", gyroreading[0], gyroreading[1], gyroreading[2]);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
