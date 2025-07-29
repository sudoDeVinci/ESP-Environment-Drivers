# Thread-Safe I2C Sensor Drivers for ESP32

[![Compile](https://github.com/sudoDeVinci/ESP-Environment-Drivers/actions/workflows/compile.yml/badge.svg?branch=main)](https://github.com/sudoDeVinci/ESP-Environment-Drivers/actions/workflows/compile.yml)
[![Unit Tests](https://github.com/sudoDeVinci/ESP-Environment-Drivers/actions/workflows/testing.yml/badge.svg?branch=main)](https://github.com/sudoDeVinci/ESP-Environment-Drivers/actions/workflows/testing.yml)
![Doxygen](/assets/doxygen_badge.svg)


While creating the [ESP-Sky-imager]("(https://github.com/sudoDeVinci/ESP-Sky-Imager") and [Esp-QuadCopter]("(https://github.com/sudoDeVinci/ESP-QuadCopter") projects, I ended up needing to also create a thread-safe alternative to the Adafruit BusIO drivers for my various sensors. 
This repository is a collection of those drivers and any updates I make to them.

## Features

- **Thread-safe I2C communication** using timed mutexes
- **Automatic I2C bus management** with clock speed negotiation
- **Address conflict detection** and pin validation
- **Statistical processing utilities** for sensor data filtering
- **Comprehensive unit testing** with AUnit framework
- **ESP platform focus** The only target board type

## Supported Sensors

- **MPU6050** - 6-axis gyroscope and accelerometer
- **SHT31D** - Temperature and humidity sensor
- **BMP3xx** - Barometric pressure sensor *COMING SOON*

## Architecture

The library is built around two core components:

- [`I2CManager`](I2CManager.hpp) - Singleton that manages I2C buses, handles sensor registration, and negotiates clock speeds
- [`I2CSensor`](I2CSensor.hpp) - Base class providing common I2C functionality and statistical utilities

### I2C Bus Management

The manager automatically handles:
- Clock speed negotiation between multiple sensors on the same bus
- Address conflict detection
- Pin configuration validation
- Bus initialization and cleanup

### Statistical Processing

The base sensor class includes utilities for:
- Mean and standard deviation calculations
- Quartile computation and outlier removal using IQR method
- CRC8 checksum validation

## Usage

```cpp
#include "MPU6050.hpp"
#include "SHT31D.hpp"

#define I2C_SDA 21
#define I2C_SCL 22

MPU6050 gyro(0, I2C_SDA, I2C_SCL);
SHT31 tempSensor(0, I2C_SDA, I2C_SCL);

void setup() {
    Serial.begin(115200);
    
    if (!gyro.init()) {
        Serial.println("MPU6050 initialization failed");
        return;
    }
    
    if (!tempSensor.init()) {
        Serial.println("SHT31D initialization failed");
        return;
    }
}

void loop() {
    MPU_XYZ gyroData = gyro.readGyro();
    
    if (tempSensor.update()) {
        float temperature = tempSensor.getTemperature();
        float humidity = tempSensor.getHumidity();
    }
    
    delay(100);
}
```

## Thread Safety

All I2C communication is protected by timed mutexes with configurable timeouts. The library ensures safe concurrent access to I2C buses when used in multi-threaded environments.

## Building and Testing

The project includes automated testing using the AUnit framework and EpoxyDuino for cross-platform compatibility.

### Running Tests

```bash
cd tests
make EPOXY_DUINO_DIR=../EpoxyDuino AUNIT_DIR=../AUnit
./tests.out
```

### Dependencies for Testing

- [AUnit](https://github.com/bxparks/AUnit) - Unit testing framework
- [EpoxyDuino](https://github.com/bxparks/EpoxyDuino) - Arduino emulation for native testing

## License

This project is licensed under the Business Source License. See [LICENSE](LICENSE) for details.
