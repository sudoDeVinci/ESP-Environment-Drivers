#pragma once

#include <Wire.h>
#include <vector>
#include <map>

/**
 * AUnit doesn't have RTOS functionality, so we swap for regular arduino functions.
 */
#ifdef EPOXY_DUINO
    #include "Arduino.h"
    #define vTaskDelay(x) delay(x)
    #define portTICK_PERIOD_MS 1
#else
    // For ESP32 - use FreeRTOS
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
#endif

// Forward declaration to avoid circular dependency
class I2CSensor;

/**
 * Structure to hold information about an I2C bus.
 * Contains the state of the bus, pins used, current clock speed,
 * and a list of registered devices.
 */
struct BusInfo {
    bool is_initialized = false;
    int sda_pin = -1;
    int scl_pin = -1;
    uint32_t current_clock = 0;
    std::vector<I2CSensor*> devices;
    TwoWire* wire = nullptr;
};

class I2CManager {
public:

    /**
     * @brief Singleton instance getter for I2CManager.
     * @return Reference to the singleton instance of I2CManager.
     */
    [[nodiscard]]
    static I2CManager& getInstance() {
        static I2CManager instance;
        return instance;
    }

    /**
     * @brief Deleted copy constructor and assignment operator to prevent copying.
     */
    I2CManager(I2CManager const&) = delete;

    /**
     * @brief Deleted assignment operator to prevent assignment.
     */
    void operator=(I2CManager const&) = delete;

    /**
     * @brief Returns the number of devices registered on a specific bus.
     * @param bus_num The bus number to check.
     * @return The number of devices on the bus, or 0 if the bus does
     */
    uint8_t deviceCount(int bus_num) const {
        auto it = _buses.find(bus_num);
        if (it != _buses.end()) {
            return it->second.devices.size();
        }
        return 0;
    }

    /**
     * @brief Registers a sensor with the manager.
     * @param sensor Ref to the sensor instance.
     * @return True if registration is successful, false otherwise.
     */
    [[nodiscard]]
    bool registerSensor(I2CSensor& sensor);

    /**
     * @brief Unregisters a sensor from the manager.
     * @param sensor Ref to the sensor instance.
     * @return True if unregistration is successful, false otherwise.
     */
    [[nodiscard]]
    bool unregisterSensor(I2CSensor& sensor);

    /**
     * @brief Clears all registered sensors and resets the bus information.
     */
    void clear();


private:
    std::map<int, BusInfo> _buses;
    
    I2CManager() {
        #ifdef EPOXY_DUINO
            // EpoxyDuino uses default TwoWire constructor
            static TwoWire w0;
            static TwoWire w1;
        #else
            // ESP32 uses TwoWire(bus_number)
            static TwoWire w0(0);
            static TwoWire w1(1);
        #endif
        
        _buses[0].wire = &w0;
        _buses[1].wire = &w1;
    }
};
