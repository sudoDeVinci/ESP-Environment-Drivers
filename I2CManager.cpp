#include "I2CManager.hpp"
#include "I2CSensor.hpp"
#include <algorithm>

/**
 * @brief Registers a sensor with the manager.
 * @param sensor Ref to the sensor instance.
 * @return True if registration is successful, false otherwise.
 */
[[nodiscard]]
bool I2CManager::registerSensor(I2CSensor& sensor) {
    uint8_t bus_num = sensor.getBusNum();
    if (_buses.find(bus_num) == _buses.end()) {
        // debug serial print: "Bus not found"
        return false;
    }

    BusInfo& bus_info = _buses[bus_num];

    // Check if the bus is initialized - if not, initialize it
    if (!bus_info.is_initialized) {
        bus_info.sda_pin = sensor.getSdaPin();
        bus_info.scl_pin = sensor.getSclPin();
        
        // TwoWire templates are different across AVR and ESP32 lib
        #ifdef EPOXY_DUINO
            // EpoxyDuino's TwoWire doesn't take pin parameters
            bus_info.wire->begin();
        #else
            // ESP32's TwoWire takes SDA/SCL pins
            bus_info.wire->begin(bus_info.sda_pin, bus_info.scl_pin);
        #endif
        
        bus_info.is_initialized = true;
    }

    // Check if the sensor is on the right pins being used by the bus
    if (bus_info.sda_pin != sensor.getSdaPin() || 
        bus_info.scl_pin != sensor.getSclPin()) {
        return false; // Pin mismatch
    }

    /**
     * Check if the sensor address conflicts with any existing sensors
     * on the bus - if so, return false.
     */
    if (std::any_of(
            bus_info.devices.cbegin(),
            bus_info.devices.cend(),
            [&sensor](const I2CSensor* existing_sensor) {
                return existing_sensor->getAddress() == sensor.getAddress();
            }
        )
    ) {
        // debug serial print: "Sensor address conflict"
        return false;
    }
    


    // Negatiating Clock speeds for sensor along selected bus.
    uint32_t new_potential_clock = sensor.getMaxClock();

    /**
     * If the sensor's potential max clock speed is higher than the bus's current highest,
     * Use the bus's current highest clock speed as the new potential max.
     */
    for (const I2CSensor* existing_sensor : bus_info.devices) {
        if (new_potential_clock > existing_sensor->getMaxClock()) {
            new_potential_clock = existing_sensor->getMaxClock();
    }
    
    }


    /**
     * Check if this new speed is too low for any existing sensors on the bus.
     */
    if (std::any_of(
        bus_info.devices.cbegin(),
        bus_info.devices.cend(),
        [&new_potential_clock](const I2CSensor* existing_sensor) {
        return new_potential_clock < existing_sensor->getMinClock();
        }
    )){
        // debug serial print: "Sensor address conflict"
        return false;
    }
    
    /**
     * Check if the new potential clock speed is within the sensor's limits.
     */
    if (new_potential_clock < sensor.getMinClock()) {
        // debug serial print: "New clock speed out of sensor's limits"
        return false;
    }

    /**
     * Everything checks out so now we add the sensor to the bus.
     * 1. set the clock on the wire.
     * 2. set the clock on the bus info.
     * 3. Add the sensor to the bus's device list.
     * 4. Set the wire on the sensor.
     */

    // No need to set clock in testing.
    #ifndef EPOXY_DUINO
        bus_info.wire->setClock(new_potential_clock);
    #endif
    bus_info.current_clock = new_potential_clock;
    bus_info.devices.push_back(&sensor);
    sensor.setWire(bus_info.wire);

    return true;
}


/**
 * @brief Unregisters a sensor from the manager.
 * @param sensor Ref to the sensor instance.
 * @return True if unregistration is successful, false otherwise.
 */
[[nodiscard]]
bool I2CManager::unregisterSensor(I2CSensor& sensor) {
    
    uint8_t bus_num = sensor.getBusNum();
    auto busit = _buses.find(bus_num);

    if (busit == _buses.end()) {
        // debug serial print: "Bus not found"
        return false;
    }

    BusInfo& bus_info = busit->second;

    auto it = std::find(
        bus_info.devices.begin(),
        bus_info.devices.end(),
        &sensor
    );

    if (it == bus_info.devices.end()) {
        // debug serial print: "Sensor not found on the bus"
        return false;
    }

    bus_info.devices.erase(it);

    // If no devices left, reset the bus info
    if (bus_info.devices.empty()) {
        bus_info.is_initialized = false;
        bus_info.sda_pin = -1;
        bus_info.scl_pin = -1;
        bus_info.current_clock = 0;

        #ifndef EPOXY_DUINO
        bus_info.wire->end();
        #endif
    }
    return true;
}

/**
 * @brief Clears all registered sensors and resets the bus information.
 */
void I2CManager::clear() {
    for (auto& iter : _buses) {
        BusInfo& bus = iter.second;
        for (auto& sensor : bus.devices) {
            sensor->setWire(nullptr);
            sensor->setInitialized(false);
        }

        bus.devices.clear();
        bus.is_initialized = false;
        bus.sda_pin = -1;
        bus.scl_pin = -1;
        bus.current_clock = 0;

        #ifndef EPOXY_DUINO
        bus.wire->end();
        #endif
    }
}