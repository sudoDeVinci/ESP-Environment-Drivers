#ifdef EPOXY_DUINO

#include <AUnit.h>
#include "../I2CManager.hpp"
#include "../I2CSensor.hpp"
#include <cstdint>

// Mock sensor class for testing
class MockSensor : public I2CSensor {
private:
    bool setup_success;
    
protected:
    bool deviceSpecificSetup() override {
        return setup_success;
    }
    
public:
    MockSensor(uint8_t addr, uint8_t bus, uint8_t sda, uint8_t scl, 
               uint32_t min_clk, uint32_t max_clk, bool setup_ok = true)
        : I2CSensor(addr, bus, sda, scl, min_clk, max_clk), setup_success(setup_ok) {}
    
        bool update() override { return this->_is_initialized; }
};

test(I2CManager_Singleton) {
    I2CManager& manager1 = I2CManager::getInstance();
    I2CManager& manager2 = I2CManager::getInstance();
    
    // Both references should point to the same instance
    assertEqual(&manager1, &manager2);
}

test(I2CManager_RegisterSensor_Success) {

    MockSensor sensor(0x48, 0, 21, 22, 100000, 400000);
    I2CManager& manager = I2CManager::getInstance();
    manager.clear();

    assertTrue(sensor.init());
    assertTrue(sensor.isInitialized());
}

test(I2CManager_RegisterSensor_AddressConflict) {
    I2CManager& manager = I2CManager::getInstance();
    manager.clear();

    MockSensor sensor1(0x48, 0, 21, 22, 100000, 400000);
    MockSensor sensor2(0x48, 0, 21, 22, 100000, 400000);

    assertTrue(sensor1.init());
    assertFalse(sensor2.init()); // Should fail due to address conflict
}

test(I2CManager_RegisterSensor_PinMismatch) {
    I2CManager& manager = I2CManager::getInstance();
    manager.clear();

    MockSensor sensor1(0x48, 0, 21, 22, 100000, 400000);
    MockSensor sensor2(0x49, 0, 19, 20, 100000, 400000);
    
    assertTrue(manager.registerSensor(sensor1));
    assertFalse(manager.registerSensor(sensor2)); // Should fail due to pin mismatch
}

test(I2CManager_ClockNegotiation) {
    I2CManager& manager = I2CManager::getInstance();
    manager.clear();

    MockSensor fastSensor(0x48, 1, 21, 22, 100000, 1000000);   // 1MHz max
    MockSensor slowSensor(0x49, 1, 21, 22, 50000, 400000);     // 400kHz max
    
    assertTrue(manager.registerSensor(fastSensor));
    assertTrue(manager.registerSensor(slowSensor));

    // Clock should be negotiated to the slower sensor's max (400kHz)
}

test(I2CManager_ClockTooSlow) {
    I2CManager& manager = I2CManager::getInstance();
    manager.clear();

    MockSensor fastSensor(0x48, 1, 19, 20, 500000, 1000000);   // Needs at least 500kHz
    MockSensor slowSensor(0x49, 1, 19, 20, 50000, 300000);     // Max 300kHz
    
    assertTrue(manager.registerSensor(slowSensor));
    assertFalse(manager.registerSensor(fastSensor)); // Should fail - clock too slow
}

#endif