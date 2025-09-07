#ifdef EPOXY_DUINO

#include <AUnit.h>
#include "../I2CSensor.hpp"
#include <cmath> 

// Test sensor to access protected utility functions
class TestUtilsSensor : public I2CSensor {
protected:
    bool deviceSpecificSetup() override { return true; }
    
public:
    TestUtilsSensor() : I2CSensor(0x48, 0, 21, 22, 100000, 400000) {}
    bool update() override { return this->_is_initialized; }
    
    // Exposing protected methods for testing
    template<typename T, size_t N>
    float testMean(const std::array<T, N>& arr) const { return mean(arr); }
    
    template<typename T>
    float testMean(const std::vector<T>& vec) const { return mean(vec); }
    
    template<typename T, size_t N>
    float testStddev(const std::array<T, N>& arr, float meanVal = NAN) const { 
        return stddev(arr, meanVal); 
    }
    
    template<typename T, size_t N>
    std::array<float, 3> testQuartiles(std::array<T, N>& arr) const { 
        return quartiles(arr); 
    }
    
    uint8_t testCrc8(const uint8_t* data, int len) const { return I2CSensor::crc8(data, len); }
};

test(I2CSensor_Mean_Array) {
    TestUtilsSensor sensor;
    std::array<int, 5> data = {1, 2, 3, 4, 5};
    
    float result = sensor.testMean(data);
    assertEqual(result, 3.0f);
}

test(I2CSensor_Mean_Vector) {
    TestUtilsSensor sensor;
    std::vector<float> data = {2.0f, 4.0f, 6.0f, 8.0f};
    
    float result = sensor.testMean(data);
    assertEqual(result, 5.0f);
}

test(I2CSensor_Stddev) {
    TestUtilsSensor sensor;
    std::array<float, 4> data = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = sensor.testStddev(data, 2.5f);
    assertNear(result, 1.118f, 0.01f); // sqrt(1.25) ≈ 1.118
}

test(I2CSensor_Quartiles) {
    TestUtilsSensor sensor;
    std::array<int, 8> data = {1, 2, 3, 4, 5, 6, 7, 8};
    
    auto result = sensor.testQuartiles(data);
    assertEqual(result[0], 3.0f); // Q1
    assertEqual(result[1], 5.0f); // Median
    assertEqual(result[2], 7.0f); // Q3
}

test(I2CSensor_CRC8) {
    TestUtilsSensor sensor;
    uint8_t data[] = {0x00, 0x00};
    
    uint8_t result = sensor.testCrc8(data, 2);
    assertEqual(result, 0x81); // Known CRC8 result for {0x00, 0x00}
}

#endif
