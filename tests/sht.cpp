#ifdef EPOXY_DUINO

#include <AUnit.h>
#include "../SHT3X/SHT31D.hpp"

test(SHT31D_Construction) {
    SHT31 sensor(0, 21, 22);
    
    assertEqual(sensor.getAddress(), SHT31_ADDRESS);
    assertEqual(sensor.getBusNum(), 0);
    assertEqual(sensor.getSdaPin(), 21);
    assertEqual(sensor.getSclPin(), 22);
    assertFalse(sensor.isInitialized());
}

test(SHT31D_InitialReadings) {
    SHT31 sensor(0, 21, 22);
    
    // Initial readings should be NaN
    assertTrue(isnan(sensor.getHumidity()));
    assertTrue(isnan(sensor.getTemperature()));
}

test(SHT31D_EnumValues) {
    // Test command enum values
    assertEqual(static_cast<uint16_t>(READSTATUS), 0xF32D);
    assertEqual(static_cast<uint16_t>(CLEARSTATUS), 0x3041);
    assertEqual(static_cast<uint16_t>(SOFTRESET), 0x30A2);
    assertEqual(static_cast<uint16_t>(HEATER_ON), 0x306D);
    assertEqual(static_cast<uint16_t>(HEATER_OFF), 0x3066);
    
    // Test measurement mode enum values
    assertEqual(static_cast<uint16_t>(LOWREP), 0x2416);
    assertEqual(static_cast<uint16_t>(MEDREP), 0x2C0B);
    assertEqual(static_cast<uint16_t>(HIGHREP), 0x2400);
}

#endif