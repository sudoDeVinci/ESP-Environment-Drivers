#include "AUnit.h"

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("Running ESP Environment Drivers Tests...");
}

void loop() {
    aunit::TestRunner::run();
}