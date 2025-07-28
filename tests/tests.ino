#ifdef EPOXY_DUINO
    #include "AUnit.h"
#endif

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("Running ESP Environment Drivers Tests...");
}


#ifdef EPOXY_DUINO
    void loop() {aunit::TestRunner::run();}
#else
    void loop () {}
#endif