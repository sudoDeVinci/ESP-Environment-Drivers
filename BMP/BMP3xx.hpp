#pragma once

#include "bmpdefs.hpp"
#include "I2CSensor.hpp"
#include <Wire.h>
#include <array>

struct BMP3XX : public I2CSensor {

    protected:
        float temperature;
        float pressure
};



