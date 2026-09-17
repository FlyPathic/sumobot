#pragma once
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include "config.hpp"

namespace sumo
{
    class Pump
    {
        Pump() = default;

        void begin()
        {
            unsigned status;
            status = bmp_external.begin();

            if (!status)
            {
                Serial.println("Could not connect to BMP external.");
                while (1)
                    delay(10);
            }

            status = bmp_internal.begin();
            if (!status)
            {
                Serial.println("Could not connect to BMP internal.");
                while (1)
                    delay(10);
            }

            bmp_external.setSampling(config::BMP_MODE,
                                     config::BMP_TEMP_SAMPLING,
                                     config::BMP_PRESSURE_SAMPLING,
                                     config::BMP_FILTERING,
                                     config::BMP_STANDBY_DURATION);

            bmp_internal.setSampling(config::BMP_MODE,
                                     config::BMP_TEMP_SAMPLING,
                                     config::BMP_PRESSURE_SAMPLING,
                                     config::BMP_FILTERING,
                                     config::BMP_STANDBY_DURATION);
        }

    private:

        Adafruit_BMP280 bmp_external = Adafruit_BMP280(pins::BMP_EXTERNAL_CS);
        Adafruit_BMP280 bmp_internal = Adafruit_BMP280(pins::BMP_INTERNAL_CS);
        
    };
} // namespace sumo
