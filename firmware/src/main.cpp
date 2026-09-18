#include <Arduino.h>
#include "pump.hpp"

using namespace sumo;

Pump pump;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Initializing pump...");

    pump.begin();

    Serial.println("Pump initialized.");

    pump.set_throttle(1.0F);
}

void loop()
{
    delay(100);
    Serial.print("Pump throttle: ");
    Serial.print(pump.get_actual_throttle());
    Serial.print(", Pressure differential: ");
    Serial.print(pump.get_pressure_differential());
    Serial.println(" Pa");
    pump.update();
}