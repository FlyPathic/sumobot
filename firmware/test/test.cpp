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
}

void loop()
{

}