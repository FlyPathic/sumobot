#include <stdint.h>
#include <Adafruit_BMP280.h>


namespace sumo::pins
{
    // ====================================================
    //                 Suction Pump
    // ====================================================

    // ----- Pressure sensor chip select pins -----

    constexpr uint8_t BMP_EXTERNAL_CS = 27;
    constexpr uint8_t BMP_INTERNAL_CS = 26;

    // ----- Suction pump impeller esc PWM pin -----

    constexpr uint8_t IMPELLER_PWM = 25;

} // namespace sumo::pins

namespace sumo::config
{
    // ====================================================
    //                 Suction Pump
    // ====================================================

    // ----- Pressure sensor config -----

    constexpr Adafruit_BMP280::sensor_mode       BMP_MODE              = Adafruit_BMP280::sensor_mode::MODE_NORMAL;
    constexpr Adafruit_BMP280::sensor_sampling   BMP_TEMP_SAMPLING     = Adafruit_BMP280::sensor_sampling::SAMPLING_X2;
    constexpr Adafruit_BMP280::sensor_sampling   BMP_PRESSURE_SAMPLING = Adafruit_BMP280::sensor_sampling::SAMPLING_X16;
    constexpr Adafruit_BMP280::sensor_filter     BMP_FILTERING         = Adafruit_BMP280::sensor_filter::FILTER_X16;
    constexpr Adafruit_BMP280::standby_duration  BMP_STANDBY_DURATION  = Adafruit_BMP280::standby_duration::STANDBY_MS_125;


    // ----- ESC control timing -----

    const uint16_t IMPELLER_STOP_THROTTLE = 1000;
    const uint16_t IMPELLER_START_THROTTLE = 1600;
    const uint16_t IMPELLER_MAX_THROTTLE = 2000;
    
    // ----- ESC control config -----

    const uint16_t THROTTLE_RAMP_RATE = 5; // How quickly the throttle reaches the setpoint as % / Second

    constexpr uint32_t IMPELLER_ARM_TIME_MS = 2000;

} // namespace sumo::config
