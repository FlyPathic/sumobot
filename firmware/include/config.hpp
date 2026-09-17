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

    constexpr uint8_t IMPELLER_PWM = 26;

} // namespace sumo::pins

namespace sumo::config
{
    // ====================================================
    //                 Suction Pump
    // ====================================================

    // ----- Pressure sensor config -----

    constexpr Adafruit_BMP280::sensor_mode       BMP_MODE               = Adafruit_BMP280::sensor_mode::MODE_NORMAL;
    constexpr Adafruit_BMP280::sensor_sampling   BMP_TEMP_SAMPLING      = Adafruit_BMP280::sensor_sampling::SAMPLING_X2;
    constexpr Adafruit_BMP280::sensor_sampling   BMP_PRESSURE_SAMPLING  = Adafruit_BMP280::sensor_sampling::SAMPLING_X16;
    constexpr Adafruit_BMP280::sensor_filter     BMP_FILTERING          = Adafruit_BMP280::sensor_filter::FILTER_X16;
    constexpr Adafruit_BMP280::standby_duration  BMP_STANDBY_DURATION   = Adafruit_BMP280::standby_duration::STANDBY_MS_500;

} // namespace sumo::config
