#pragma once
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>

#include "config.hpp"

namespace sumo
{
    class Pump
    {
    public:
        Pump() = default;

        /// @brief Initializes the pump; initializes and configures the BMP sensors and the ESC.
        void begin()
        {
            // ===============================================
            //             Sensor Initialization
            // ===============================================

            // ----- External sensor -----
            unsigned status;

            status = bmp_external.begin();

            if (!status)
            {
                Serial.println("[Pump] Could not connect to BMP external.");
            }
            else
            {
                external_bmp_ready = true;
                bmp_external.setSampling(config::BMP_MODE,
                                         config::BMP_TEMP_SAMPLING,
                                         config::BMP_PRESSURE_SAMPLING,
                                         config::BMP_FILTERING,
                                         config::BMP_STANDBY_DURATION);
            }

            // ----- Internal Sensor -----

            status = bmp_internal.begin();

            if (!status)
            {
                Serial.println("[Pump] Could not connect to BMP internal.");
            }
            else
            {
                internal_bmp_ready = true;
                bmp_internal.setSampling(config::BMP_MODE,
                                         config::BMP_TEMP_SAMPLING,
                                         config::BMP_PRESSURE_SAMPLING,
                                         config::BMP_FILTERING,
                                         config::BMP_STANDBY_DURATION);
            }

            float ext_pressure = bmp_external.readPressure();
            float int_pressure = bmp_internal.readPressure();

            internal_pressure_offset = ext_pressure - int_pressure;

            Serial.print("[Pump] Normalizing pressure to: ");
            Serial.print(ext_pressure);
            Serial.print(" Pa (");
            Serial.print(internal_pressure_offset);
            Serial.println(" offset) Pa");

            // ===============================================
            //                  ESC setup
            // ===============================================

            esc.attach(
                pins::IMPELLER_PWM,
                config::IMPELLER_STOP_THROTTLE,
                config::IMPELLER_MAX_THROTTLE);

            // Begin ESC arming sequence.
            esc_armed = false;
            esc_arm_start_us = esp_timer_get_time();

            // First write must be zero throttle.
            esc_current = 0.0F;
            esc.writeMicroseconds(config::IMPELLER_STOP_THROTTLE);
        }

        /// @brief Force stops the motor. Does not restart the ESC arming sequence.
        void stop()
        {
            esc_setpoint = 0.0F;
            esc_current = 0.0F;
            last_update_us = 0;

            esc.writeMicroseconds(config::IMPELLER_STOP_THROTTLE);
        }

        /// @brief Sets the target throttle.
        /// @param throttle The target throttle value.
        /// @note Actual throttle remains at 0 until the ESC arming delay has elapsed.
        void set_throttle(const float throttle)
        {
            esc_setpoint = constrain(throttle, 0.0F, 1.0F);
        }

        /// @brief Gets the actual throttle sent to the ESC.
        /// @return Actual throttle.
        float get_actual_throttle() const
        {
            return esc_current;
        }

        /// @brief Gets the actual throttle's pwm frequency.
        /// @return Frequency in microseconds
        uint16_t get_us() const
        {
            return throttle_to_us(esc_current);
        }

        /// @brief Gets the pressure differential between the internal and external sensor.
        /// @return Pressure differential in pascals.
        float get_pressure_differential()
        {
            float ext_pressure = bmp_external.readPressure();
            float int_pressure = bmp_internal.readPressure() + internal_pressure_offset;

            return ext_pressure - int_pressure;
        }

        /// @brief Handles internal state keeping; moves actual throttle towards the setpoint.
        void update()
        {
            const int64_t now = esp_timer_get_time();

            // ===============================================
            //                  ESC arming
            // ===============================================

            if (!esc_armed)
            {
                const int64_t arm_time_us =
                    static_cast<int64_t>(config::IMPELLER_ARM_TIME_MS) * 1000LL;

                // Keep writing zero throttle while the ESC arms.
                if ((now - esc_arm_start_us) < arm_time_us)
                {
                    esc_current = 0.0F;
                    last_update_us = now;
                    esc.writeMicroseconds(config::IMPELLER_STOP_THROTTLE);
                    return;
                }

                // Arming period has elapsed.
                esc_armed = true;

                // Reset the ramp timer so the first post-arm update
                // cannot get a large delta from the arming period.
                last_update_us = now;

                esc_current = 0.0F;
                esc.writeMicroseconds(config::IMPELLER_STOP_THROTTLE);
                return;
            }

            // ===============================================
            //                  Normal operation
            // ===============================================

            // First update after arming.
            if (last_update_us == 0)
            {
                last_update_us = now;
                return;
            }

            const int64_t dt_us = now - last_update_us;
            last_update_us = now;

            const float ramp_per_second =
                config::THROTTLE_RAMP_RATE / 100.0F;

            const float max_delta =
                ramp_per_second *
                (static_cast<float>(dt_us) / 1000000.0F);

            // Ramp toward the requested setpoint.
            if (esc_current < esc_setpoint)
            {
                esc_current =
                    min(esc_current + max_delta, esc_setpoint);
            }
            else if (esc_current > esc_setpoint)
            {
                esc_current =
                    max(esc_current - max_delta, esc_setpoint);
            }

            write_esc(esc_current);
        }

    private:
        int64_t last_update_us = 0;
        int64_t esc_arm_start_us = 0;

        Adafruit_BMP280 bmp_external =
            Adafruit_BMP280(pins::BMP_EXTERNAL_CS);

        Adafruit_BMP280 bmp_internal =
            Adafruit_BMP280(pins::BMP_INTERNAL_CS);

        bool external_bmp_ready = false;
        bool internal_bmp_ready = false;

        float internal_pressure_offset = 0.0F;

        Servo esc;

        float esc_setpoint = 0.0F;
        float esc_current = 0.0F;

        bool esc_armed = false;

    private:
        uint16_t throttle_to_us(const float throttle) const
        {
            if (throttle <= 0.0F)
                return config::IMPELLER_STOP_THROTTLE;

            return static_cast<uint16_t>(
                config::IMPELLER_START_THROTTLE +
                throttle *
                    (config::IMPELLER_MAX_THROTTLE -
                     config::IMPELLER_START_THROTTLE));
        }

        void write_esc(float throttle)
        {
            esc.writeMicroseconds(throttle_to_us(throttle));
        }
    };
}
