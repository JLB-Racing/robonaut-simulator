#ifndef VEHICLE_MODEL_HXX
#define VEHICLE_MODEL_HXX

#include <cmath>
#include <random>

namespace rsim
{
    namespace vmodel
    {
        class State
        {
        public:
            float x = 0.0f;
            float y = 0.0f;
            float orientation = 0.0f;
            float velocity = 0.0f;
            float angular_velocity = 0.0f;
            float wheel_angle = 0.0f;
            float wheel_rpm = 0.0f;
            float motor_rpm = 0.0f;

            State(float x_, float y_, float orientation_)
                : x(x_), y(y_), orientation(orientation_)
            {
            }

            void update(const float wheel_angle_in, const float velocity_in, const float dt)
            {
                // Limit the input values to their maximums
                wheel_angle = std::max(-MAX_WHEEL_ANGLE, std::min(MAX_WHEEL_ANGLE, wheel_angle_in));
                velocity = std::max(-MAX_VELOCITY, std::min(MAX_VELOCITY, velocity_in));

                // Calculate the new orientation and position
                float delta_s = velocity * dt;
                float delta_theta = (delta_s / WHEELBASE) * tan(wheel_angle);

                x += delta_s * std::cos(orientation + delta_theta / 2.0f);
                y += delta_s * std::sin(orientation + delta_theta / 2.0f);
                orientation += delta_theta;

                // Ensure the orientation is within the range [0, 2*pi)
                orientation = std::fmod(orientation, 2.0f * M_PI);

                // Calculate the angular velocity
                angular_velocity = delta_theta / dt;

                wheel_rpm = (velocity / (M_PI * WHEEL_DIAMETER)) * 60.0f;
                motor_rpm = wheel_rpm / GEAR_RATIO_MOTOR_TO_WHEEL;
            }

            float noisy_yaw_rate()
            {
                std::normal_distribution<double> yaw_rate_distribution{angular_velocity, YAW_RATE_NOISE};
                return yaw_rate_distribution(generator);
            }

            float noisy_motor_rpm()
            {
                std::normal_distribution<double> motor_rpm_distribution{motor_rpm, MOTOR_RPM_NOISE};
                return motor_rpm_distribution(generator);
            }

        private:
            std::default_random_engine generator;
        };
    } // namespace vmodel

} // namespace rsim

#endif // VEHICLE_MODEL_HXX