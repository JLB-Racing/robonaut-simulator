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
                return angular_velocity + yaw_rate_distribution(generator);
            }

            float noisy_motor_rpm()
            {
                return motor_rpm + motor_rpm_distribution(generator);
            }

        private:
            std::default_random_engine generator;
            std::normal_distribution<float> motor_rpm_distribution{0.0f, MOTOR_RPM_NOISE};
            std::normal_distribution<float> yaw_rate_distribution{0.0f, YAW_RATE_NOISE};
        };

        class DynamicState : public State
        {
        public:
            DynamicState(float x_, float y_, float orientation_)
                : State(x_, y_, orientation_)
            {
            }

            void update(const float steering_angle, const float throttle, const float dt)
            {
                // Limit the input values to their maximums
                float normalized_steering_angle = std::max(-MAX_WHEEL_ANGLE, std::min(MAX_WHEEL_ANGLE, steering_angle));
                float normalized_throttle = std::max(-MAX_VELOCITY, std::min(MAX_VELOCITY, throttle));

                // Calculate slip angles
                float slip_angle_front = std::atan2(velocity * std::sin(normalized_steering_angle), velocity * std::cos(normalized_steering_angle) + EPSILON);
                float slip_angle_rear = std::atan2(velocity * std::sin(normalized_steering_angle), velocity * std::cos(normalized_steering_angle) + L);

                // Calculate lateral forces at front and rear tires
                float Fyf = -Cf * slip_angle_front;
                float Fyr = -Cr * slip_angle_rear;

                // Calculate longitudinal force at rear tires
                float Fx = Cm * normalized_throttle - Cr0 - Cr2 * velocity * velocity;

                // Calculate angular acceleration
                float angular_acceleration = (L * Fyf * std::cos(normalized_steering_angle) - L * Fyr) / Iz;

                // Calculate linear acceleration
                float acceleration = (Fyf * std::sin(normalized_steering_angle) + Fx) / mass;

                // Update state variables
                velocity += acceleration * dt;
                orientation += (velocity * std::tan(normalized_steering_angle)) / L * dt;
                x += velocity * std::cos(orientation) * dt;

                y += velocity * std::sin(orientation) * dt;
                angular_velocity += angular_acceleration * dt;

                // Ensure the orientation is within the range [0, 2*pi)
                orientation = std::fmod(orientation, 2.0f * M_PI);

                // Calculate wheel RPM and motor RPM
                wheel_rpm = (velocity / (M_PI * WHEEL_DIAMETER)) * 60.0f;
                motor_rpm = wheel_rpm / GEAR_RATIO_MOTOR_TO_WHEEL;
            }

        private:
            // Constants for dynamic bicycle model
            const float mass = 1000.0f; // Vehicle mass
            const float Iz = 2500.0f;   // Moment of inertia about the vehicle's vertical axis
            const float L = 2.5f;       // Wheelbase
            const float Cf = 16000.0f;  // Front tire cornering stiffness
            const float Cr = 18000.0f;  // Rear tire cornering stiffness
            const float Cm = 1500.0f;   // Motor torque constant
            const float Cr0 = 2000.0f;  // Rolling resistance constant
            const float Cr2 = 0.2f;     // Rolling resistance speed-squared term
            const float EPSILON = 1e-6; // Small constant to prevent division by zero
        };
    } // namespace vmodel

} // namespace rsim

#endif // VEHICLE_MODEL_HXX