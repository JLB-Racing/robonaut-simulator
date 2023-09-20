#ifndef VEHICLE_MODEL_HXX
#define VEHICLE_MODEL_HXX

#include <cmath>

namespace rsim
{
    namespace vmodel
    {
        static constexpr double DELTA_T = 0.1;
        static constexpr double MAX_WHEEL_ANGLE = 1.0;
        static constexpr double MAX_VELOCITY = 50.0;
        static constexpr double WHEELBASE = 16.0;

        struct State
        {
            double x;
            double y;
            double orientation;
            double velocity = 0.0;
            double angular_velocity = 0.0;
            double wheel_angle = 0.0;

            State(double x_, double y_, double orientation_)
                : x(x_), y(y_), orientation(orientation_) {}

            void update(double wheel_angle_in, double velocity_in)
            {
                // Limit the input values to their maximums
                wheel_angle = std::max(-MAX_WHEEL_ANGLE, std::min(MAX_WHEEL_ANGLE, wheel_angle_in));
                velocity = std::max(-MAX_VELOCITY, std::min(MAX_VELOCITY, velocity_in));

                // Calculate the new orientation and position
                double delta_s = velocity * DELTA_T;
                double delta_theta = (delta_s / WHEELBASE) * tan(wheel_angle);

                x += delta_s * cos(orientation + delta_theta / 2.0);
                y += delta_s * sin(orientation + delta_theta / 2.0);
                orientation += delta_theta;

                // Ensure the orientation is within the range [0, 2*pi)
                orientation = fmod(orientation, 2.0 * M_PI);

                // Calculate the angular velocity
                angular_velocity = delta_theta / DELTA_T;
            }
        };
    } // namespace vmodel

} // namespace rsim

#endif // VEHICLE_MODEL_HXX