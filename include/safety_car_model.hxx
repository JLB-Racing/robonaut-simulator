#ifndef SAFETY_CAR_MODEL_HXX
#define SAFETY_CAR_MODEL_HXX

#include <string>
#include <vector>
#include <tuple>
#include <algorithm>

#include "utility.hxx"

namespace rsim
{
    namespace scmodel
    {
        class SafetyCarController
        {
        public:
            unsigned long selected     = 0;
            float         target_angle = 0.0f;
            float         target_speed = 0.0f;
            bool          started      = false;

            SafetyCarController() {}

            ~SafetyCarController() {}

            float PID(const float error, const float dt)
            {
                float proportional_term = Kp * error;
                integral += Ki * error * dt;
                float derivative_term = Kd * (error - prev_error) / dt;
                return proportional_term + integral + derivative_term;
            }

            template <size_t cols>
            void lateral_control(bool (&detection_)[cols])
            {
                if (std::all_of(std::begin(detection_), std::end(detection_), [](bool b) { return b; })) { return; }

                auto  control_timestamp_ = std::chrono::steady_clock::now();
                float dt = std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
                prev_control_timestamp_ = control_timestamp_;

                unsigned long sensor_center = cols / 2;

                unsigned long rightmost = 0;
                for (unsigned long i = 0; i < cols; i++)
                    if (!detection_[i] && i > rightmost) rightmost = i;

                unsigned long leftmost = cols;
                for (unsigned long i = 0; i < cols; i++)
                    if (!detection_[i] && i < leftmost) leftmost = i;

                unsigned long center = leftmost;
                for (unsigned long i = leftmost; i <= rightmost; i++)
                    if (!detection_[i] &&
                        std::abs(static_cast<int>(i - (rightmost + leftmost) / 2)) < std::abs(static_cast<int>(center - (rightmost + leftmost) / 2)))
                        center = i;

                selected = center;

                float error  = (static_cast<int>(selected - sensor_center)) / static_cast<float>(sensor_center);
                target_angle = PID(error, dt);

                prev_error = error;
            }

            void longitudinal_control() { target_speed = SPEED; }

            template <size_t cols>
            void update(bool (&detection_)[cols])
            {
                if (started)
                {
                    lateral_control(detection_);
                    longitudinal_control();
                }
                else
                {
                    target_angle = 0.0f;
                    target_speed = 0.0f;
                }
            }

            float                                              integral                = 0.0f;
            float                                              prev_error              = 0.0f;
            std::chrono::time_point<std::chrono::steady_clock> prev_control_timestamp_ = std::chrono::steady_clock::now();
        };
    }  // namespace scmodel

}  // namespace rsim

#endif  // SAFETY_CAR_MODEL_HXX