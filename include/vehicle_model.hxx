#ifndef VEHICLE_MODEL_HXX
#define VEHICLE_MODEL_HXX

#include <boost/numeric/odeint.hpp>
#include <cmath>
#include <random>

#include "utility.hxx"

namespace rsim
{
    namespace vmodel
    {
        class State
        {
        public:
            float x                = 0.0f;
            float y                = 0.0f;
            float orientation      = 0.0f;
            float velocity         = 0.0f;
            float angular_velocity = 0.0f;
            float wheel_angle      = 0.0f;
            float wheel_rpm        = 0.0f;
            float motor_rpm        = 0.0f;

            State(float x_, float y_, float orientation_) : x(x_), y(y_), orientation(orientation_) {}

            void update(const float wheel_angle_in, const float velocity_in, const float dt)
            {
                if (dt == 0) return;

                // Limit the input values to their maximums
                wheel_angle = std::max(-MAX_WHEEL_ANGLE, std::min(MAX_WHEEL_ANGLE, wheel_angle_in));

                // check velocity for max acceleration and max deceleration
                if (velocity_in > velocity) { velocity = std::min(velocity + MAX_ACCELERATION * dt, velocity_in); }
                else { velocity = std::max(velocity - MAX_DECELERATION * dt, velocity_in); }

                velocity = std::max(-MAX_VELOCITY, std::min(MAX_VELOCITY, velocity));

                // Calculate the new orientation and position
                float delta_s     = velocity * dt;
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

            float noisy_yaw_rate() { return angular_velocity + yaw_rate_distribution(generator); }

            float noisy_motor_rpm() { return motor_rpm + motor_rpm_distribution(generator); }

        private:
            std::default_random_engine      generator;
            std::normal_distribution<float> motor_rpm_distribution{0.0f, MOTOR_RPM_NOISE};
            std::normal_distribution<float> yaw_rate_distribution{0.0f, YAW_RATE_NOISE};
        };

        // numerical integration
        class Integrator
        {
        public:
            float value;

            Integrator(float initial_value) : value(initial_value) {}

            float integrate(float input, float dt)
            {
                // return euler_method(input, dt);
                return runge_kutta_4(input, dt);
            }

            float euler_method(float input, float dt)
            {
                float tmp = input * dt;
                value += tmp;

                return value;
            }

            float runge_kutta_4(float input, float dt)
            {
                float k1 = input;
                float k2 = input + 0.5 * dt * k1;
                float k3 = input + 0.5 * dt * k2;
                float k4 = input + dt * k3;

                float tmp = (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
                value += tmp;

                return value;
            }
        };

        // class PacejkaTireModel
        // {
        // public:
        //     float xi = 0.0f;
        //     float FyF_t = 0.0f;
        //     float FyR_t = 0.0f;

        //     void update(const float alphaF_t, const float alphaR_t, [[maybe_unused]] const float Fx_t)
        //     {
        //         // // rad to deg
        //         float alphaF_t_deg = alphaF_t * 180 / M_PI;
        //         float alphaR_t_deg = alphaR_t * 180 / M_PI;

        //         xi = sqrt(pow(mu * m * g * lr / (lf + lr), 2) - pow(Fx_t, 2)) / (mu * m * g * lr / (lf + lr));

        //         FyF_t = -D_front * sinf(C_front * atanf(B_front * ((1 - E_front) * (alphaF_t_deg + Sh_front) + (E_front / B_front) * atanf(B_front
        //         * (alphaF_t_deg + Sh_front))))) + Sv_front;

        //         FyR_t = -(D_rear * xi) * sinf(C_rear * atanf(B_rear * ((1 - E_rear) * (alphaR_t_deg + Sh_rear) + (E_rear / B_rear) * atanf(B_rear *
        //         (alphaR_t_deg + Sh_rear))))) + Sv_rear;
        //     }
        // };

        class SimplifiedPacejkaModel
        {
        public:
            float FyF_t = 0.0f;
            float FyR_t = 0.0f;

            void update(const float alphaF_t, const float alphaR_t, [[maybe_unused]] const float delta_t, [[maybe_unused]] const float Fx_t)
            {
                // float F_zf = m * (-delta_t * h + g * lr) / (lr + lf);
                // float F_zr = m * (delta_t * h + g * lf) / (lr + lf);

                float F_zf = m * g * lr / (lr + lf);
                float F_zr = m * g * lf / (lr + lf);

                FyF_t = -mu * F_zf * C_f * alphaF_t;
                FyR_t = -mu * F_zr * C_r * alphaR_t;
            }
        };

        class DynamicBicycleModel
        {
        public:
            float alphaF_t      = 0.0f;
            float alphaR_t      = 0.0f;
            float yaw_rate_t    = 0.0f;
            float beta_t        = 0.0f;
            float vy_t          = 0.0f;
            float vx_t          = 0.0f;
            float wheel_rpm     = 0.0f;
            float motor_rpm     = 0.0f;
            float x_t           = 0.0f;
            float y_t           = 0.0f;
            float orientation_t = 0.0f;

            Integrator vx_integrator{vx_t};
            Integrator yaw_rate_integrator{yaw_rate_t};
            Integrator beta_integrator{beta_t};
            Integrator x_integrator{x_t};
            Integrator y_integrator{y_t};
            Integrator orientation_integrator{orientation_t};

            SimplifiedPacejkaModel tire_model;

            float previous_wheel_angle = 0.0f;

            DynamicBicycleModel(float x_, float y_, float orientation_)
                : x_t(x_), y_t(y_), orientation_t(orientation_), x_integrator(x_t), y_integrator(y_t), orientation_integrator(orientation_t)
            {
            }

            void tire_slip_calculation([[maybe_unused]] const float delta_t)
            {
                if (vx_t < 0.0001f)
                {
                    alphaR_t = 0.0f;
                    alphaF_t = 0.0f;
                }
                else
                {
                    alphaR_t = atanf(beta_t - (yaw_rate_t * lr / vx_t));
                    alphaF_t = atanf(beta_t + (yaw_rate_t * lf / vx_t)) - delta_t;
                }
            }

            void vx_calculation(const float delta_t, const float Fx_t, const float dt)
            {
                float value = (Fx_t - (tire_model.FyF_t * sinf(delta_t))) / m + yaw_rate_t * beta_t * vx_t;
                vx_t        = vx_integrator.integrate(value, dt);
                wheel_rpm   = (vx_t / (M_PI * WHEEL_DIAMETER)) * 60.0f;
                motor_rpm   = wheel_rpm / GEAR_RATIO_MOTOR_TO_WHEEL;
            }

            void yaw_rate_calculation(const float dt)
            {
                float value = (tire_model.FyF_t * lf - tire_model.FyR_t * lr) / Iz;
                yaw_rate_t  = yaw_rate_integrator.integrate(value, dt);
            }

            void beta_calculation(const float dt)
            {
                if (vx_t < 0.0001f) { beta_t = 0.0f; }
                else
                {
                    float value = (tire_model.FyF_t + tire_model.FyR_t) / (vx_t * m) - yaw_rate_t;
                    beta_t      = beta_integrator.integrate(value, dt);
                }
            }

            void vy_calculation() { vy_t = tanf(beta_t) * vx_t; }

            void position_calculation(const float dt)
            {
                orientation_t = orientation_integrator.integrate(yaw_rate_t, dt);

                float value_x = sqrt(vx_t * vx_t + vy_t * vy_t) * cos(beta_t + orientation_t);
                x_t           = x_integrator.integrate(value_x, dt);

                float value_y = sqrt(vx_t * vx_t + vy_t * vy_t) * sin(beta_t + orientation_t);
                y_t           = y_integrator.integrate(value_y, dt);
            }

            void update_impl(const float delta_t, const float Fx_t, const float dt)
            {
                tire_slip_calculation(delta_t);
                tire_model.update(alphaF_t, alphaR_t, delta_t, Fx_t);
                vx_calculation(delta_t, Fx_t, dt);
                yaw_rate_calculation(dt);
                beta_calculation(dt);
                vy_calculation();
                position_calculation(dt);

                std::cout << "delta_t"
                          << "\t\t" << delta_t << std::endl;
                std::cout << "alphaF_t"
                          << "\t" << alphaF_t << std::endl;
                std::cout << "alphaR_t"
                          << "\t" << alphaR_t << std::endl;
                std::cout << "FyR_t"
                          << "\t\t" << tire_model.FyR_t << std::endl;
                std::cout << "FyF_t"
                          << "\t\t" << tire_model.FyF_t << std::endl;
                std::cout << "w_t"
                          << "\t\t" << yaw_rate_t << std::endl;
                std::cout << "vy_t"
                          << "\t\t" << vy_t << std::endl;
                std::cout << "vx_t"
                          << "\t\t" << vx_t << std::endl;
                std::cout << "x_t"
                          << "\t\t" << x_t << std::endl;
                std::cout << "y_t"
                          << "\t\t" << y_t << std::endl;
                std::cout << "orientation_t"
                          << "\t" << orientation_t << std::endl;
                std::cout << "------------------------" << std::endl;
            }

            void update(const float wheel_angle_in, const float velocity_in, const float dt)
            {
                if (dt == 0) return;

                // Limit the input values to their maximums
                float                  wheel_angle = std::max(-MAX_WHEEL_ANGLE, std::min(MAX_WHEEL_ANGLE, wheel_angle_in));
                [[maybe_unused]] float velocity    = std::max(-MAX_VELOCITY, std::min(MAX_VELOCITY, velocity_in));

                // float d = 2.0f;
                // float Fx_t = vx_t > 0 ? Cm1 * d - Cm2 * vx_t - Cm3 : Cm1 * d - Cm2 * vx_t + Cm3;

                float Fx_t = 25.0f;

                update_impl(wheel_angle, Fx_t, dt);

                previous_wheel_angle = wheel_angle;
            }

            float noisy_yaw_rate() { return yaw_rate_t + yaw_rate_distribution(generator); }

            float noisy_motor_rpm() { return motor_rpm + motor_rpm_distribution(generator); }

        private:
            std::default_random_engine      generator;
            std::normal_distribution<float> motor_rpm_distribution{0.0f, MOTOR_RPM_NOISE};
            std::normal_distribution<float> yaw_rate_distribution{0.0f, YAW_RATE_NOISE};
        };

        // proxy for dynamic bicycle model
        class DynamicState : public State
        {
        public:
            DynamicState(float x_, float y_, float orientation_) : State(x_, y_, orientation_), model(x_, y_, orientation_) {}

            void update(const float wheel_angle_in, const float velocity_in, const float dt)
            {
                model.update(wheel_angle_in, velocity_in, dt);
                x                = model.x_t;
                y                = model.y_t;
                orientation      = model.orientation_t;
                velocity         = model.vx_t;
                angular_velocity = model.yaw_rate_t;
                wheel_angle      = wheel_angle_in;
                wheel_rpm        = model.wheel_rpm;
                motor_rpm        = model.motor_rpm;
            }

            float noisy_yaw_rate() { return model.noisy_yaw_rate(); }

            float noisy_motor_rpm() { return model.noisy_motor_rpm(); }

        private:
            DynamicBicycleModel model;
        };
    }  // namespace vmodel

}  // namespace rsim

#endif  // VEHICLE_MODEL_HXX