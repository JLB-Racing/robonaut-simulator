#ifndef SIMULATION_HXX
#define SIMULATION_HXX

#include "environment.hxx"
#include "pirate_model.hxx"
#include "safety_car_model.hxx"
#include "utility.hxx"

namespace rsim
{
    class Simulation
    {
    public:
        env::Car car;
        env::Car pirate;
        env::Car safety_car;
        env::Map map;

        pmodel::PirateController     pirate_controller;
        scmodel::SafetyCarController safety_car_controller;

        int                     collected_points     = 0;
        bool                    car_under_gate       = false;
        bool                    car_at_cross_section = false;
        bool                    flood                = false;
        bool                    safety_car_reset     = false;
        pmodel::PirateInterface pirate_interface;

        Simulation(const float x_t_ = START_X, const float y_t_ = START_Y, const float theta_t_ = START_ORIENTATION)
            : car{x_t_, y_t_, theta_t_},
              pirate{PIRATE_START_X, PIRATE_START_Y, PIRATE_START_ORIENTATION},
              safety_car{SAFETY_CAR_START_X, SAFETY_CAR_START_Y, SAFETY_CAR_START_ORIENTATION}
        {
        }

        void update([[maybe_unused]] const float target_angle, [[maybe_unused]] const float target_speed)
        {
            car_under_gate               = false;
            car_at_cross_section         = false;
            bool pirate_under_gate       = false;
            bool pirate_at_cross_section = false;

            // #ifndef SEND_RADIO
            //             if (std::sqrt(std::pow(BALANCER_END_CENTER_X - car.state.x, 2) + std::pow(BALANCER_END_CENTER_Y - car.state.y, 2)) <
            //             BALANCER_END_RADIUS)
            //             {
            //                 flood = false;
            //             }
            // #endif

            if (safety_car_reset)
            {
                safety_car.state.x = 64;
                safety_car.state.y = 256;
            }

            for (auto &gate : map.gates)
            {
                if (std::sqrt(std::pow(gate.x - car.state.x, 2) + std::pow(gate.y - car.state.y, 2)) < 8.0f)
                {
                    car_under_gate       = true;
                    car_at_cross_section = true;
                    if (!flood && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gate.last_seen).count() > 2)
                    {
                        switch (gate.state)
                        {
                            case env::Gate::State::UNMAPPED:
                                gate.state = env::Gate::State::MAPPED;
                                collected_points += 2;
                                break;
                            case env::Gate::State::STOLEN_ONCE:
                                gate.state = env::Gate::State::MAPPED;
                                collected_points += 1;
                                break;
                            case env::Gate::State::STOLEN_TWICE:
                                gate.state = env::Gate::State::MAPPED;
                                break;

                            case env::Gate::State::MAPPED:
                                break;

                            default:
                                break;
                        }
                    }
                    gate.last_seen = std::chrono::steady_clock::now();
                }

                if (std::sqrt(std::pow(gate.x - pirate.state.x, 2) + std::pow(gate.y - pirate.state.y, 2)) < 8.0f)
                {
                    pirate_under_gate       = true;
                    pirate_at_cross_section = true;
                    if (!flood && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gate.last_stolen).count() > 2)
                    {
                        switch (gate.state)
                        {
                            case env::Gate::State::UNMAPPED:
                                gate.state = env::Gate::State::STOLEN_ONCE;
                                collected_points += 2;
                                break;
                            case env::Gate::State::STOLEN_ONCE:
                                gate.state = env::Gate::State::STOLEN_TWICE;
                                collected_points += 1;
                                break;
                            case env::Gate::State::STOLEN_TWICE:
                                gate.state = env::Gate::State::STOLEN_TWICE;
                                break;

                            case env::Gate::State::MAPPED:
                                break;

                            default:
                                break;
                        }
                    }
                    gate.last_stolen = std::chrono::steady_clock::now();
                }
            }

            for (auto &cross_section : map.cross_sections)
            {
                if (std::sqrt(std::pow(cross_section.x - car.state.x, 2) + std::pow(cross_section.y - car.state.y, 2)) < 8.0f)
                {
                    car_at_cross_section = true;
                }

                if (std::sqrt(std::pow(cross_section.x - pirate.state.x, 2) + std::pow(cross_section.y - pirate.state.y, 2)) < 8.0f)
                {
                    pirate_at_cross_section = true;
                }
            }

#ifndef SEND_RADIO
            car.update(target_angle, m_to_px(target_speed));
#endif

            [[maybe_unused]] auto [detection_front, line_positions_front] = pirate.detect_front(map.data);
            [[maybe_unused]] auto [detection_rear, line_positions_rear]   = pirate.detect_rear(map.data);
            pirate_controller.set_detection_front(detection_front, line_positions_front);
            pirate_controller.set_detection_rear(detection_rear, line_positions_rear);
            pirate_interface = pirate_controller.update(pirate_under_gate, pirate_at_cross_section, pirate.state);
            pirate.update(pirate_controller.target_angle, pirate_controller.target_speed);
            pirate_controller.set_flood(flood);

            safety_car.detect_front(map.data);
            safety_car_controller.update(safety_car.line_sensor_front.get_detection());
            safety_car.update(safety_car_controller.target_angle, safety_car_controller.target_speed);
        }

        void start() { pirate_controller.start(); }

        void reset()
        {
            pirate_controller.reset();
            pirate.state.x           = PIRATE_START_X;
            pirate.state.y           = PIRATE_START_Y;
            pirate.state.orientation = PIRATE_START_ORIENTATION;

            car.state.x           = START_X;
            car.state.y           = START_Y;
            car.state.orientation = START_ORIENTATION;
        }

    private:
    };
}  // namespace rsim

#endif  // SIMULATION_HXX