#ifndef SIMULATION_HXX
#define SIMULATION_HXX

#include "utility.hxx"
#include "environment.hxx"
#include "pirate_model.hxx"
#include "safety_car_model.hxx"

namespace rsim
{
    class Simulation
    {
    public:
        env::Car car;
        env::Car pirate;
        env::Car safety_car;
        env::Map map;

        pmodel::PirateController pirate_controller;
        scmodel::SafetyCarController safety_car_controller;

        int collected_points = 0;
        bool car_under_gate = false;
        bool car_at_cross_section = false;

        Simulation(const float x_t_ = START_X, const float y_t_ = START_Y, const float theta_t_ = START_ORIENTATION) : car{x_t_, y_t_, theta_t_}, pirate{PIRATE_START_X, PIRATE_START_Y, PIRATE_START_ORIENTATION}, safety_car{SAFETY_CAR_START_X, SAFETY_CAR_START_Y, SAFETY_CAR_START_ORIENTATION} {}

        void update([[maybe_unused]] const float target_angle, [[maybe_unused]] const float target_speed)
        {
            car_under_gate = false;
            car_at_cross_section = false;
            bool pirate_under_gate = false;
            bool pirate_at_cross_section = false;
            for (auto &gate : map.gates)
            {
                if (std::sqrt(std::pow(gate.x - car.state.x, 2) + std::pow(gate.y - car.state.y, 2)) < 8.0f)
                {
                    car_under_gate = true;
                    car_at_cross_section = true;
                    if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gate.last_seen).count() > 2)
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
                    pirate_under_gate = true;
                    pirate_at_cross_section = true;
                    if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gate.last_stolen).count() > 2)
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

            car.update(target_angle, m_to_px(target_speed));

            pirate.detect_front(map.data);
            pirate_controller.update(pirate.line_sensor_front.get_detection(), pirate_under_gate, pirate_at_cross_section, pirate.state);
            pirate.update(pirate_controller.target_angle, pirate_controller.target_speed);

            safety_car.detect_front(map.data);
            safety_car_controller.update(safety_car.line_sensor_front.get_detection());
            safety_car.update(safety_car_controller.target_angle, safety_car_controller.target_speed);

            // std::cout << safety_car_controller.target_speed << "\t" << safety_car.state.velocity << std::endl;
        }

    private:
    };
} // namespace rsim

#endif // SIMULATION_HXX