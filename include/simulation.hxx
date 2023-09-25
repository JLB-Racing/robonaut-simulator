#ifndef SIMULATION_HXX
#define SIMULATION_HXX

#include "environment.hxx"
#include "pirate_model.hxx"

namespace rsim
{
    double start_x = 320.0;
    double start_y = 724.0;
    double start_orientation = -M_PI / 2.0;

    double pirate_start_x = 320.0;
    double pirate_start_y = 300.0;
    double pirate_start_orientation = M_PI / 2.0;

    class Simulation
    {
    public:
        env::Car car;
        env::Car pirate;
        env::Map map;

        logic::PirateController pirate_controller;

        int collected_points = 0;

        Simulation() : car{start_x, start_y, start_orientation}, pirate{pirate_start_x, pirate_start_y, pirate_start_orientation}
        {
        }

        void update(const double target_angle, const double target_speed)
        {
            bool car_under_gate = false;
            bool pirate_under_gate = false;
            for (auto &gate : map.gates)
            {
                if (std::sqrt(std::pow(gate.x - car.state.x, 2) + std::pow(gate.y - car.state.y, 2)) < 8)
                {
                    car_under_gate = true;
                    for (unsigned long i = 0; i < smodel::SENSOR_WIDTH; i++)
                    {
                        // 1111010110101111 gate signature
                        if (i == smodel::SENSOR_WIDTH / 2 + 1 || i == smodel::SENSOR_WIDTH / 2 - 1 - 1 ||
                            i == smodel::SENSOR_WIDTH / 2 + 3 || i == smodel::SENSOR_WIDTH / 2 - 1 - 3)
                        {
                            car.line_sensor.detection[i] = false;
                        }
                        else
                        {
                            car.line_sensor.detection[i] = true;
                        }
                    }
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

                if (std::sqrt(std::pow(gate.x - pirate.state.x, 2) + std::pow(gate.y - pirate.state.y, 2)) < 8)
                {
                    pirate_under_gate = true;
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

            car.detect(map.data);
            car.update(target_angle, target_speed);

            pirate.detect(map.data);
            pirate_controller.update(pirate.line_sensor.detection, pirate_under_gate);
            pirate.update(pirate_controller.target_angle, pirate_controller.target_speed);
        }

    private:
    };
} // namespace rsim

#endif // SIMULATION_HXX