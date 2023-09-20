#ifndef SIMULATION_HXX
#define SIMULATION_HXX

#include "environment.hxx"

namespace rsim
{
    class Simulation
    {
    public:
        rsim::env::Car car;
        rsim::env::Car opp;
        rsim::env::Map map;

        int collected_points = 0;

        Simulation(const double start_x, const double start_y, const double start_orientation, const double opp_start_x, const double opp_start_y, const double opp_start_orientation) : car{start_x, start_y, start_orientation}, opp{opp_start_x, opp_start_y, opp_start_orientation}
        {
        }

        void update(const double wheel_angle, const double velocity, const double opp_wheel_angle, double opp_velocity)
        {
            car.detect(map.data);
            car.update(wheel_angle, velocity);

            opp.detect(map.data);
            opp.update(opp_wheel_angle, opp_velocity);

            for (auto &gate : map.gates)
            {
                if (std::sqrt(std::pow(gate.x - car.state.x, 2) + std::pow(gate.y - car.state.y, 2)) < 8)
                {
                    for (unsigned long i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
                    {
                        // 1111010110101111 gate signature
                        if (i == rsim::smodel::SENSOR_WIDTH / 2 + 1 || i == rsim::smodel::SENSOR_WIDTH / 2 - 1 - 1 ||
                            i == rsim::smodel::SENSOR_WIDTH / 2 + 3 || i == rsim::smodel::SENSOR_WIDTH / 2 - 1 - 3)
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
                        case rsim::env::Gate::State::UNMAPPED:
                            gate.state = rsim::env::Gate::State::MAPPED;
                            collected_points += 2;
                            break;
                        case rsim::env::Gate::State::STOLEN_ONCE:
                            gate.state = rsim::env::Gate::State::MAPPED;
                            collected_points += 1;
                            break;
                        case rsim::env::Gate::State::STOLEN_TWICE:
                            gate.state = rsim::env::Gate::State::MAPPED;
                            break;

                        case rsim::env::Gate::State::MAPPED:
                            break;

                        default:
                            break;
                        }
                    }
                    gate.last_seen = std::chrono::steady_clock::now();
                }

                if (std::sqrt(std::pow(gate.x - opp.state.x, 2) + std::pow(gate.y - opp.state.y, 2)) < 8)
                {
                    for (unsigned long i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
                    {
                        // 1111010110101111 gate signature
                        // if (i == rsim::smodel::SENSOR_WIDTH / 2 + 1 || i == rsim::smodel::SENSOR_WIDTH / 2 - 1 - 1 ||
                        //     i == rsim::smodel::SENSOR_WIDTH / 2 + 3 || i == rsim::smodel::SENSOR_WIDTH / 2 - 1 - 3)
                        // {
                        //     opp.line_sensor.detection[i] = false;
                        // }
                        // else
                        // {
                        //     opp.line_sensor.detection[i] = true;
                        // }
                    }
                    if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gate.last_stolen).count() > 2)
                    {
                        switch (gate.state)
                        {
                        case rsim::env::Gate::State::UNMAPPED:
                            gate.state = rsim::env::Gate::State::STOLEN_ONCE;
                            collected_points += 2;
                            break;
                        case rsim::env::Gate::State::STOLEN_ONCE:
                            gate.state = rsim::env::Gate::State::STOLEN_TWICE;
                            collected_points += 1;
                            break;
                        case rsim::env::Gate::State::STOLEN_TWICE:
                            gate.state = rsim::env::Gate::State::STOLEN_TWICE;
                            break;

                        case rsim::env::Gate::State::MAPPED:
                            break;

                        default:
                            break;
                        }
                    }
                    gate.last_stolen = std::chrono::steady_clock::now();
                }
            }
        }

    private:
    };
} // namespace rsim

#endif // SIMULATION_HXX