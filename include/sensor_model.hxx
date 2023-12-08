#ifndef SENSOR_MODEL_HXX
#define SENSOR_MODEL_HXX

#include "vehicle_model.hxx"
#include "utility.hxx"

class Map;

namespace rsim
{
    namespace smodel
    {

        struct SensorDetection
        {
            bool detection[SENSOR_COUNT];
            std::vector<float> line_positions;

            SensorDetection(bool (&detection_)[SENSOR_COUNT], std::vector<float> line_positions_) : line_positions{line_positions_}
            {
                for (int i = 0; i < SENSOR_COUNT; i++)
                    detection[i] = detection_[i];
            }
        };

        class LineSensor
        {
        public:
            LineSensor(const vmodel::State &state_) : state{state_}
            {
                for (int i = 0; i < SENSOR_COUNT; i++)
                    detection[i] = true;
            }

            ~LineSensor() {}

            void update(const vmodel::State &state_)
            {
                state = state_;
            }

            template <size_t cols, size_t rows>
            SensorDetection detect(bool (&map)[cols][rows], int offset = 0)
            {
                // Reset detection results
                for (int i = 0; i < SENSOR_COUNT; i++)
                    detection[i] = true;

                // Reset line positions
                line_positions.clear();

                float sensor_center = (SENSOR_COUNT + 1) / 2.0f;

                // Calculate the points along the sensor's line
                for (int i = 0; i < SENSOR_COUNT; i++)
                {
                    unsigned long x = static_cast<unsigned long>(state.x) +
                                      offset * std::cos(state.orientation) +
                                      (i - SENSOR_COUNT / 2) * std::cos(state.orientation + M_PI / 2.0f);
                    unsigned long y = static_cast<unsigned long>(state.y) +
                                      offset * std::sin(state.orientation) +
                                      (i - SENSOR_COUNT / 2) * std::sin(state.orientation + M_PI / 2.0f);

                    // Check if the point is within the map bounds
                    if (x >= 0 && x < cols && y >= 0 && y < rows)
                    {
                        detection[i] = map[x][y];
                    }
                }

                // iterate over the detection array to find clusters of detected points and calculate their center of mass relative to the center of the sensor

                unsigned long cluster_start = SENSOR_COUNT + 1;
                unsigned long cluster_end = SENSOR_COUNT + 1;
                for (unsigned long current_idx = 0; current_idx < SENSOR_COUNT; current_idx++)
                {
                    if (!detection[current_idx])
                    {
                        if (cluster_start == SENSOR_COUNT + 1)
                        {
                            cluster_start = current_idx;
                        }
                        cluster_end = current_idx;
                    }
                    else
                    {
                        if (cluster_start != SENSOR_COUNT + 1)
                        {
                            // calculate the center of mass of the cluster
                            float cluster_center = (cluster_start + 1 + cluster_end + 1) / 2.0f;
                            // calculate the position of the line relative to the center of the sensor
                            float line_position = cluster_center - sensor_center;
                            // add the line position to the vector
                            line_positions.push_back(line_position);
                            // reset the cluster start and end
                            cluster_start = SENSOR_COUNT + 1;
                            cluster_end = SENSOR_COUNT + 1;
                        }
                    }
                }

                return SensorDetection{detection, line_positions};
            }

            bool (&get_detection())[SENSOR_COUNT]
            {
                return detection;
            }

            void set_detection(unsigned long i, bool value)
            {
                detection[i] = value;
            }

        private:
            vmodel::State state;
            bool detection[SENSOR_COUNT];
            std::vector<float> line_positions;
        };

        class ObjectSensor
        {
        public:
            ObjectSensor(const vmodel::State &state_) : state{state_}
            {
            }

            ~ObjectSensor()
            {
            }

            void update(const vmodel::State &state_)
            {
                state = state_;
            }

            float detect(const vmodel::State &object_)
            {
                float distance = std::sqrt(std::pow(object_.x - state.x, 2) + std::pow(object_.y - state.y, 2));
                return distance;
            }

        private:
            vmodel::State state;
        };
    } // namespace smodel

} // namespace rsim

#endif // SENSOR_MODEL_HXX