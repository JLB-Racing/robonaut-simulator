#ifndef SENSOR_MODEL_HXX
#define SENSOR_MODEL_HXX

#include "vehicle_model.hxx"

class Map;

namespace rsim
{
    namespace smodel
    {
        class LineSensor
        {
        public:
            LineSensor(const rsim::vmodel::State &state_) : state{state_}
            {
                for (int i = 0; i < SENSOR_COUNT; i++)
                    detection[i] = true;
            }

            void update(const rsim::vmodel::State &state_)
            {
                state = state_;
            }

            template <size_t cols, size_t rows>
            bool (&detect(bool (&map)[cols][rows], int offset = 0))[SENSOR_COUNT]
            {
                // Reset detection results
                for (int i = 0; i < SENSOR_COUNT; i++)
                    detection[i] = true;

                // Calculate the points along the sensor's line
                for (int i = 0; i < SENSOR_COUNT; i++)
                {
                    // Calculate the position of the point along the line
                    // unsigned long x = state.x + (i - SENSOR_COUNT / 2) * std::cos(state.orientation + M_PI / 2);
                    // unsigned long y = state.y + (i - SENSOR_COUNT / 2) * std::sin(state.orientation + M_PI / 2);

                    // the above commented out code is the original code, but I've added an offset parameter, that allows the sensor to be offset from the center of the car in the direction of the car's orientation
                    unsigned long x = state.x + (i - SENSOR_COUNT / 2) * std::cos(state.orientation + M_PI / 2) + offset * std::cos(state.orientation);
                    unsigned long y = state.y + (i - SENSOR_COUNT / 2) * std::sin(state.orientation + M_PI / 2) + offset * std::sin(state.orientation);

                    // Check if the point is within the map bounds
                    if (x >= 0 && x < cols && y >= 0 && y < rows)
                    {
                        detection[i] = map[x][y];
                    }
                }

                return detection;
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
            rsim::vmodel::State state;
            bool detection[SENSOR_COUNT];
        };
    } // namespace smodel

} // namespace rsim

#endif // SENSOR_MODEL_HXX