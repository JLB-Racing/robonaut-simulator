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
                for (int i = 0; i < SENSOR_WIDTH; i++)
                    detection[i] = true;
            }

            void update(const rsim::vmodel::State &state_)
            {
                state = state_;
            }

            template <size_t cols, size_t rows>
            bool (&detect(bool (&map)[cols][rows]))[SENSOR_WIDTH]
            {
                // Reset detection results
                for (int i = 0; i < SENSOR_WIDTH; i++)
                    detection[i] = true;

                // Calculate the points along the sensor's line
                for (int i = 0; i < SENSOR_WIDTH; i++)
                {
                    // Calculate the position of the point along the line
                    unsigned long x = state.x + (i - SENSOR_WIDTH / 2) * std::cos(state.orientation + M_PI / 2);
                    unsigned long y = state.y + (i - SENSOR_WIDTH / 2) * std::sin(state.orientation + M_PI / 2);

                    // Check if the point is within the map bounds
                    if (x >= 0 && x < cols && y >= 0 && y < rows)
                    {
                        detection[i] = map[x][y];
                    }
                }

                return detection;
            }

            bool (&get_detection())[SENSOR_WIDTH]
            {
                return detection;
            }

            void set_detection(unsigned long i, bool value)
            {
                detection[i] = value;
            }

        private:
            rsim::vmodel::State state;
            bool detection[SENSOR_WIDTH];
        };
    } // namespace smodel

} // namespace rsim

#endif // SENSOR_MODEL_HXX