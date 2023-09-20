#ifndef SENSOR_MODEL_HXX
#define SENSOR_MODEL_HXX

#include "vehicle_model.hxx"

class Map;

namespace rsim
{
    namespace smodel
    {
        static constexpr int SENSOR_WIDTH = 16;

        // this is simulating a Line sensor that has a 16x1 return array
        class LineSensor
        {
        public:
            bool detection[SENSOR_WIDTH];

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
            void detect(bool (&map)[cols][rows])
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
            }

        private:
            rsim::vmodel::State state;
        };
    } // namespace smodel

} // namespace rsim

#endif // SENSOR_MODEL_HXX