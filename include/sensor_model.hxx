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

            void detect(bool map[][1024], unsigned long map_width, unsigned long map_height)
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
                    if (x >= 0 && x < map_width && y >= 0 && y < map_height)
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