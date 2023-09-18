#ifndef ENVIRONMENT_HXX
#define ENVIRONMENT_HXX

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <bitset>

#include "vehicle_model.hxx"
#include "sensor_model.hxx"

namespace rsim
{

    namespace env
    {
        static constexpr int BITMAP_SIZE = 64;
        static constexpr int GRID_WIDTH = 15;
        static constexpr int GRID_HEIGHT = 15;
        static constexpr int MAP_WIDTH = GRID_WIDTH * BITMAP_SIZE;
        static constexpr int MAP_HEIGHT = GRID_HEIGHT * BITMAP_SIZE;

        class Bitmap
        {
        public:
            bool bitmap[BITMAP_SIZE][BITMAP_SIZE];

            Bitmap()
            {
                for (int i = 0; i < BITMAP_SIZE; i++)
                {
                    for (int j = 0; j < BITMAP_SIZE; j++)
                    {
                        bitmap[i][j] = true;
                    }
                }
            }

            Bitmap(const std::string &filename)
            {
                auto file = std::ifstream(filename, std::ios::binary);
                if (!file.is_open())
                {
                    std::cerr << "Could not open file: " << filename << std::endl;
                    return;
                }

                std::string binary;
                std::getline(file, binary);
                std::vector<uint8_t> data = std::vector<uint8_t>(binary.begin(), binary.end());
                std::stringstream ss;
                unsigned long header = 0;
                for (auto &byte : data)
                {
                    if (header < 62)
                    {
                        header++;
                        continue;
                    }
                    ss << std::bitset<8>(byte);
                }
                file.close();

                // populate bitmap 64x64
                std::string bits = ss.str();
                unsigned long row = 0;
                unsigned long col = 0;
                for (auto &bit : bits)
                {
                    if (bit == '1')
                        bitmap[col][row] = true;
                    else
                        bitmap[col][row] = false;
                    col++;
                    if (col == BITMAP_SIZE)
                    {
                        col = 0;
                        row++;
                    }
                }
            }

            ~Bitmap() = default;

            void rotate90()
            {
                bool rotated[BITMAP_SIZE][BITMAP_SIZE];

                // Transpose
                for (int i = 0; i < BITMAP_SIZE; i++)
                {
                    for (int j = 0; j < BITMAP_SIZE; j++)
                    {
                        rotated[i][j] = bitmap[j][i];
                    }
                }

                // reversing each row
                for (int i = 0; i < BITMAP_SIZE; i++)
                {
                    for (int j = 0, k = BITMAP_SIZE - 1; j < k; j++, k--)
                    {
                        std::swap(rotated[j][i], rotated[k][i]);
                    }
                }

                for (int i = 0; i < BITMAP_SIZE; i++)
                {
                    for (int j = 0; j < BITMAP_SIZE; j++)
                    {
                        bitmap[i][j] = rotated[i][j];
                    }
                }
            }

            void rotate180()
            {
                rotate90();
                rotate90();
            }

            void rotate270()
            {
                rotate90();
                rotate90();
                rotate90();
            }

            void flip_horizontal()
            {
                for (int i = 0; i < BITMAP_SIZE; i++)
                {
                    for (int j = 0, k = BITMAP_SIZE - 1; j < k; j++, k--)
                    {
                        std::swap(bitmap[j][i], bitmap[k][i]);
                    }
                }
            }

            void flip_vertical()
            {
                for (int i = 0; i < BITMAP_SIZE; i++)
                {
                    for (int j = 0, k = BITMAP_SIZE - 1; j < k; j++, k--)
                    {
                        std::swap(bitmap[i][j], bitmap[i][k]);
                    }
                }
            }

            friend std::ostream &operator<<(std::ostream &os, const Bitmap &bitmap)
            {
                for (int i = 0; i < BITMAP_SIZE; i++)
                {
                    for (int j = 0; j < BITMAP_SIZE; j++)
                    {
                        if (bitmap.bitmap[i][j])
                            os << "░";
                        else
                            os << "█";
                    }
                    os << std::endl;
                }
                return os;
            }
        };

        class Gate
        {
        public:
            enum class State
            {
                UNMAPPED,
                STOLEN_ONCE,
                STOLEN_TWICE,
                MAPPED,
            };

            State state = State::UNMAPPED;
            double x;
            double y;

            Gate(double x_, double y_) : x{x_}, y{y_} {}
        };

        class Map
        {
        public:
            bool data[MAP_WIDTH][MAP_HEIGHT];
            std::vector<Gate> gates;

            Map()
            {
                for (unsigned long col = 0; col < MAP_WIDTH; col++)
                    for (unsigned long row = 0; row < MAP_HEIGHT; row++)
                        data[col][row] = true;

                build_grid();
                build_gates();

                // iterate over grid and fill in data
                for (int i = 0; i < GRID_WIDTH; i++)
                {
                    for (int j = 0; j < GRID_HEIGHT; j++)
                    {
                        for (int k = 0; k < BITMAP_SIZE; k++)
                        {
                            for (int l = 0; l < BITMAP_SIZE; l++)
                            {
                                data[i * BITMAP_SIZE + k][j * BITMAP_SIZE + l] = grid[i][j].bitmap[k][l];
                            }
                        }
                    }
                }
            }

        private:
            Bitmap grid[GRID_HEIGHT][GRID_WIDTH] = {{rsim::env::Bitmap{}}};
            Bitmap start = Bitmap("assets/start.bmp");
            Bitmap line = Bitmap("assets/line.bmp");
            Bitmap line_dotted = Bitmap("assets/line-dotted.bmp");
            Bitmap turn = Bitmap("assets/turn.bmp");
            Bitmap turn2 = Bitmap("assets/turn-2.bmp");
            Bitmap turn3 = Bitmap("assets/turn-3.bmp");
            Bitmap turn_line = Bitmap("assets/turn-line.bmp");
            Bitmap turn_line2 = Bitmap("assets/turn-line-2.bmp");
            Bitmap turn_line3 = Bitmap("assets/turn-line-3.bmp");
            Bitmap turn_line_wide = Bitmap("assets/turn-line-wide.bmp");
            Bitmap turn_line2_wide = Bitmap("assets/turn-line-2-wide.bmp");
            Bitmap turn_line3_wide = Bitmap("assets/turn-line-3-wide.bmp");

            void build_grid()
            {
                auto copy = start;
                copy.flip_vertical();
                grid[4][3] = std::move(copy);

                copy = start;
                copy.flip_horizontal();
                copy.flip_vertical();
                grid[5][3] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                grid[5][4] = std::move(copy);

                copy = line;
                grid[3][5] = std::move(copy);

                copy = turn_line2_wide;
                copy.flip_horizontal();
                copy.flip_vertical();
                grid[4][5] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                copy.flip_vertical();
                grid[5][5] = std::move(copy);

                copy = turn_line2_wide;
                copy.flip_vertical();
                grid[6][5] = std::move(copy);

                copy = turn_line3_wide;
                copy.flip_horizontal();
                copy.flip_vertical();
                grid[7][5] = std::move(copy);

                copy = turn;
                copy.flip_vertical();
                grid[8][5] = std::move(copy);

                copy = turn2;
                copy.flip_vertical();
                copy.rotate270();
                grid[9][5] = std::move(copy);

                copy = turn;
                copy.flip_vertical();
                grid[10][5] = std::move(copy);

                copy = line_dotted;
                copy.flip_vertical();
                grid[3][6] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                grid[4][6] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                grid[5][6] = std::move(copy);

                copy = turn_line;
                grid[6][6] = std::move(copy);

                copy = turn_line;
                copy.rotate90();
                grid[7][6] = std::move(copy);

                copy = turn_line;
                grid[8][6] = std::move(copy);

                copy = turn_line;
                copy.rotate90();
                grid[9][6] = std::move(copy);

                copy = turn_line2_wide;
                copy.flip_vertical();
                copy.rotate90();
                grid[10][6] = std::move(copy);

                copy = line_dotted;
                grid[3][7] = std::move(copy);

                copy = turn_line_wide;
                copy.flip_vertical();
                copy.rotate270();
                grid[4][7] = std::move(copy);

                copy = turn_line_wide;
                copy.flip_vertical();
                copy.rotate270();
                grid[5][7] = std::move(copy);

                copy = turn_line;
                copy.flip_vertical();
                grid[6][7] = std::move(copy);

                copy = turn_line;
                copy.flip_vertical();
                copy.rotate270();
                grid[7][7] = std::move(copy);

                copy = turn_line;
                copy.flip_vertical();
                grid[8][7] = std::move(copy);

                copy = turn_line;
                copy.flip_vertical();
                copy.rotate270();
                grid[9][7] = std::move(copy);

                copy = turn_line3_wide;
                copy.rotate270();
                grid[10][7] = std::move(copy);

                copy = turn;
                copy.flip_horizontal();
                grid[4][8] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                grid[5][8] = std::move(copy);

                copy = turn;
                grid[6][8] = std::move(copy);

                copy = turn2;
                copy.rotate90();
                grid[7][8] = std::move(copy);

                copy = turn_line2_wide;
                grid[8][8] = std::move(copy);

                copy = turn_line3_wide;
                copy.flip_horizontal();
                grid[9][8] = std::move(copy);

                copy = turn;
                grid[10][8] = std::move(copy);

                copy = turn_line_wide;
                copy.flip_horizontal();
                copy.rotate90();
                grid[5][9] = std::move(copy);

                copy = start;
                grid[4][10] = std::move(copy);

                copy = start;
                copy.flip_horizontal();
                grid[5][10] = std::move(copy);
            }

            void build_gates()
            {
                gates.push_back(Gate(321.0, 321.0));
                gates.push_back(Gate(385.0, 321.0));
                gates.push_back(Gate(512.0, 321.0));
            }
        };

        class Car
        {
        public:
            rsim::vmodel::State state;
            rsim::smodel::LineSensor line_sensor;

            Car(double x_, double y_, double orientation_) : state{x_, y_, orientation_}, line_sensor{state}
            {
            }

            void update(double wheel_angle, double velocity)
            {
                state.update(wheel_angle, velocity);
                line_sensor.update(state);
            }

            void detect(bool map[][960], unsigned long map_width, unsigned long map_height)
            {
                line_sensor.detect(map, map_width, map_height);
            }

        private:
        };

    } // namespace env

} // namespace rsim

#endif // ENVIRONMENT_HXX
