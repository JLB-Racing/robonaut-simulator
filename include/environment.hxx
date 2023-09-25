#ifndef ENVIRONMENT_HXX
#define ENVIRONMENT_HXX

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <bitset>
#include <chrono>

#include "vehicle_model.hxx"
#include "sensor_model.hxx"

namespace rsim
{

    namespace env
    {
        static constexpr int BITMAP_SIZE = 64;
        static constexpr int GRID_WIDTH = 17;
        static constexpr int GRID_HEIGHT = 16;
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

            std::chrono::steady_clock::time_point last_seen;
            std::chrono::steady_clock::time_point last_stolen;

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

                // iterate over grid and fill in data indexing is [col][row]
                for (int gcol = 0; gcol < GRID_WIDTH; gcol++)
                    for (int grow = 0; grow < GRID_HEIGHT; grow++)
                        for (int bcol = 0; bcol < BITMAP_SIZE; bcol++)
                            for (int brow = 0; brow < BITMAP_SIZE; brow++)
                                data[gcol * BITMAP_SIZE + bcol][grow * BITMAP_SIZE + brow] = grid[gcol][grow].bitmap[bcol][brow];
            }

        private:
            Bitmap grid[GRID_WIDTH][GRID_HEIGHT] = {{rsim::env::Bitmap{}}};
            Bitmap balancer = Bitmap("assets/balancer.bmp");
            Bitmap start = Bitmap("assets/start.bmp");
            Bitmap line = Bitmap("assets/line.bmp");
            Bitmap line_dotted = Bitmap("assets/line-dotted.bmp");
            Bitmap line_edge = Bitmap("assets/line-edge.bmp");
            Bitmap line_triple = Bitmap("assets/line-triple.bmp");
            Bitmap line_triple_half = Bitmap("assets/line-triple-half.bmp");
            Bitmap line_triple_half_dotted = Bitmap("assets/line-triple-half-dotted.bmp");
            Bitmap turn = Bitmap("assets/turn.bmp");
            Bitmap turn2 = Bitmap("assets/turn-2.bmp");
            Bitmap turn3 = Bitmap("assets/turn-3.bmp");
            Bitmap turn_t = Bitmap("assets/turn-t.bmp");
            Bitmap turn_half = Bitmap("assets/turn-half.bmp");
            Bitmap turn_half_full = Bitmap("assets/turn-half-full.bmp");
            Bitmap turn_line = Bitmap("assets/turn-line.bmp");
            Bitmap turn_line2 = Bitmap("assets/turn-line-2.bmp");
            Bitmap turn_line3 = Bitmap("assets/turn-line-3.bmp");
            Bitmap turn_line_wide = Bitmap("assets/turn-line-wide.bmp");
            Bitmap turn_line2_wide = Bitmap("assets/turn-line-2-wide.bmp");
            Bitmap turn_line3_wide = Bitmap("assets/turn-line-3-wide.bmp");

            void build_grid()
            {

                auto copy = Bitmap{};

                copy = turn_half;
                copy.flip_vertical();
                copy.flip_horizontal();
                grid[13][1] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[14][1] = std::move(copy);

                copy = turn_half;
                copy.flip_vertical();
                grid[15][1] = std::move(copy);

                copy = line;
                grid[13][2] = std::move(copy);

                copy = line_triple_half;
                copy.flip_vertical();
                grid[15][2] = std::move(copy);

                copy = turn_half;
                copy.flip_horizontal();
                copy.flip_vertical();
                grid[3][3] = std::move(copy);

                copy = line_triple_half;
                copy.rotate90();
                grid[4][3] = std::move(copy);

                copy = line_triple;
                copy.rotate90();
                grid[5][3] = std::move(copy);

                copy = line_triple_half;
                copy.rotate90();
                copy.flip_horizontal();
                grid[6][3] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[7][3] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[8][3] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[9][3] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[10][3] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[11][3] = std::move(copy);

                copy = line_triple_half_dotted;
                copy.rotate90();
                copy.flip_horizontal();
                grid[12][3] = std::move(copy);

                copy = turn_half;
                grid[13][3] = std::move(copy);

                copy = line_triple;
                grid[15][3] = std::move(copy);

                copy = line_triple_half_dotted;
                copy.flip_vertical();
                grid[3][4] = std::move(copy);

                copy = start;
                copy.flip_vertical();
                grid[4][4] = std::move(copy);

                copy = start;
                copy.flip_horizontal();
                copy.flip_vertical();
                grid[5][4] = std::move(copy);

                copy = line_triple_half;
                grid[15][4] = std::move(copy);

                copy = line;
                grid[3][5] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                grid[5][5] = std::move(copy);

                copy = line;
                grid[15][5] = std::move(copy);

                copy = turn_half_full;
                copy.rotate180();
                grid[1][6] = std::move(copy);

                copy = line_edge;
                copy.rotate90();
                grid[2][6] = std::move(copy);

                copy = turn_t;
                grid[3][6] = std::move(copy);

                copy = turn_line2_wide;
                copy.flip_horizontal();
                copy.flip_vertical();
                grid[4][6] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                copy.flip_vertical();
                grid[5][6] = std::move(copy);

                copy = turn_line2_wide;
                copy.flip_vertical();
                grid[6][6] = std::move(copy);

                copy = turn_line3_wide;
                copy.flip_horizontal();
                copy.flip_vertical();
                grid[7][6] = std::move(copy);

                copy = turn;
                copy.flip_vertical();
                grid[8][6] = std::move(copy);

                copy = turn2;
                copy.flip_vertical();
                copy.rotate270();
                grid[9][6] = std::move(copy);

                copy = turn;
                copy.flip_vertical();
                grid[10][6] = std::move(copy);

                copy = line;
                grid[15][6] = std::move(copy);

                copy = balancer;
                grid[1][7] = std::move(copy);

                copy = line_dotted;
                copy.flip_vertical();
                grid[3][7] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                grid[4][7] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                grid[5][7] = std::move(copy);

                copy = turn_line;
                grid[6][7] = std::move(copy);

                copy = turn_line;
                copy.rotate90();
                grid[7][7] = std::move(copy);

                copy = turn_line;
                grid[8][7] = std::move(copy);

                copy = turn_line;
                copy.rotate90();
                grid[9][7] = std::move(copy);

                copy = turn_line2_wide;
                copy.flip_vertical();
                copy.rotate90();
                grid[10][7] = std::move(copy);

                copy = line;
                grid[15][7] = std::move(copy);

                copy = balancer;
                grid[1][8] = std::move(copy);

                copy = line_dotted;
                grid[3][8] = std::move(copy);

                copy = turn_line_wide;
                copy.flip_vertical();
                copy.rotate270();
                grid[4][8] = std::move(copy);

                copy = turn_line_wide;
                copy.flip_vertical();
                copy.rotate270();
                grid[5][8] = std::move(copy);

                copy = turn_line;
                copy.flip_vertical();
                grid[6][8] = std::move(copy);

                copy = turn_line;
                copy.flip_vertical();
                copy.rotate270();
                grid[7][8] = std::move(copy);

                copy = turn_line;
                copy.flip_vertical();
                grid[8][8] = std::move(copy);

                copy = turn_line;
                copy.flip_vertical();
                copy.rotate270();
                grid[9][8] = std::move(copy);

                copy = turn_line3_wide;
                copy.rotate270();
                grid[10][8] = std::move(copy);

                copy = line;
                grid[15][8] = std::move(copy);

                copy = line_triple_half;
                copy.flip_vertical();
                grid[3][9] = std::move(copy);

                copy = turn;
                copy.flip_horizontal();
                grid[4][9] = std::move(copy);

                copy = turn_line_wide;
                copy.rotate90();
                grid[5][9] = std::move(copy);

                copy = turn;
                grid[6][9] = std::move(copy);

                copy = turn2;
                copy.rotate90();
                grid[7][9] = std::move(copy);

                copy = turn_line2_wide;
                grid[8][9] = std::move(copy);

                copy = turn_line3_wide;
                copy.flip_horizontal();
                grid[9][9] = std::move(copy);

                copy = turn;
                grid[10][9] = std::move(copy);

                copy = line;
                grid[15][9] = std::move(copy);

                copy = line_triple;
                grid[3][10] = std::move(copy);

                copy = turn_line_wide;
                copy.flip_horizontal();
                copy.rotate90();
                grid[5][10] = std::move(copy);

                copy = line;
                grid[15][10] = std::move(copy);

                copy = start;
                grid[4][11] = std::move(copy);

                copy = line_triple_half;
                grid[3][11] = std::move(copy);

                copy = start;
                copy.flip_horizontal();
                grid[5][11] = std::move(copy);

                copy = line;
                grid[15][11] = std::move(copy);

                copy = turn_half;
                copy.flip_horizontal();
                grid[3][12] = std::move(copy);

                copy = line_triple_half_dotted;
                copy.rotate90();
                grid[4][12] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[5][12] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[6][12] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[7][12] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[8][12] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[9][12] = std::move(copy);

                copy = line_triple_half;
                copy.rotate90();
                grid[10][12] = std::move(copy);

                copy = line_triple;
                copy.rotate90();
                grid[11][12] = std::move(copy);

                copy = line_triple_half;
                copy.rotate90();
                copy.flip_horizontal();
                grid[12][12] = std::move(copy);

                copy = turn_half;
                copy.flip_vertical();
                grid[13][12] = std::move(copy);

                copy = line;
                grid[15][12] = std::move(copy);

                copy = line;
                grid[13][13] = std::move(copy);

                copy = line_triple_half_dotted;
                grid[15][13] = std::move(copy);

                copy = turn_half;
                copy.flip_horizontal();
                grid[13][14] = std::move(copy);

                copy = line;
                copy.rotate90();
                grid[14][14] = std::move(copy);

                copy = turn_half;
                grid[15][14] = std::move(copy);
            }

            void build_gates()
            {
                // gates.push_back(Gate(320.0, 384.0));
                gates.push_back(Gate(384.0, 384.0));
                gates.push_back(Gate(512.0, 384.0));
                gates.push_back(Gate(640.0, 384.0));

                // gates.push_back(Gate(256.0, 448.0));
                gates.push_back(Gate(320.0, 448.0));
                gates.push_back(Gate(448.0, 448.0));
                gates.push_back(Gate(576.0, 448.0));
                gates.push_back(Gate(704.0, 448.0));

                // gates.push_back(Gate(320.0, 512.0));
                gates.push_back(Gate(384.0, 512.0));
                gates.push_back(Gate(512.0, 512.0));
                gates.push_back(Gate(640.0, 512.0));

                // gates.push_back(Gate(256.0, 576.0));
                gates.push_back(Gate(320.0, 576.0));
                gates.push_back(Gate(448.0, 576.0));
                gates.push_back(Gate(576.0, 576.0));
                gates.push_back(Gate(704.0, 576.0));

                gates.push_back(Gate(384.0, 640.0));
                gates.push_back(Gate(512.0, 640.0));
                gates.push_back(Gate(640.0, 640.0));

                gates.push_back(Gate(320.0, 704.0));
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

            template <size_t cols, size_t rows>
            void detect(bool (&map)[cols][rows])
            {
                line_sensor.detect(map);
            }

        private:
        };

    } // namespace env

} // namespace rsim

#endif // ENVIRONMENT_HXX
