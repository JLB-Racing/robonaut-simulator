#ifndef ENVIRONMENT_HXX
#define ENVIRONMENT_HXX

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <bitset>
#include <chrono>
#include <filesystem>

#include "vehicle_model.hxx"
#include "sensor_model.hxx"

namespace rsim
{
    namespace env
    {
        class Bitmap
        {
        public:
            bool bitmap[BITMAP_SIZE][BITMAP_SIZE];

            Bitmap()
            {
                for (unsigned i = 0; i < BITMAP_SIZE; i++)
                {
                    for (unsigned j = 0; j < BITMAP_SIZE; j++)
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
                for (unsigned i = 0; i < BITMAP_SIZE; i++)
                {
                    for (unsigned j = 0; j < BITMAP_SIZE; j++)
                    {
                        rotated[i][j] = bitmap[j][i];
                    }
                }

                // reversing each row
                for (unsigned i = 0; i < BITMAP_SIZE; i++)
                {
                    for (unsigned j = 0, k = BITMAP_SIZE - 1; j < k; j++, k--)
                    {
                        std::swap(rotated[j][i], rotated[k][i]);
                    }
                }

                for (unsigned i = 0; i < BITMAP_SIZE; i++)
                {
                    for (unsigned j = 0; j < BITMAP_SIZE; j++)
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
                for (unsigned i = 0; i < BITMAP_SIZE; i++)
                {
                    for (unsigned j = 0, k = BITMAP_SIZE - 1; j < k; j++, k--)
                    {
                        std::swap(bitmap[j][i], bitmap[k][i]);
                    }
                }
            }

            void flip_vertical()
            {
                for (unsigned i = 0; i < BITMAP_SIZE; i++)
                {
                    for (unsigned j = 0, k = BITMAP_SIZE - 1; j < k; j++, k--)
                    {
                        std::swap(bitmap[i][j], bitmap[i][k]);
                    }
                }
            }

            friend std::ostream &operator<<(std::ostream &os, const Bitmap &bitmap)
            {
                for (unsigned i = 0; i < BITMAP_SIZE; i++)
                {
                    for (unsigned j = 0; j < BITMAP_SIZE; j++)
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
            float x;
            float y;

            std::chrono::steady_clock::time_point last_seen;
            std::chrono::steady_clock::time_point last_stolen;

            Gate(float x_, float y_) : x{x_}, y{y_} {}
        };

        class CrossSection
        {
        public:
            float x;
            float y;

            CrossSection(float x_, float y_) : x{x_}, y{y_} {}
        };

        class Map
        {
        public:
            bool data[MAP_WIDTH][MAP_HEIGHT];
            std::vector<Gate> gates;
            std::vector<CrossSection> cross_sections;

            Map()
            {
                for (unsigned long col = 0; col < MAP_WIDTH; col++)
                    for (unsigned long row = 0; row < MAP_HEIGHT; row++)
                        data[col][row] = true;

                build_grid_R();
                build_gates_R();
                build_cross_sections_R();

                for (unsigned gcol = 0; gcol < GRID_WIDTH; gcol++)
                    for (unsigned grow = 0; grow < GRID_HEIGHT; grow++)
                        for (unsigned bcol = 0; bcol < BITMAP_SIZE; bcol++)
                            for (unsigned brow = 0; brow < BITMAP_SIZE; brow++)
                                data[gcol * BITMAP_SIZE + bcol][grow * BITMAP_SIZE + brow] = grid[gcol][grow].bitmap[bcol][brow];

                serialize_map("competition.map");
            }

        private:
            Bitmap grid[GRID_WIDTH][GRID_HEIGHT] = {{env::Bitmap{}}};
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

            void serialize_map(const std::string &filename)
            {
                // # The map data, in row-major order, starting with (0,0).
                // # Cell (1, 0) will be listed second, representing the next cell in the x direction.
                // # Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
                // # The values inside are application dependent, but frequently,
                // # 0 represents unoccupied, 1 represents definitely occupied, and
                // # -1 represents unknown.
                // int8[] data

                // serialize the map into the above format:
                std::ofstream map_file;
                // with preferred separator
                auto file_path = "maps" + std::string{std::filesystem::path::preferred_separator} + filename;
                map_file.open(file_path);
                map_file << MAP_WIDTH << std::endl;
                map_file << MAP_HEIGHT << std::endl;
                for (unsigned long row = 0; row < MAP_HEIGHT; row++)
                {
                    for (unsigned long col = 0; col < MAP_WIDTH; col++)
                    {
                        if (data[col][row])
                            map_file << "0";
                        else
                            map_file << "1";
                    }
                }

                map_file.close();
            }

            void build_grid_R()
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

                copy = turn_line2_wide;
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

                copy = turn_line2_wide;
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

            void build_gates_R()
            {
                gates.push_back(Gate(384.0f, 384.0f));
                gates.push_back(Gate(512.0f, 384.0f));
                gates.push_back(Gate(640.0f, 384.0f));

                gates.push_back(Gate(320.0f, 448.0f));
                gates.push_back(Gate(448.0f, 448.0f));
                gates.push_back(Gate(576.0f, 448.0f));
                gates.push_back(Gate(704.0f, 448.0f));

                gates.push_back(Gate(384.0f, 512.0f));
                gates.push_back(Gate(512.0f, 512.0f));
                gates.push_back(Gate(640.0f, 512.0f));

                gates.push_back(Gate(320.0f, 576.0f));
                gates.push_back(Gate(448.0f, 576.0f));
                gates.push_back(Gate(576.0f, 576.0f));
                gates.push_back(Gate(704.0f, 576.0f));

                gates.push_back(Gate(384.0f, 640.0f));
                gates.push_back(Gate(512.0f, 640.0f));
                gates.push_back(Gate(640.0f, 640.0f));
            }

            void build_cross_sections_R()
            {
                cross_sections.push_back(CrossSection(320.0f, 320.0f));

                cross_sections.push_back(CrossSection(320.0f, 384.0f));

                cross_sections.push_back(CrossSection(256.0f, 448.0f));

                cross_sections.push_back(CrossSection(320.0f, 512.0f));
                cross_sections.push_back(CrossSection(448.0f, 512.0f));
                cross_sections.push_back(CrossSection(576.0f, 512.0f));

                cross_sections.push_back(CrossSection(256.0f, 576.0f));

                cross_sections.push_back(CrossSection(320.0f, 640.0f));

                cross_sections.push_back(CrossSection(320.0f, 704.0f));
            }
        };

        class Car
        {
        public:
            vmodel::State state;
            smodel::LineSensor line_sensor_front;
            smodel::LineSensor line_sensor_rear;
            smodel::ObjectSensor object_sensor;

            Car(float x_, float y_, float orientation_) : state{x_, y_, orientation_}, line_sensor_front{state}, line_sensor_rear{state}, object_sensor{state}
            {
            }

            void update(float wheel_angle, float velocity)
            {
                auto update_timestamp_ = std::chrono::steady_clock::now();
                float dt = std::chrono::duration_cast<std::chrono::milliseconds>(update_timestamp_ - prev_update_timestamp_).count() / 1000.0f;
                prev_update_timestamp_ = update_timestamp_;

                state.update(wheel_angle, velocity, dt);
                line_sensor_front.update(state);
                line_sensor_rear.update(state);
                object_sensor.update(state);
            }

            template <size_t cols, size_t rows>
            smodel::SensorDetection detect_front(bool (&map)[cols][rows])
            {
                auto detection = line_sensor_front.detect(map, 8);
                for (auto &line : detection.line_positions)
                    line = px_to_m(line);
                return detection;
            }

            template <size_t cols, size_t rows>
            smodel::SensorDetection detect_rear(bool (&map)[cols][rows])
            {
                auto detection = line_sensor_rear.detect(map, -7);
                for (auto &line : detection.line_positions)
                    line = px_to_m(line);
                return detection;
            }

            float detect_object(vmodel::State object_)
            {
                auto distance = px_to_m(object_sensor.detect(object_));
                if (distance > 2.0f)
                    distance = 2.0f;
                return distance;
            }

            float noisy_motor_rpm()
            {
                return state.noisy_motor_rpm();
            }

            float noisy_yaw_rate()
            {
                return state.noisy_yaw_rate();
            }

        private:
            std::chrono::time_point<std::chrono::steady_clock> prev_update_timestamp_ = std::chrono::steady_clock::now();
        };

    } // namespace env

} // namespace rsim

#endif // ENVIRONMENT_HXX
