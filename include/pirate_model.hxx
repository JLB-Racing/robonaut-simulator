#ifndef PIRATE_MODEL_HXX
#define PIRATE_MODEL_HXX

#include <string>
#include <vector>
#include <tuple>

namespace rsim
{
    namespace logic
    {
        enum class Direction
        {
            LEFT,
            RIGHT,
            STRAIGHT,
        };

        struct Edge
        {
            char node;
            Direction direction;
            std::vector<char> prev_nodes;
            float weight;
        };

        class Node
        {
        public:
            char name;
            std::vector<Edge> edges;

            Node(char name_) : name{name_} {}
            ~Node() {}

            void add_edge(char name_, Direction direction_, std::vector<char> prev_nodes_, float weight_ = 0.0f)
            {
                edges.push_back(Edge{name_, direction_, prev_nodes_, weight_});
            }
        };

        class Graph
        {
        public:
            Graph()
            {
                // add nodes A - X
                for (int i = 0; i < 24; i++)
                {
                    nodes.push_back(Node{static_cast<char>('A' + i)});
                }

                // add edges
                this->operator[]('A').add_edge('C', Direction::LEFT, {'B', 'D'}, M_PI);
                this->operator[]('A').add_edge('B', Direction::STRAIGHT, {'C'}, 4.0f);
                this->operator[]('A').add_edge('D', Direction::RIGHT, {'C'}, M_PI);
                this->operator[]('B').add_edge('A', Direction::STRAIGHT, {'E'}, 4.0f);
                this->operator[]('B').add_edge('E', Direction::RIGHT, {'A', 'D'}, M_PI);
                this->operator[]('B').add_edge('D', Direction::LEFT, {'E'}, M_PI);
                this->operator[]('C').add_edge('A', Direction::RIGHT, {'F'}, M_PI);
                this->operator[]('C').add_edge('F', Direction::LEFT, {'A'}, M_PI);
                this->operator[]('D').add_edge('A', Direction::LEFT, {'F', 'G', 'I'}, M_PI);
                this->operator[]('D').add_edge('B', Direction::RIGHT, {'F', 'G', 'I'}, M_PI);
                this->operator[]('D').add_edge('G', Direction::LEFT, {'A', 'B'}, M_PI);
                this->operator[]('D').add_edge('I', Direction::STRAIGHT, {'A', 'B'}, 4.0f);
                this->operator[]('D').add_edge('F', Direction::RIGHT, {'A', 'B'}, M_PI);
                this->operator[]('E').add_edge('B', Direction::LEFT, {'G', 'J'}, M_PI);
                this->operator[]('E').add_edge('J', Direction::STRAIGHT, {'B'}, 4.0f);
                this->operator[]('E').add_edge('G', Direction::RIGHT, {'B'}, M_PI);
                this->operator[]('F').add_edge('C', Direction::RIGHT, {'D', 'G', 'I'}, M_PI);
                this->operator[]('F').add_edge('D', Direction::LEFT, {'C', 'H'}, M_PI);
                this->operator[]('F').add_edge('G', Direction::STRAIGHT, {'C', 'H'}, 4.0f);
                this->operator[]('F').add_edge('I', Direction::RIGHT, {'C', 'H'}, M_PI);
                this->operator[]('F').add_edge('H', Direction::LEFT, {'D', 'G', 'I'}, M_PI);
                this->operator[]('G').add_edge('F', Direction::STRAIGHT, {'E', 'J'}, 4.0f);
                this->operator[]('G').add_edge('D', Direction::RIGHT, {'E', 'J'}, M_PI);
                this->operator[]('G').add_edge('E', Direction::LEFT, {'D', 'F', 'I'}, M_PI);
                this->operator[]('G').add_edge('J', Direction::RIGHT, {'D', 'F', 'I'}, M_PI);
                this->operator[]('G').add_edge('I', Direction::LEFT, {'E', 'J'}, M_PI);
                this->operator[]('H').add_edge('F', Direction::RIGHT, {'K', 'M'}, M_PI);
                this->operator[]('H').add_edge('K', Direction::LEFT, {'F'}, M_PI);
                this->operator[]('H').add_edge('M', Direction::STRAIGHT, {'F'}, 4.0f);
                this->operator[]('I').add_edge('F', Direction::LEFT, {'K', 'L', 'N'}, M_PI);
                this->operator[]('I').add_edge('D', Direction::STRAIGHT, {'K', 'L', 'N'}, 4.0f);
                this->operator[]('I').add_edge('G', Direction::RIGHT, {'K', 'L', 'N'}, M_PI);
                this->operator[]('I').add_edge('L', Direction::LEFT, {'D', 'F', 'G'}, M_PI);
                this->operator[]('I').add_edge('N', Direction::STRAIGHT, {'D', 'F', 'G'}, 4.0f);
                this->operator[]('I').add_edge('K', Direction::RIGHT, {'D', 'F', 'G'}, M_PI);
                this->operator[]('J').add_edge('G', Direction::LEFT, {'L'}, M_PI);
                this->operator[]('J').add_edge('E', Direction::STRAIGHT, {'L'}, 4.0f);
                this->operator[]('J').add_edge('L', Direction::RIGHT, {'E', 'G'}, M_PI);
                this->operator[]('K').add_edge('H', Direction::RIGHT, {'I', 'L', 'N'}, M_PI);
                this->operator[]('K').add_edge('I', Direction::LEFT, {'H', 'M'}, M_PI);
                this->operator[]('K').add_edge('L', Direction::STRAIGHT, {'H', 'M'}, 4.0f);
                this->operator[]('K').add_edge('N', Direction::RIGHT, {'H', 'M'}, M_PI);
                this->operator[]('K').add_edge('M', Direction::LEFT, {'I', 'L', 'N'}, M_PI);
                this->operator[]('L').add_edge('K', Direction::STRAIGHT, {'J', 'O'}, 4.0f);
                this->operator[]('L').add_edge('I', Direction::RIGHT, {'J', 'O'}, M_PI);
                this->operator[]('L').add_edge('J', Direction::LEFT, {'I', 'K', 'N'}, M_PI);
                this->operator[]('L').add_edge('O', Direction::RIGHT, {'I', 'K', 'N'}, M_PI);
                this->operator[]('L').add_edge('N', Direction::LEFT, {'J', 'O'}, M_PI);
                this->operator[]('M').add_edge('H', Direction::STRAIGHT, {'P', 'Q', 'R'}, 4.0f);
                this->operator[]('M').add_edge('K', Direction::RIGHT, {'P', 'Q', 'R'}, M_PI);
                this->operator[]('M').add_edge('R', Direction::LEFT, {'H', 'K'}, M_PI);
                this->operator[]('M').add_edge('Q', Direction::STRAIGHT, {'H', 'K'}, 2.0f);
                this->operator[]('M').add_edge('P', Direction::RIGHT, {'H', 'K'}, M_PI);
                this->operator[]('N').add_edge('K', Direction::LEFT, {'R', 'S', 'T'}, M_PI);
                this->operator[]('N').add_edge('I', Direction::STRAIGHT, {'R', 'S', 'T'}, 4.0f);
                this->operator[]('N').add_edge('L', Direction::RIGHT, {'R', 'S', 'T'}, M_PI);
                this->operator[]('N').add_edge('T', Direction::LEFT, {'K', 'I', 'L'}, M_PI);
                this->operator[]('N').add_edge('S', Direction::STRAIGHT, {'K', 'I', 'L'}, 2.0f);
                this->operator[]('N').add_edge('R', Direction::RIGHT, {'K', 'I', 'L'}, M_PI);
                this->operator[]('O').add_edge('L', Direction::LEFT, {'T', 'U', 'W'}, M_PI);
                this->operator[]('O').add_edge('U', Direction::LEFT, {'L'}, M_PI);
                this->operator[]('O').add_edge('W', Direction::STRAIGHT, {'L'}, 2.0f + M_PI);
                this->operator[]('O').add_edge('T', Direction::RIGHT, {'L'}, M_PI);
                this->operator[]('P').add_edge('M', Direction::LEFT, {'P'}, M_PI);
                this->operator[]('P').add_edge('Q', Direction::STRAIGHT, {'P'}, 2.0f);
                this->operator[]('Q').add_edge('P', Direction::STRAIGHT, {'R'}, 2.0f);
                this->operator[]('Q').add_edge('M', Direction::STRAIGHT, {'V', 'X'}, 2.0f);
                this->operator[]('Q').add_edge('R', Direction::STRAIGHT, {'P'}, 2.0f);
                this->operator[]('Q').add_edge('V', Direction::LEFT, {'M'}, M_PI);
                this->operator[]('Q').add_edge('X', Direction::STRAIGHT, {'M'}, 5.0f + M_PI);
                this->operator[]('R').add_edge('Q', Direction::STRAIGHT, {'N', 'S'}, 2.0f);
                this->operator[]('R').add_edge('M', Direction::RIGHT, {'N', 'S'}, M_PI);
                this->operator[]('R').add_edge('N', Direction::LEFT, {'M', 'Q'}, M_PI);
                this->operator[]('R').add_edge('S', Direction::STRAIGHT, {'M', 'Q'}, 2.0f);
                this->operator[]('S').add_edge('R', Direction::STRAIGHT, {'T'}, 2.0f);
                this->operator[]('S').add_edge('N', Direction::STRAIGHT, {'V', 'W'}, 2.0f);
                this->operator[]('S').add_edge('T', Direction::STRAIGHT, {'R'}, 2.0f);
                this->operator[]('S').add_edge('W', Direction::LEFT, {'N'}, M_PI);
                this->operator[]('S').add_edge('V', Direction::RIGHT, {'N'}, M_PI);
                this->operator[]('T').add_edge('S', Direction::STRAIGHT, {'O', 'U'}, 2.0f);
                this->operator[]('T').add_edge('N', Direction::RIGHT, {'O', 'U'}, M_PI);
                this->operator[]('T').add_edge('O', Direction::LEFT, {'N', 'S'}, M_PI);
                this->operator[]('T').add_edge('U', Direction::STRAIGHT, {'N', 'S'}, 4.0f);
                this->operator[]('U').add_edge('T', Direction::STRAIGHT, {'U'}, 4.0f);
                this->operator[]('U').add_edge('O', Direction::RIGHT, {'U'}, M_PI);
                this->operator[]('V').add_edge('Q', Direction::RIGHT, {'S', 'W'}, M_PI);
                this->operator[]('V').add_edge('S', Direction::LEFT, {'Q'}, M_PI);
                this->operator[]('V').add_edge('W', Direction::STRAIGHT, {'Q'}, 4.0f);
                this->operator[]('W').add_edge('V', Direction::STRAIGHT, {'O'}, 4.0f);
                this->operator[]('W').add_edge('S', Direction::RIGHT, {'O'}, M_PI);
                this->operator[]('W').add_edge('O', Direction::STRAIGHT, {'S', 'V'}, 2.0f + M_PI);
                this->operator[]('X').add_edge('Q', Direction::STRAIGHT, {}, 5.0f + M_PI);
            }

            ~Graph() {}

            // Node& get_node(std::string name)
            Node &operator[](char name)
            {
                if (nodes.empty())
                    throw std::runtime_error("Graph is empty");

                if (name < 'A' || name > 'X')
                    throw std::runtime_error("Invalid node name");

                return nodes[static_cast<int>(name - 'A')];
            }

        private:
            std::vector<Node> nodes;
        };

        class PirateController
        {
        public:
            static constexpr float Kp = 0.8f;
            static constexpr float Ki = 0.01f;
            static constexpr float Kd = 0.6f;
            static constexpr float SPEED = 25.0f;

            unsigned long selected = 0;
            float target_angle = 0.0f;
            float target_speed = 0.0f;

            Direction direction = Direction::LEFT;

            PirateController()
            {
                current_node = 'P';
                next_node = 'Q';
            }

            ~PirateController() {}

            float PID(const float error, const float dt)
            {
                float proportional_term = Kp * error;
                integral += Ki * error * dt;
                float derivative_term = Kd * (error - prev_error) / dt;
                return proportional_term + integral + derivative_term;
            }

            template <size_t cols>
            void lateral_control(bool (&detection_)[cols])
            {
                if (std::all_of(std::begin(detection_), std::end(detection_), [](bool b)
                                { return b; }))
                {
                    return;
                }

                auto control_timestamp_ = std::chrono::steady_clock::now();
                float dt = std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
                prev_control_timestamp_ = control_timestamp_;

                unsigned long sensor_center = cols / 2;

                unsigned long rightmost = 0;
                for (unsigned long i = 0; i < cols; i++)
                    if (!detection_[i] && i > rightmost)
                        rightmost = i;

                unsigned long leftmost = cols;
                for (unsigned long i = 0; i < cols; i++)
                    if (!detection_[i] && i < leftmost)
                        leftmost = i;

                // unsigned long center = leftmost;
                // for (unsigned long i = leftmost; i <= rightmost; i++)
                //     if (!detection_[i] && std::abs(static_cast<int>(i - (rightmost + leftmost) / 2)) < std::abs(static_cast<int>(center - (rightmost + leftmost) / 2)))
                //         center = i;

                // instead find the closest to the sensor center
                unsigned long center = leftmost;
                for (unsigned long i = leftmost; i <= rightmost; i++)
                    if (!detection_[i] && std::abs(static_cast<int>(i - sensor_center)) < std::abs(static_cast<int>(center - sensor_center)))
                        center = i;

                if (direction == Direction::LEFT)
                    selected = leftmost;
                if (direction == Direction::RIGHT)
                    selected = rightmost;
                if (direction == Direction::STRAIGHT)
                    selected = center;

                float error = (static_cast<int>(selected - sensor_center)) / static_cast<float>(sensor_center);
                target_angle = PID(error, dt);

                prev_error = error;
            }

            void longitudinal_control()
            {
                target_speed = SPEED;
            }

            template <size_t cols>
            void update(bool (&detection_)[cols], bool under_gate_)
            {
                if (!prev_under_gate && under_gate_)
                {
                    srand(time(NULL));
                    direction = static_cast<Direction>(rand() % 3);
                }

                lateral_control(detection_);
                longitudinal_control();

                prev_under_gate = under_gate_;
            }

        private:
            float integral = 0.0f;
            float prev_error = 0.0f;
            bool prev_under_gate = false;

            std::chrono::time_point<std::chrono::steady_clock> prev_control_timestamp_ = std::chrono::steady_clock::now();

            Graph graph;
            // char prev_node;
            char current_node;
            char next_node;
        };
    } // namespace logic

} // namespace rsim

#endif // PIRATE_MODEL_HXX