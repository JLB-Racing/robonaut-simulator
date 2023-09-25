#ifndef PIRATE_MODEL_HXX
#define PIRATE_MODEL_HXX

#include <string>
#include <vector>
#include <tuple>
#include <numbers>
#define PI std::numbers::pi_v<float>

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

            void add_edge(char name_, Direction direction_, std::vector<char> prev_nodes_, float weight_ = 0.0)
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
                this->operator[]('A').add_edge('C', Direction::LEFT, {'B', 'D'}, PI);
                this->operator[]('A').add_edge('B', Direction::STRAIGHT, {'C'}, 4.0f);
                this->operator[]('A').add_edge('D', Direction::RIGHT, {'C'}, PI);
                this->operator[]('B').add_edge('A', Direction::STRAIGHT, {'E'}, 4.0f);
                this->operator[]('B').add_edge('E', Direction::RIGHT, {'A', 'D'}, PI);
                this->operator[]('B').add_edge('D', Direction::LEFT, {'E'}, PI);
                this->operator[]('C').add_edge('A', Direction::RIGHT, {'F'}, PI);
                this->operator[]('C').add_edge('F', Direction::LEFT, {'A'}, PI);
                this->operator[]('D').add_edge('A', Direction::LEFT, {'F', 'G', 'I'}, PI);
                this->operator[]('D').add_edge('B', Direction::RIGHT, {'F', 'G', 'I'}, PI);
                this->operator[]('D').add_edge('G', Direction::LEFT, {'A', 'B'}, PI);
                this->operator[]('D').add_edge('I', Direction::STRAIGHT, {'A', 'B'}, 4.0f);
                this->operator[]('D').add_edge('F', Direction::RIGHT, {'A', 'B'}, PI);
                this->operator[]('E').add_edge('B', Direction::LEFT, {'G', 'J'}, PI);
                this->operator[]('E').add_edge('J', Direction::STRAIGHT, {'B'}, 4.0f);
                this->operator[]('E').add_edge('G', Direction::RIGHT, {'B'}, PI);
                this->operator[]('F').add_edge('C', Direction::RIGHT, {'D', 'G', 'I'}, PI);
                this->operator[]('F').add_edge('D', Direction::LEFT, {'C', 'H'}, PI);
                this->operator[]('F').add_edge('G', Direction::STRAIGHT, {'C', 'H'}, 4.0f);
                this->operator[]('F').add_edge('I', Direction::RIGHT, {'C', 'H'}, PI);
                this->operator[]('F').add_edge('H', Direction::LEFT, {'D', 'G', 'I'}, PI);
                this->operator[]('G').add_edge('F', Direction::STRAIGHT, {'E', 'J'}, 4.0f);
                this->operator[]('G').add_edge('D', Direction::RIGHT, {'E', 'J'}, PI);
                this->operator[]('G').add_edge('E', Direction::LEFT, {'D', 'F', 'I'}, PI);
                this->operator[]('G').add_edge('J', Direction::RIGHT, {'D', 'F', 'I'}, PI);
                this->operator[]('G').add_edge('I', Direction::LEFT, {'E', 'J'}, PI);
                this->operator[]('H').add_edge('F', Direction::RIGHT, {'K', 'M'}, PI);
                this->operator[]('H').add_edge('K', Direction::LEFT, {'F'}, PI);
                this->operator[]('H').add_edge('M', Direction::STRAIGHT, {'F'}, 4.0f);
                this->operator[]('I').add_edge('F', Direction::LEFT, {'K', 'L', 'N'}, PI);
                this->operator[]('I').add_edge('D', Direction::STRAIGHT, {'K', 'L', 'N'}, 4.0f);
                this->operator[]('I').add_edge('G', Direction::RIGHT, {'K', 'L', 'N'}, PI);
                this->operator[]('I').add_edge('L', Direction::LEFT, {'D', 'F', 'G'}, PI);
                this->operator[]('I').add_edge('N', Direction::STRAIGHT, {'D', 'F', 'G'}, 4.0f);
                this->operator[]('I').add_edge('K', Direction::RIGHT, {'D', 'F', 'G'}, PI);
                this->operator[]('J').add_edge('G', Direction::LEFT, {'L'}, PI);
                this->operator[]('J').add_edge('E', Direction::STRAIGHT, {'L'}, 4.0f);
                this->operator[]('J').add_edge('L', Direction::RIGHT, {'E', 'G'}, PI);
                this->operator[]('K').add_edge('H', Direction::RIGHT, {'I', 'L', 'N'}, PI);
                this->operator[]('K').add_edge('I', Direction::LEFT, {'H', 'M'}, PI);
                this->operator[]('K').add_edge('L', Direction::STRAIGHT, {'H', 'M'}, 4.0f);
                this->operator[]('K').add_edge('N', Direction::RIGHT, {'H', 'M'}, PI);
                this->operator[]('K').add_edge('M', Direction::LEFT, {'I', 'L', 'N'}, PI);
                this->operator[]('L').add_edge('K', Direction::STRAIGHT, {'J', 'O'}, 4.0f);
                this->operator[]('L').add_edge('I', Direction::RIGHT, {'J', 'O'}, PI);
                this->operator[]('L').add_edge('J', Direction::LEFT, {'I', 'K', 'N'}, PI);
                this->operator[]('L').add_edge('O', Direction::RIGHT, {'I', 'K', 'N'}, PI);
                this->operator[]('L').add_edge('N', Direction::LEFT, {'J', 'O'}, PI);
                this->operator[]('M').add_edge('H', Direction::STRAIGHT, {'P', 'Q', 'R'}, 4.0f);
                this->operator[]('M').add_edge('K', Direction::RIGHT, {'P', 'Q', 'R'}, PI);
                this->operator[]('M').add_edge('R', Direction::LEFT, {'H', 'K'}, PI);
                this->operator[]('M').add_edge('Q', Direction::STRAIGHT, {'H', 'K'}, 2.0f);
                this->operator[]('M').add_edge('P', Direction::RIGHT, {'H', 'K'}, PI);
                this->operator[]('N').add_edge('K', Direction::LEFT, {'R', 'S', 'T'}, PI);
                this->operator[]('N').add_edge('I', Direction::STRAIGHT, {'R', 'S', 'T'}, 4.0f);
                this->operator[]('N').add_edge('L', Direction::RIGHT, {'R', 'S', 'T'}, PI);
                this->operator[]('N').add_edge('T', Direction::LEFT, {'K', 'I', 'L'}, PI);
                this->operator[]('N').add_edge('S', Direction::STRAIGHT, {'K', 'I', 'L'}, 2.0f);
                this->operator[]('N').add_edge('R', Direction::RIGHT, {'K', 'I', 'L'}, PI);
                this->operator[]('O').add_edge('L', Direction::LEFT, {'T', 'U', 'W'}, PI);
                this->operator[]('O').add_edge('U', Direction::LEFT, {'L'}, PI);
                this->operator[]('O').add_edge('W', Direction::STRAIGHT, {'L'}, 2.0f + PI);
                this->operator[]('O').add_edge('T', Direction::RIGHT, {'L'}, PI);
                this->operator[]('P').add_edge('M', Direction::LEFT, {'P'}, PI);
                this->operator[]('P').add_edge('Q', Direction::STRAIGHT, {'P'}, 2.0f);
                this->operator[]('Q').add_edge('P', Direction::STRAIGHT, {'R'}, 2.0f);
                this->operator[]('Q').add_edge('M', Direction::STRAIGHT, {'V', 'X'}, 2.0f);
                this->operator[]('Q').add_edge('R', Direction::STRAIGHT, {'P'}, 2.0f);
                this->operator[]('Q').add_edge('V', Direction::LEFT, {'M'}, PI);
                this->operator[]('Q').add_edge('X', Direction::STRAIGHT, {'M'}, 5.0f + PI);
                this->operator[]('R').add_edge('Q', Direction::STRAIGHT, {'N', 'S'}, 2.0f);
                this->operator[]('R').add_edge('M', Direction::RIGHT, {'N', 'S'}, PI);
                this->operator[]('R').add_edge('N', Direction::LEFT, {'M', 'Q'}, PI);
                this->operator[]('R').add_edge('S', Direction::STRAIGHT, {'M', 'Q'}, 2.0f);
                this->operator[]('S').add_edge('R', Direction::STRAIGHT, {'T'}, 2.0f);
                this->operator[]('S').add_edge('N', Direction::STRAIGHT, {'V', 'W'}, 2.0f);
                this->operator[]('S').add_edge('T', Direction::STRAIGHT, {'R'}, 2.0f);
                this->operator[]('S').add_edge('W', Direction::LEFT, {'N'}, PI);
                this->operator[]('S').add_edge('V', Direction::RIGHT, {'N'}, PI);
                this->operator[]('T').add_edge('S', Direction::STRAIGHT, {'O', 'U'}, 2.0f);
                this->operator[]('T').add_edge('N', Direction::RIGHT, {'O', 'U'}, PI);
                this->operator[]('T').add_edge('O', Direction::LEFT, {'N', 'S'}, PI);
                this->operator[]('T').add_edge('U', Direction::STRAIGHT, {'N', 'S'}, 4.0f);
                this->operator[]('U').add_edge('T', Direction::STRAIGHT, {'U'}, 4.0f);
                this->operator[]('U').add_edge('O', Direction::RIGHT, {'U'}, PI);
                this->operator[]('V').add_edge('Q', Direction::RIGHT, {'S', 'W'}, PI);
                this->operator[]('V').add_edge('S', Direction::LEFT, {'Q'}, PI);
                this->operator[]('V').add_edge('W', Direction::STRAIGHT, {'Q'}, 4.0f);
                this->operator[]('W').add_edge('V', Direction::STRAIGHT, {'O'}, 4.0f);
                this->operator[]('W').add_edge('S', Direction::RIGHT, {'O'}, PI);
                this->operator[]('W').add_edge('O', Direction::STRAIGHT, {'S', 'V'}, 2.0f + PI);
                this->operator[]('X').add_edge('Q', Direction::STRAIGHT, {}, 5.0f + PI);
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
            static constexpr double DELTA_T = 0.1;
            static constexpr double Kp = 0.8;
            static constexpr double Ki = 0.01;
            static constexpr double Kd = 0.6;
            static constexpr double SPEED = 5.0;

            unsigned long selected = 0;
            double target_angle = 0.0;
            double target_speed = 0.0;

            Direction direction = Direction::LEFT;

            PirateController()
            {
                current_node = 'P';
                next_node = 'Q';
            }

            ~PirateController() {}

            double PID(const double error)
            {
                double proportional_term = Kp * error;
                integral += Ki * error * DELTA_T;
                double derivative_term = Kd * (error - prev_error) / DELTA_T;
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

                double error = (static_cast<int>(selected - sensor_center)) / static_cast<double>(sensor_center);
                target_angle = PID(error);

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
            double integral = 0.0;
            double prev_error = 0.0;
            bool prev_under_gate = false;

            Graph graph;
            // char prev_node;
            char current_node;
            char next_node;
        };
    } // namespace logic

} // namespace rsim

#endif // PIRATE_MODEL_HXX