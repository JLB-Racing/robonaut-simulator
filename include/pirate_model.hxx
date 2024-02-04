#ifndef PIRATE_MODEL_HXX
#define PIRATE_MODEL_HXX

#include <algorithm>
#include <string>
#include <tuple>
#include <vector>

#include "utility.hxx"

namespace rsim
{
    namespace pmodel
    {

        struct PirateInterface
        {
            char previous_node;
            char next_node;
            char after_next_node;
            int  section_percentage;
        };

        enum class Direction
        {
            LEFT,
            RIGHT,
            STRAIGHT,
        };

        std::ostream &operator<<(std::ostream &os, const Direction &direction)
        {
            switch (direction)
            {
                case Direction::LEFT:
                    os << "L";
                    break;
                case Direction::RIGHT:
                    os << "R";
                    break;
                case Direction::STRAIGHT:
                    os << "S";
                    break;
                default:
                    os << "-";
                    break;
            }
            return os;
        }

        struct Edge
        {
            char              node;
            Direction         direction;
            std::vector<char> prev_nodes;
            float             distance;
        };

        class Node
        {
        public:
            char              name;
            float             x;
            float             y;
            std::vector<Edge> edges;

            Node(char name_, float x_, float y_) : name{name_}, x(x_), y(y_) {}
            ~Node() {}

            void add_edge(char name_, Direction direction_, std::vector<char> prev_nodes_, float weight_ = 0.0f)
            {
                edges.push_back(Edge{name_, direction_, prev_nodes_, weight_});
            }
        };

        class Graph
        {
        public:
            std::vector<Node> nodes;
            Node              invalid_node = Node{'@', 0.0f, 0.0f};

            Graph()
            {
#ifdef Q2
                nodes.push_back(Node{static_cast<char>('A'), 128.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('B'), 192.0f, 192.0f});
                nodes.push_back(Node{static_cast<char>('C'), 256.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('D'), 320.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('E'), 320.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('F'), 448.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('G'), 448.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('H'), 576.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('I'), 576.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('J'), 704.0f, 192.0f});
                nodes.push_back(Node{static_cast<char>('K'), 832.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('L'), 832.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('M'), 960.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('N'), 960.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('O'), 1088.0f, 192.0f});
                nodes.push_back(Node{static_cast<char>('P'), 1216.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('Q'), 1216.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('R'), 1344.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('S'), 1344.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('T'), 1472.0f, 192.0f});
                nodes.push_back(Node{static_cast<char>('U'), 1600.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('V'), 1600.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('W'), 1664.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('X'), 1664.0f, 256.0f});
                nodes.push_back(Node{static_cast<char>('Y'), 1792.0f, 128.0f});
                nodes.push_back(Node{static_cast<char>('Z'), 1792.0f, 224.0f});
                nodes.push_back(Node{static_cast<char>('['), 1920.0f, 224.0f});

                const auto UNIT            = m_to_px(SQUARE_LENGTH * 2.0f);
                const auto QUARTER_CIRCLE  = m_to_px(2 * UNIT * M_PI / 4.0f);
                const auto CROSS_UNIT      = m_to_px(3.65f);
                const auto HALF_CROSS_UNIT = m_to_px(2.7f);

                this->operator[]('A').add_edge('C', Direction::RIGHT, {'A'}, 2.0f * UNIT);
                this->operator[]('B').add_edge('C', Direction::LEFT, {'D'}, QUARTER_CIRCLE);
                this->operator[]('B').add_edge('D', Direction::STRAIGHT, {'C'}, QUARTER_CIRCLE + UNIT);
                // this->operator[]('C').add_edge('A', Direction::LEFT, {'E'}, 2.0f * UNIT);
                this->operator[]('C').add_edge('E', Direction::STRAIGHT, {'A', 'B'}, UNIT);
                this->operator[]('C').add_edge('B', Direction::RIGHT, {'E'}, QUARTER_CIRCLE);
                this->operator[]('D').add_edge('B', Direction::STRAIGHT, {'F'}, QUARTER_CIRCLE + UNIT);
                this->operator[]('D').add_edge('F', Direction::LEFT, {'B'}, 2.0f * UNIT);
                this->operator[]('E').add_edge('C', Direction::STRAIGHT, {'F', 'G'}, UNIT);
                this->operator[]('E').add_edge('F', Direction::LEFT, {'C'}, CROSS_UNIT);
                this->operator[]('E').add_edge('G', Direction::RIGHT, {'C'}, 2.0f * UNIT);
                this->operator[]('F').add_edge('D', Direction::RIGHT, {'H', 'I'}, 2.0f * UNIT);
                this->operator[]('F').add_edge('E', Direction::LEFT, {'H', 'I'}, CROSS_UNIT);
                this->operator[]('F').add_edge('H', Direction::LEFT, {'D', 'E'}, 2.0f * UNIT);
                this->operator[]('F').add_edge('I', Direction::RIGHT, {'D', 'E'}, CROSS_UNIT);
                this->operator[]('G').add_edge('E', Direction::LEFT, {'H', 'I'}, 2.0f * UNIT);
                this->operator[]('G').add_edge('H', Direction::LEFT, {'E'}, CROSS_UNIT);
                this->operator[]('G').add_edge('I', Direction::RIGHT, {'E'}, 2.0f * UNIT);
                this->operator[]('H').add_edge('F', Direction::RIGHT, {'J', 'K'}, 2.0f * UNIT);
                this->operator[]('H').add_edge('G', Direction::LEFT, {'J', 'K'}, CROSS_UNIT);
                this->operator[]('H').add_edge('J', Direction::RIGHT, {'F', 'G'}, HALF_CROSS_UNIT);
                this->operator[]('H').add_edge('K', Direction::LEFT, {'F', 'G'}, 4.0f * UNIT);
                this->operator[]('I').add_edge('F', Direction::RIGHT, {'L'}, CROSS_UNIT);
                this->operator[]('I').add_edge('G', Direction::LEFT, {'L'}, 2.0f * UNIT);
                this->operator[]('I').add_edge('L', Direction::RIGHT, {'F', 'G'}, 4.0f * UNIT);
                this->operator[]('J').add_edge('H', Direction::LEFT, {'K', 'L'}, HALF_CROSS_UNIT);
                this->operator[]('J').add_edge('K', Direction::LEFT, {'H'}, HALF_CROSS_UNIT);
                this->operator[]('J').add_edge('L', Direction::RIGHT, {'H'}, HALF_CROSS_UNIT);
                this->operator[]('K').add_edge('H', Direction::RIGHT, {'M', 'N'}, 4.0f * UNIT);
                this->operator[]('K').add_edge('J', Direction::LEFT, {'M', 'N'}, HALF_CROSS_UNIT);
                this->operator[]('K').add_edge('M', Direction::LEFT, {'H', 'J'}, 2.0f * UNIT);
                this->operator[]('K').add_edge('N', Direction::RIGHT, {'H', 'J'}, CROSS_UNIT);
                this->operator[]('L').add_edge('I', Direction::LEFT, {'M', 'N'}, 4.0f * UNIT);
                this->operator[]('L').add_edge('J', Direction::RIGHT, {'M', 'N'}, HALF_CROSS_UNIT);
                this->operator[]('L').add_edge('M', Direction::LEFT, {'I', 'J'}, CROSS_UNIT);
                this->operator[]('L').add_edge('N', Direction::RIGHT, {'I', 'J'}, 2.0f * UNIT);
                this->operator[]('M').add_edge('K', Direction::RIGHT, {'O', 'P'}, 2.0f * UNIT);
                this->operator[]('M').add_edge('L', Direction::LEFT, {'O', 'P'}, CROSS_UNIT);
                this->operator[]('M').add_edge('O', Direction::RIGHT, {'K', 'L'}, HALF_CROSS_UNIT);
                this->operator[]('M').add_edge('P', Direction::LEFT, {'K', 'L'}, 4.0f * UNIT);
                this->operator[]('N').add_edge('K', Direction::RIGHT, {'O', 'Q'}, CROSS_UNIT);
                this->operator[]('N').add_edge('L', Direction::LEFT, {'O', 'Q'}, 2.0f * UNIT);
                this->operator[]('N').add_edge('O', Direction::LEFT, {'K', 'L'}, HALF_CROSS_UNIT);
                this->operator[]('N').add_edge('Q', Direction::RIGHT, {'K', 'L'}, 4.0f * UNIT);
                this->operator[]('O').add_edge('M', Direction::RIGHT, {'P'}, HALF_CROSS_UNIT);
                this->operator[]('O').add_edge('N', Direction::LEFT, {'P'}, HALF_CROSS_UNIT);
                this->operator[]('O').add_edge('P', Direction::RIGHT, {'M', 'N'}, HALF_CROSS_UNIT);
                this->operator[]('P').add_edge('M', Direction::RIGHT, {'R', 'S'}, 4.0f * UNIT);
                this->operator[]('P').add_edge('O', Direction::LEFT, {'R', 'S'}, HALF_CROSS_UNIT);
                this->operator[]('P').add_edge('R', Direction::LEFT, {'M', 'O'}, 2.0f * UNIT);
                this->operator[]('P').add_edge('S', Direction::RIGHT, {'M', 'O'}, CROSS_UNIT);
                this->operator[]('Q').add_edge('N', Direction::LEFT, {'R', 'S'}, 4.0f * UNIT);
                this->operator[]('Q').add_edge('R', Direction::LEFT, {'N'}, CROSS_UNIT);
                this->operator[]('Q').add_edge('S', Direction::RIGHT, {'N'}, 2.0f * UNIT);
                this->operator[]('R').add_edge('P', Direction::RIGHT, {'T', 'U'}, 2.0f * UNIT);
                this->operator[]('R').add_edge('Q', Direction::LEFT, {'T', 'U'}, CROSS_UNIT);
                this->operator[]('R').add_edge('T', Direction::RIGHT, {'P', 'Q'}, HALF_CROSS_UNIT);
                this->operator[]('R').add_edge('U', Direction::LEFT, {'P', 'Q'}, 4.0f * UNIT);
                this->operator[]('S').add_edge('P', Direction::RIGHT, {'T', 'V'}, CROSS_UNIT);
                this->operator[]('S').add_edge('Q', Direction::LEFT, {'T', 'V'}, 2.0f * UNIT);
                this->operator[]('S').add_edge('T', Direction::LEFT, {'P', 'Q'}, HALF_CROSS_UNIT);
                this->operator[]('S').add_edge('V', Direction::RIGHT, {'P', 'Q'}, 4.0f * UNIT);
                this->operator[]('T').add_edge('R', Direction::RIGHT, {'U', 'V'}, HALF_CROSS_UNIT);
                this->operator[]('T').add_edge('S', Direction::LEFT, {'U', 'V'}, HALF_CROSS_UNIT);
                this->operator[]('T').add_edge('U', Direction::LEFT, {'R', 'S'}, HALF_CROSS_UNIT);
                this->operator[]('T').add_edge('V', Direction::RIGHT, {'R', 'S'}, HALF_CROSS_UNIT);
                this->operator[]('U').add_edge('R', Direction::RIGHT, {'W'}, 4.0f * UNIT);
                this->operator[]('U').add_edge('T', Direction::LEFT, {'W'}, HALF_CROSS_UNIT);
                this->operator[]('U').add_edge('W', Direction::STRAIGHT, {'R', 'T'}, UNIT);
                this->operator[]('V').add_edge('S', Direction::LEFT, {'X'}, 4.0f * UNIT);
                this->operator[]('V').add_edge('T', Direction::RIGHT, {'X'}, HALF_CROSS_UNIT);
                this->operator[]('V').add_edge('X', Direction::STRAIGHT, {'S', 'T'}, UNIT);
                this->operator[]('W').add_edge('U', Direction::STRAIGHT, {'X', 'Y'}, UNIT);
                this->operator[]('W').add_edge('X', Direction::RIGHT, {'U'}, 2.0f * QUARTER_CIRCLE);
                // this->operator[]('W').add_edge('Y', Direction::LEFT, {'U'}, 2.0f * UNIT);
                this->operator[]('X').add_edge('V', Direction::STRAIGHT, {'W', 'Z'}, UNIT);
                this->operator[]('X').add_edge('W', Direction::LEFT, {'V'}, 2.0f * QUARTER_CIRCLE);
                // this->operator[]('X').add_edge('Z', Direction::RIGHT, {'V'}, 2.8f);
                // this->operator[]('Y').add_edge('W', Direction::RIGHT, {'Y'}, 2.0f * UNIT);
                // this->operator[]('Z').add_edge('X', Direction::LEFT, {'['}, 2.8f);
                // this->operator[]('Z').add_edge('[', Direction::STRAIGHT, {'X'}, 2.0f * UNIT);
                // this->operator[]('[').add_edge('Z', Direction::STRAIGHT, {'['}, 2.0f * UNIT);

#else
                nodes.push_back(Node{static_cast<char>('A'), 704, 448});
                nodes.push_back(Node{static_cast<char>('B'), 704, 576});
                nodes.push_back(Node{static_cast<char>('C'), 640, 384});
                nodes.push_back(Node{static_cast<char>('D'), 640, 512});
                nodes.push_back(Node{static_cast<char>('E'), 640, 640});
                nodes.push_back(Node{static_cast<char>('F'), 576, 448});
                nodes.push_back(Node{static_cast<char>('G'), 576, 576});
                nodes.push_back(Node{static_cast<char>('H'), 512, 384});
                nodes.push_back(Node{static_cast<char>('I'), 512, 512});
                nodes.push_back(Node{static_cast<char>('J'), 512, 640});
                nodes.push_back(Node{static_cast<char>('K'), 448, 448});
                nodes.push_back(Node{static_cast<char>('L'), 448, 576});
                nodes.push_back(Node{static_cast<char>('M'), 384, 384});
                nodes.push_back(Node{static_cast<char>('N'), 384, 512});
                nodes.push_back(Node{static_cast<char>('O'), 384, 640});
                nodes.push_back(Node{static_cast<char>('P'), 320, 320});
                nodes.push_back(Node{static_cast<char>('Q'), 320, 384});
                nodes.push_back(Node{static_cast<char>('R'), 320, 448});
                nodes.push_back(Node{static_cast<char>('S'), 320, 512});
                nodes.push_back(Node{static_cast<char>('T'), 320, 576});
                nodes.push_back(Node{static_cast<char>('U'), 320, 704});
                nodes.push_back(Node{static_cast<char>('V'), 256, 448});
                nodes.push_back(Node{static_cast<char>('W'), 256, 576});
                nodes.push_back(Node{static_cast<char>('X'), 96, 448});

                const auto UNIT           = 64;
                const auto QUARTER_CIRCLE = 2 * UNIT * M_PI / 4.0f;

                this->operator[]('A').add_edge('C', Direction::LEFT, {'B', 'D'}, QUARTER_CIRCLE);
                this->operator[]('A').add_edge('B', Direction::LEFT, {'C'}, 2.0f * UNIT);
                this->operator[]('A').add_edge('D', Direction::RIGHT, {'C'}, QUARTER_CIRCLE);
                this->operator[]('B').add_edge('A', Direction::RIGHT, {'E'}, 2.0f * UNIT);
                this->operator[]('B').add_edge('E', Direction::RIGHT, {'A', 'D'}, QUARTER_CIRCLE);
                this->operator[]('B').add_edge('D', Direction::LEFT, {'E'}, QUARTER_CIRCLE);
                this->operator[]('C').add_edge('A', Direction::RIGHT, {'F'}, QUARTER_CIRCLE);
                this->operator[]('C').add_edge('F', Direction::LEFT, {'A'}, QUARTER_CIRCLE);
                this->operator[]('D').add_edge('A', Direction::LEFT, {'F', 'G', 'I'}, QUARTER_CIRCLE);
                this->operator[]('D').add_edge('B', Direction::RIGHT, {'F', 'G', 'I'}, QUARTER_CIRCLE);
                this->operator[]('D').add_edge('G', Direction::LEFT, {'A', 'B'}, QUARTER_CIRCLE);
                this->operator[]('D').add_edge('I', Direction::STRAIGHT, {'A', 'B'}, 2.0f * UNIT);
                this->operator[]('D').add_edge('F', Direction::RIGHT, {'A', 'B'}, QUARTER_CIRCLE);
                this->operator[]('E').add_edge('B', Direction::LEFT, {'G', 'J'}, QUARTER_CIRCLE);
                this->operator[]('E').add_edge('J', Direction::LEFT, {'B'}, 2.0f * UNIT);
                this->operator[]('E').add_edge('G', Direction::RIGHT, {'B'}, QUARTER_CIRCLE);
                this->operator[]('F').add_edge('C', Direction::RIGHT, {'D', 'G', 'I'}, QUARTER_CIRCLE);
                this->operator[]('F').add_edge('D', Direction::LEFT, {'C', 'H'}, QUARTER_CIRCLE);
                this->operator[]('F').add_edge('G', Direction::STRAIGHT, {'C', 'H'}, 2.0f * UNIT);
                this->operator[]('F').add_edge('I', Direction::RIGHT, {'C', 'H'}, QUARTER_CIRCLE);
                this->operator[]('F').add_edge('H', Direction::LEFT, {'D', 'G', 'I'}, QUARTER_CIRCLE);
                this->operator[]('G').add_edge('F', Direction::STRAIGHT, {'E', 'J'}, 2.0f * UNIT);
                this->operator[]('G').add_edge('D', Direction::RIGHT, {'E', 'J'}, QUARTER_CIRCLE);
                this->operator[]('G').add_edge('E', Direction::LEFT, {'D', 'F', 'I'}, QUARTER_CIRCLE);
                this->operator[]('G').add_edge('J', Direction::RIGHT, {'D', 'F', 'I'}, QUARTER_CIRCLE);
                this->operator[]('G').add_edge('I', Direction::LEFT, {'E', 'J'}, QUARTER_CIRCLE);
                this->operator[]('H').add_edge('F', Direction::RIGHT, {'K', 'M'}, QUARTER_CIRCLE);
                this->operator[]('H').add_edge('K', Direction::LEFT, {'F'}, QUARTER_CIRCLE);
                this->operator[]('H').add_edge('M', Direction::RIGHT, {'F'}, 2.0f * UNIT);
                this->operator[]('I').add_edge('F', Direction::LEFT, {'K', 'L', 'N'}, QUARTER_CIRCLE);
                this->operator[]('I').add_edge('D', Direction::STRAIGHT, {'K', 'L', 'N'}, 2.0f * UNIT);
                this->operator[]('I').add_edge('G', Direction::RIGHT, {'K', 'L', 'N'}, QUARTER_CIRCLE);
                this->operator[]('I').add_edge('L', Direction::LEFT, {'D', 'F', 'G'}, QUARTER_CIRCLE);
                this->operator[]('I').add_edge('N', Direction::STRAIGHT, {'D', 'F', 'G'}, 2.0f * UNIT);
                this->operator[]('I').add_edge('K', Direction::RIGHT, {'D', 'F', 'G'}, QUARTER_CIRCLE);
                this->operator[]('J').add_edge('G', Direction::LEFT, {'L'}, QUARTER_CIRCLE);
                this->operator[]('J').add_edge('E', Direction::RIGHT, {'L'}, 2.0f * UNIT);
                this->operator[]('J').add_edge('L', Direction::RIGHT, {'E', 'G'}, QUARTER_CIRCLE);
                this->operator[]('K').add_edge('H', Direction::RIGHT, {'I', 'L', 'N'}, QUARTER_CIRCLE);
                this->operator[]('K').add_edge('I', Direction::LEFT, {'H', 'M'}, QUARTER_CIRCLE);
                this->operator[]('K').add_edge('L', Direction::STRAIGHT, {'H', 'M'}, 2.0f * UNIT);
                this->operator[]('K').add_edge('N', Direction::RIGHT, {'H', 'M'}, QUARTER_CIRCLE);
                this->operator[]('K').add_edge('M', Direction::LEFT, {'I', 'L', 'N'}, QUARTER_CIRCLE);
                this->operator[]('L').add_edge('K', Direction::STRAIGHT, {'J', 'O'}, 2.0f * UNIT);
                this->operator[]('L').add_edge('I', Direction::RIGHT, {'J', 'O'}, QUARTER_CIRCLE);
                this->operator[]('L').add_edge('J', Direction::LEFT, {'I', 'K', 'N'}, QUARTER_CIRCLE);
                this->operator[]('L').add_edge('O', Direction::RIGHT, {'I', 'K', 'N'}, QUARTER_CIRCLE);
                this->operator[]('L').add_edge('N', Direction::LEFT, {'J', 'O'}, QUARTER_CIRCLE);
                this->operator[]('M').add_edge('H', Direction::LEFT, {'P', 'Q', 'R'}, 2.0f * UNIT);
                this->operator[]('M').add_edge('K', Direction::RIGHT, {'P', 'Q', 'R'}, QUARTER_CIRCLE);
                this->operator[]('M').add_edge('R', Direction::LEFT, {'H', 'K'}, QUARTER_CIRCLE);
                this->operator[]('M').add_edge('Q', Direction::STRAIGHT, {'H', 'K'}, UNIT);
                // this->operator[]('M').add_edge('P', Direction::RIGHT, {'H', 'K'}, QUARTER_CIRCLE);
                this->operator[]('N').add_edge('K', Direction::LEFT, {'R', 'S', 'T'}, QUARTER_CIRCLE);
                this->operator[]('N').add_edge('I', Direction::STRAIGHT, {'R', 'S', 'T'}, 2.0f * UNIT);
                this->operator[]('N').add_edge('L', Direction::RIGHT, {'R', 'S', 'T'}, QUARTER_CIRCLE);
                this->operator[]('N').add_edge('T', Direction::LEFT, {'K', 'I', 'L'}, QUARTER_CIRCLE);
                this->operator[]('N').add_edge('S', Direction::STRAIGHT, {'K', 'I', 'L'}, UNIT);
                this->operator[]('N').add_edge('R', Direction::RIGHT, {'K', 'I', 'L'}, QUARTER_CIRCLE);
                this->operator[]('O').add_edge('L', Direction::LEFT, {'T', 'U', 'W'}, QUARTER_CIRCLE);
                // this->operator[]('O').add_edge('U', Direction::LEFT, {'L'}, QUARTER_CIRCLE);
                this->operator[]('O').add_edge('W', Direction::STRAIGHT, {'L'}, UNIT + QUARTER_CIRCLE);
                this->operator[]('O').add_edge('T', Direction::RIGHT, {'L'}, QUARTER_CIRCLE);
                this->operator[]('P').add_edge('M', Direction::LEFT, {'P'}, QUARTER_CIRCLE);
                this->operator[]('P').add_edge('R', Direction::RIGHT, {'P'}, 2.0f * UNIT);
                this->operator[]('Q').add_edge('M', Direction::STRAIGHT, {'V', 'X'}, UNIT);
                this->operator[]('Q').add_edge('V', Direction::LEFT, {'M'}, QUARTER_CIRCLE);
                // this->operator[]('Q').add_edge('X', Direction::RIGHT, {'M'}, 3.0f * UNIT + 0.85f * QUARTER_CIRCLE);
                this->operator[]('R').add_edge('M', Direction::RIGHT, {'N', 'T'}, QUARTER_CIRCLE);
                this->operator[]('R').add_edge('N', Direction::LEFT, {'M', 'P'}, QUARTER_CIRCLE);
                this->operator[]('R').add_edge('T', Direction::RIGHT, {'M', 'P'}, 2.0f * UNIT);
                // this->operator[]('R').add_edge('P', Direction::LEFT, {'N', 'T'}, 2.0f * UNIT);
                this->operator[]('S').add_edge('N', Direction::STRAIGHT, {'V', 'W'}, UNIT);
                this->operator[]('S').add_edge('W', Direction::LEFT, {'N'}, QUARTER_CIRCLE);
                this->operator[]('S').add_edge('V', Direction::RIGHT, {'N'}, QUARTER_CIRCLE);
                this->operator[]('T').add_edge('R', Direction::LEFT, {'O', 'U'}, 2.0f * UNIT);
                this->operator[]('T').add_edge('N', Direction::RIGHT, {'O', 'U'}, QUARTER_CIRCLE);
                this->operator[]('T').add_edge('O', Direction::LEFT, {'N', 'R'}, QUARTER_CIRCLE);
                // this->operator[]('T').add_edge('U', Direction::RIGHT, {'N', 'R'}, 2.0f * UNIT);
                // this->operator[]('U').add_edge('T', Direction::LEFT, {'U'}, 2.0f * UNIT);
                // this->operator[]('U').add_edge('O', Direction::RIGHT, {'U'}, QUARTER_CIRCLE);
                this->operator[]('V').add_edge('Q', Direction::RIGHT, {'S', 'W'}, QUARTER_CIRCLE);
                this->operator[]('V').add_edge('S', Direction::LEFT, {'Q'}, QUARTER_CIRCLE);
                this->operator[]('V').add_edge('W', Direction::RIGHT, {'Q'}, 2.0f * UNIT);
                this->operator[]('W').add_edge('V', Direction::LEFT, {'O'}, 2.0f * UNIT);
                this->operator[]('W').add_edge('S', Direction::RIGHT, {'O'}, QUARTER_CIRCLE);
                this->operator[]('W').add_edge('O', Direction::STRAIGHT, {'S', 'V'}, UNIT + QUARTER_CIRCLE);
                // this->operator[]('X').add_edge('Q', Direction::LEFT, {'Y'}, 3.0f * UNIT + 0.85f * QUARTER_CIRCLE);
                // this->operator[]('X').add_edge('Y', Direction::STRAIGHT, {'Q'}, 2.0f * UNIT);
                // this->operator[]('Y').add_edge('X', Direction::STRAIGHT, {'Y'}, 2.0f * UNIT);
#endif
            }

            ~Graph() {}

            Node &operator[](char name)
            {
#ifdef Q2
                if (nodes.empty() || name < 'A' || name > '[') { return invalid_node; }
                return nodes[static_cast<int>(name - 'A')];
#else
                if (nodes.empty() || name < 'A' || name > 'Y') { return invalid_node; }
                return nodes[static_cast<int>(name - 'A')];
#endif
            }

            const Node &operator[](char name) const
            {
#ifdef Q2
                if (nodes.empty() || name < 'A' || name > '[') { return invalid_node; }
                return nodes[static_cast<int>(name - 'A')];
#else
                if (nodes.empty() || name < 'A' || name > 'Y') { return invalid_node; }
                return nodes[static_cast<int>(name - 'A')];
#endif
            }
        };

        class PirateController
        {
        public:
            unsigned long          selected_front           = smodel::SENSOR_COUNT / 2;
            unsigned long          selected_rear            = smodel::SENSOR_COUNT / 2;
            [[maybe_unused]] float line_position_front      = 0.0f;
            [[maybe_unused]] float line_position_rear       = 0.0f;
            [[maybe_unused]] float prev_line_position_front = 0.0f;
            [[maybe_unused]] float prev_line_position_rear  = 0.0f;
            float                  target_angle             = 0.0f;
            float                  target_speed             = 0.0f;

            Direction previous_direction;
            Direction next_direction;
            Direction after_next_direction;

            bool               detection_front[smodel::SENSOR_COUNT];
            bool               detection_rear[smodel::SENSOR_COUNT];
            std::vector<float> line_positions_front;
            std::vector<float> line_positions_rear;

            unsigned long selected_edge = 0u;

            float distance_travelled = 0.0f;
            int   section_percentage = 0;

            bool started = false;

            PirateController()
            {
                if (USE_SEED) { srand(RANDOM_SEED); }
                else
                {
                    auto seed = time(NULL);
                    std::cout << "SEED: " << seed << std::endl;
                    srand(seed);
                }
                init();
            }

            ~PirateController() {}

            void init()
            {
#ifdef Q2
                previous_node      = 'A';
                previous_direction = Direction::STRAIGHT;

                next_node            = 'C';
                next_direction       = Direction::RIGHT;
                after_next_node      = 'E';
                after_next_direction = Direction::STRAIGHT;
#else
                previous_node      = 'P';
                previous_direction = Direction::STRAIGHT;

                if (rand() % 2 == 0)
                {
                    next_node      = 'R';
                    next_direction = Direction::RIGHT;
                    if (rand() % 2 == 0)
                    {
                        after_next_node      = 'N';
                        after_next_direction = Direction::LEFT;
                    }
                    else
                    {
                        after_next_node      = 'T';
                        after_next_direction = Direction::RIGHT;
                    }
                }
                else
                {
                    next_node      = 'M';
                    next_direction = Direction::LEFT;
                    if (rand() % 2 == 0)
                    {
                        after_next_node      = 'H';
                        after_next_direction = Direction::LEFT;
                    }
                    else
                    {
                        after_next_node      = 'K';
                        after_next_direction = Direction::RIGHT;
                    }
                }
#endif
                section_percentage      = 0;
                distance_travelled      = 0.0f;
                selected_edge           = 0u;
                target_angle            = 0.0f;
                target_speed            = 0.0f;
                prev_error              = 0.0f;
                integral                = 0.0f;
                prev_control_timestamp_ = std::chrono::steady_clock::now();
                std::cout << previous_node << "->" << next_node << "->" << after_next_node << "[" << next_direction << ", " << after_next_direction
                          << "]" << std::endl;
            }

            void reset()
            {
                started = false;
                init();
            }

            void start()
            {
                started = true;
                init();
            }

            float PID(const float error, const float dt)
            {
                float proportional_term = Kp * error;
                integral += Ki * error * dt;
                float derivative_term = Kd * (error - prev_error) / dt;
                return proportional_term + integral + derivative_term;
            }

            void set_detection_front(bool *detection_front_, std::vector<float> line_positions_front_)
            {
                for (unsigned long i = 0; i < smodel::SENSOR_COUNT; i++) detection_front[i] = detection_front_[i];
                line_positions_front = line_positions_front_;
            }

            void set_detection_rear(bool *detection_rear_, std::vector<float> line_positions_rear_)
            {
                for (unsigned long i = 0; i < smodel::SENSOR_COUNT; i++) detection_rear[i] = detection_rear_[i];
                line_positions_rear = line_positions_rear_;
            }

            float select_control_point(std::vector<float> line_positions, float prev_line_position)
            {
                std::sort(line_positions.begin(), line_positions.end());

                if (line_positions.size() == 1) { return line_positions[0]; }
                else if (line_positions.size() == 2)
                {
                    float tmp;
                    switch (next_direction)
                    {
                        case Direction::LEFT:
                        {
                            tmp = line_positions[0];
                            break;
                        }
                        case Direction::STRAIGHT:
                        {
                            if (next_direction == previous_direction)
                            {
                                tmp = std::fabs(line_positions[0] - prev_line_position) > std::fabs(line_positions[1] - prev_line_position)
                                          ? line_positions[0]
                                          : line_positions[1];
                            }
                            else
                            {
                                tmp = std::fabs(line_positions[0] - prev_line_position) < std::fabs(line_positions[1] - prev_line_position)
                                          ? line_positions[0]
                                          : line_positions[1];
                            }
                            break;
                        }
                        case Direction::RIGHT:
                        {
                            tmp = line_positions[1];
                            break;
                        }
                        default:
                        {
                            tmp = 0.0f;
                            break;
                        }
                    }

                    if (section_percentage > 50)
                    {
                        tmp = std::fabs(line_positions[0]) < std::fabs(line_positions[1]) ? line_positions[0] : line_positions[1];
                    }
                    return tmp;
                }
                else if (line_positions.size() == 3)
                {
                    switch (next_direction)
                    {
                        case Direction::LEFT:
                        {
                            return line_positions[0];
                        }
                        case Direction::STRAIGHT:
                        {
                            return line_positions[1];
                        }
                        case Direction::RIGHT:
                        {
                            return line_positions[2];
                        }
                        default:
                            return 0.0f;
                    }
                }
                else if (line_positions.size() == 4)
                {
                    switch (next_direction)
                    {
                        case Direction::LEFT:
                        {
                            return line_positions[0];
                        }
                        case Direction::STRAIGHT:
                        {
                            return line_positions[1] + line_positions[2] / 2.0f;
                        }
                        case Direction::RIGHT:
                        {
                            return line_positions[3];
                        }
                        default:
                            return 0.0f;
                    }
                }
                else
                {
                    // this should never happen

                    return 0.0f;
                }
            }

            void lateral_control([[maybe_unused]] const float dt)
            {
                if (std::all_of(std::begin(detection_front), std::end(detection_front), [](bool b) { return b; }) || line_positions_front.size() == 0)
                {
                    return;
                }

                if (line_positions_front.size() > 4 || line_positions_rear.size() > 4) { return; }

                line_position_front      = select_control_point(line_positions_front, prev_line_position_front);
                line_position_rear       = select_control_point(line_positions_rear, prev_line_position_rear);
                prev_line_position_front = line_position_front;
                prev_line_position_rear  = line_position_rear;

                float sensor_rate   = 1.0;
                float sensor_center = smodel::SENSOR_COUNT / 2.0f;
                selected_front      = static_cast<unsigned long>(line_position_front / sensor_rate + sensor_center);
                selected_rear       = static_cast<unsigned long>(line_position_rear / sensor_rate + sensor_center);

                target_angle = PID(line_position_front, dt);

                prev_error = line_position_front;
            }

            void longitudinal_control([[maybe_unused]] const float dt) { target_speed = SPEED; }

            PirateInterface update([[maybe_unused]] bool under_gate_, [[maybe_unused]] bool at_cross_section_, [[maybe_unused]] vmodel::State state_)
            {
                if (started)
                {
                    auto                   control_timestamp_ = std::chrono::steady_clock::now();
                    [[maybe_unused]] float dt =
                        std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
                    prev_control_timestamp_ = control_timestamp_;

                    bool at_decision_point = under_gate_ || at_cross_section_;

                    for (unsigned long i = 0; i < graph[previous_node].edges.size(); i++)
                    {
                        if (graph[previous_node].edges[i].node == next_node)
                        {
                            selected_edge = i;
                            break;
                        }
                    }

                    distance_travelled += SPEED * dt;

                    if (/*!prev_at_decision_point && at_decision_point*/ true)
                    {
                        if (std::sqrt(std::pow(graph[next_node].x - state_.x, 2) + std::pow(graph[next_node].y - state_.y, 2)) < 8.0f)
                        {
                            previous_node      = next_node;
                            next_node          = after_next_node;
                            previous_direction = next_direction;
                            next_direction     = after_next_direction;
                            distance_travelled = 0.0f;

                            while (true)
                            {
                                unsigned long num_neighbors = graph[next_node].edges.size();
                                auto          selected_edge = rand() % num_neighbors;

                                auto prev_nodes = graph[next_node].edges[selected_edge].prev_nodes;
                                if (std::find(prev_nodes.begin(), prev_nodes.end(), previous_node) != prev_nodes.end())
                                {
                                    after_next_node      = graph[next_node].edges[selected_edge].node;
                                    after_next_direction = graph[next_node].edges[selected_edge].direction;
                                    break;
                                }
                            }

                            std::cout << previous_node << "->" << next_node << "->" << after_next_node << "[" << next_direction << ", "
                                      << after_next_direction << "]" << std::endl;
                        }
                    }

                    auto distance      = graph[previous_node].edges[selected_edge].distance;
                    section_percentage = static_cast<int>(distance_travelled / distance * 100.0f);

                    lateral_control(dt);
                    longitudinal_control(dt);

                    prev_at_decision_point = at_decision_point;

                    return PirateInterface{previous_node, next_node, after_next_node, section_percentage};
                }
                else { return PirateInterface{previous_node, next_node, after_next_node, section_percentage}; }
            }

        private:
            float integral               = 0.0f;
            float prev_error             = 0.0f;
            bool  prev_at_decision_point = false;

            std::chrono::time_point<std::chrono::steady_clock> prev_control_timestamp_ = std::chrono::steady_clock::now();

            Graph graph;
            char  previous_node;
            char  next_node;
            char  after_next_node;
        };
    }  // namespace pmodel

}  // namespace rsim

#endif  // PIRATE_MODEL_HXX