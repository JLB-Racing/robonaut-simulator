#include <iostream>

#include "simulation.hxx"
#include "environment.hxx"
#include "JLB/logic.hxx"

#include <SFML/Graphics.hpp>

int main(int, char **)
{
    double start_x = 320.0;
    double start_y = 724.0;
    double start_orientation = -M_PI / 2.0;

    double opp_start_x = 320.0;
    double opp_start_y = 300.0;
    double opp_start_orientation = M_PI / 2.0;

    rsim::Simulation simulation{start_x, start_y, start_orientation, opp_start_x, opp_start_y, opp_start_orientation};
    jlb::Controller controller;
    jlb::Controller opp_controller;
    jlb::Controller fast_controller;

    double fast_start_x = 224.0;
    double fast_start_y = 384.0;
    double fast_start_orientation = M_PI / 2.0;

    rsim::env::Car fast_car{fast_start_x, fast_start_y, fast_start_orientation};

    sf::RenderWindow window(sf::VideoMode(rsim::env::MAP_WIDTH, rsim::env::MAP_HEIGHT), "RobonAUT Simulator");
    window.setPosition(sf::Vector2i(500, 50));
    window.setFramerateLimit(60);

    sf::Texture car_texture;
    if (!car_texture.loadFromFile("assets/car.png"))
    {
        std::cout << "Error loading assets/car.png" << std::endl;
        return 1;
    }
    sf::Sprite car_sprite;
    car_sprite.setTexture(car_texture);
    car_sprite.setOrigin(8.0f, 8.0f);

    sf::Texture opp_texture;
    if (!opp_texture.loadFromFile("assets/opp.png"))
    {
        std::cout << "Error loading assets/opp.png" << std::endl;
        return 1;
    }
    sf::Sprite opp_sprite;
    opp_sprite.setTexture(opp_texture);
    opp_sprite.setOrigin(8.0f, 8.0f);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        double target_angle = controller.lateral_control(simulation.car, jlb::Controller::Direction::RIGHT);
        double target_speed = controller.longitudinal_control(jlb::Controller::Direction::RIGHT);
        double opp_target_angle = opp_controller.lateral_control(simulation.opp, jlb::Controller::Direction::LEFT);
        double opp_target_speed = opp_controller.longitudinal_control(jlb::Controller::Direction::LEFT);
        double fast_target_angle = fast_controller.lateral_control(fast_car, jlb::Controller::Direction::STRAIGHT);
        // double fast_target_speed = fast_controller.longitudinal_control(jlb::Controller::Direction::STRAIGHT);
        simulation.update(target_angle, target_speed, opp_target_angle, opp_target_speed);
        fast_car.detect(simulation.map.data, rsim::env::MAP_WIDTH, rsim::env::MAP_HEIGHT);
        fast_car.update(fast_target_angle, 30.0);
        // print fast detection
        for (int i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
        {
            std::cout << fast_car.line_sensor.detection[i];
        }
        std::cout << "\t" << fast_controller.selected << std::endl;

        window.clear(sf::Color::White);

        for (auto gate : simulation.map.gates)
        {
            // print a circle for each gate
            sf::CircleShape circle(8);
            // circle.setFillColor(sf::Color::Red);
            switch (gate.state)
            {
            case rsim::env::Gate::State::UNMAPPED:
                circle.setFillColor(sf::Color(200, 200, 200));
                break;
            case rsim::env::Gate::State::STOLEN_ONCE:
                circle.setFillColor(sf::Color::Yellow);
                break;
            case rsim::env::Gate::State::STOLEN_TWICE:
                circle.setFillColor(sf::Color::Red);
                break;
            case rsim::env::Gate::State::MAPPED:
                circle.setFillColor(sf::Color::Green);
                break;
            }

            circle.setPosition(gate.x - 8, gate.y - 8);
            window.draw(circle);
        }

        // iterate through map
        for (unsigned long col = 0; col < rsim::env::MAP_WIDTH; col++)
        {
            for (unsigned long row = 0; row < rsim::env::MAP_HEIGHT; row++)
            {
                if (!simulation.map.data[col][row])
                {
                    sf::RectangleShape rectangle(sf::Vector2f(1, 1));
                    rectangle.setPosition(col, row);
                    rectangle.setFillColor(sf::Color::Black);
                    window.draw(rectangle);
                }
            }
        }

        car_sprite.setPosition(simulation.car.state.x, simulation.car.state.y);
        car_sprite.setRotation(simulation.car.state.orientation * 180 / M_PI + 90);
        window.draw(car_sprite);

        car_sprite.setPosition(fast_car.state.x, fast_car.state.y);
        car_sprite.setRotation(fast_car.state.orientation * 180 / M_PI + 90);
        window.draw(car_sprite);

        opp_sprite.setPosition(simulation.opp.state.x, simulation.opp.state.y);
        opp_sprite.setRotation(simulation.opp.state.orientation * 180 / M_PI + 90);
        window.draw(opp_sprite);

        window.display();
    }

    return 0;
}
