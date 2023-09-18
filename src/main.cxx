#include <iostream>

#include "vehicle_model.hxx"
#include "sensor_model.hxx"
#include "environment.hxx"
#include "utility.hxx"
#include "JLB/logic.hxx"

#include <SFML/Graphics.hpp>

int main(int, char **)
{
    double start_x = 320.0;
    double start_y = 660.0;
    double start_orientation = -M_PI / 2.0;

    rsim::env::Car car = rsim::env::Car{start_x, start_y, start_orientation};
    rsim::env::Map map = rsim::env::Map{};
    jlb::Controller controller;

    sf::RenderWindow window(sf::VideoMode(rsim::env::MAP_WIDTH, rsim::env::MAP_HEIGHT), "RobonAUT Simulator");
    window.setPosition(sf::Vector2i(1250, 250));
    // set framerate
    window.setFramerateLimit(200);

    sf::Texture car_texture;
    if (!car_texture.loadFromFile("assets/car.png"))
    {
        std::cout << "Error loading assets/car.png" << std::endl;
        return 1;
    }
    sf::Sprite car_sprite;
    car_sprite.setTexture(car_texture);
    car_sprite.setOrigin(8.0f, 8.0f);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        car.detect(map.data, rsim::env::MAP_HEIGHT, rsim::env::MAP_WIDTH);
        double target_angle = controller.lateral_control(car.state, car.line_sensor.detection);
        car.update(target_angle, 10.0);
        window.clear(sf::Color::White);

        // iterate through map
        for (unsigned long col = 0; col < rsim::env::MAP_WIDTH; col++)
        {
            for (unsigned long row = 0; row < rsim::env::MAP_HEIGHT; row++)
            {
                if (!map.data[row][col])
                {
                    sf::RectangleShape rectangle(sf::Vector2f(1, 1));
                    rectangle.setPosition(row, col);
                    rectangle.setFillColor(sf::Color::Black);
                    window.draw(rectangle);
                }
            }
        }

        car_sprite.setPosition(car.state.x, car.state.y);
        car_sprite.setRotation(car.state.orientation * 180 / M_PI + 90);
        window.draw(car_sprite);

        window.display();
    }

    return 0;
}
