#include <iostream>
#include <thread>

#include <SFML/Graphics.hpp>

#include "simulation.hxx"

/*===================================================*/
/*      INCLUDE YOUR HEADERS HERE                    */
/*                                                   */
#include "JLB/logic.hxx"
#include "JLB/odometry.hxx"
/*                                                   */
/*===================================================*/

int main(int, char **)
{
    rsim::Simulation simulation;

    jlb::Controller controller{jlb::Direction::RIGHT};
    jlb::Odometry odometry{rsim::START_X, rsim::START_Y, rsim::START_ORIENTATION};

    std::thread thread_control(
        [&]()
        {
            while (true)
            {

                /*===================================================*/
                /*      UPDATE CONTROL LOGIC HERE                    */
                /*                                                   */
                [[maybe_unused]] auto [vx, x, y, theta] = odometry.update_odom();
                controller.set_current_velocity(vx);
                controller.update();
                /*                                                   */
                /*===================================================*/

                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
            }
        });

    std::thread thread_sensors_high(
        [&]()
        {
            while (true)
            {
                [[maybe_unused]] auto motor_rpm = simulation.car.noisy_motor_rpm();
                [[maybe_unused]] auto yaw_rate = simulation.car.noisy_yaw_rate();

                /*===================================================*/
                /*      UPDATE SENSOR LOGIC HERE                     */
                /*                                                   */
                odometry.rpm_callback(motor_rpm);
                odometry.imu_callback(yaw_rate);
                /*                                                   */
                /*===================================================*/

                std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 200 Hz
            }
        });

    std::thread thread_sensors_low(
        [&]()
        {
            while (true)
            {
                [[maybe_unused]] auto detection = simulation.car.detect(simulation.map.data);

                /*===================================================*/
                /*      UPDATE SENSOR LOGIC HERE                     */
                /*                                                   */
                controller.set_detection(detection);
                /*                                                   */
                /*===================================================*/

                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
            }
        });

    std::thread thread_simulation(
        [&]()
        {
            while (true)
            {
                simulation.update(controller.target_angle, controller.target_speed);
                std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50 Hz
            }
        });

    ///////////////////////////////////////////////////////////////////////////
    //
    //      GUI
    //

    sf::RenderWindow window(sf::VideoMode(rsim::env::MAP_WIDTH, rsim::env::MAP_HEIGHT), "RobonAUT Simulator");
    window.setPosition(sf::Vector2i(500, 50));
    window.setFramerateLimit(60);

    /*===================================================*/
    /*      LOAD TEXTURES                                */
    /*===================================================*/

    sf::Texture car_texture;
    if (!car_texture.loadFromFile("assets/car.png"))
    {
        std::cout << "Error loading assets/car.png" << std::endl;
        return EXIT_FAILURE;
    }
    sf::Sprite car_sprite;
    car_sprite.setTexture(car_texture);
    car_sprite.setOrigin(8.0f, 8.0f);

    sf::Texture odom_texture;
    if (!odom_texture.loadFromFile("assets/odom.png"))
    {
        std::cout << "Error loading assets/odom.png" << std::endl;
        return EXIT_FAILURE;
    }
    sf::Sprite odom_sprite;
    odom_sprite.setTexture(odom_texture);
    odom_sprite.setOrigin(8.0f, 8.0f);

    sf::Texture pirate_texture;
    if (!pirate_texture.loadFromFile("assets/pirate.png"))
    {
        std::cout << "Error loading assets/pirate.png" << std::endl;
        return EXIT_FAILURE;
    }
    sf::Sprite pirate_sprite;
    pirate_sprite.setTexture(pirate_texture);
    pirate_sprite.setOrigin(8.0f, 8.0f);

    /*===================================================*/
    /*      DRAW GUI                                     */
    /*===================================================*/

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);

        /*===================================================*/
        /*      GATES                                       */
        /*===================================================*/

        for (auto gate : simulation.map.gates)
        {
            sf::CircleShape circle(8);
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

        /*===================================================*/
        /*      MAP                                         */
        /*===================================================*/

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

        /*===================================================*/
        /*      CARS                                        */
        /*===================================================*/

        odom_sprite.setPosition(odometry.x_t, odometry.y_t);
        odom_sprite.setRotation(odometry.theta_t * 180 / M_PI + 90);
        window.draw(odom_sprite);

        car_sprite.setPosition(simulation.car.state.x, simulation.car.state.y);
        car_sprite.setRotation(simulation.car.state.orientation * 180 / M_PI + 90);
        window.draw(car_sprite);

        pirate_sprite.setPosition(simulation.pirate.state.x, simulation.pirate.state.y);
        pirate_sprite.setRotation(simulation.pirate.state.orientation * 180 / M_PI + 90);
        window.draw(pirate_sprite);

        window.display();
    }

    //
    //      END GUI
    //
    ///////////////////////////////////////////////////////////////////////////

    thread_control.join();
    thread_sensors_high.join();
    thread_sensors_low.join();
    thread_simulation.join();

    return EXIT_SUCCESS;
}
