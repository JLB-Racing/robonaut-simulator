#include <iostream>
#include <thread>

#include <SFML/Graphics.hpp>

#include "simulation.hxx"

/*===================================================*/
/*      TODO: INCLUDE YOUR HEADERS HERE              */
/*                                                   */
#include "JLB/logic.hxx"
/*                                                   */
/*===================================================*/

int main(int, char **)
{
    rsim::Simulation simulation;
    float wheel_angle = 0.0f;
    float velocity = m_to_px(0.0f);

    [[maybe_unused]] auto x_t0 = px_to_m(rsim::START_X);
    [[maybe_unused]] auto y_t0 = px_to_m(rsim::START_Y);
    [[maybe_unused]] auto theta_t0 = rsim::START_ORIENTATION;

    /*===================================================*/
    /*      TODO: INITIALIZE YOUR LOGIC HERE             */
    /*                                                   */
    jlb::Logic logic{jlb::Direction::RIGHT, x_t0, y_t0, theta_t0};
    /*                                                   */
    /*===================================================*/

    std::thread thread_control(
        [&]()
        {
            while (true)
            {
                /*===================================================*/
                /*      TODO: UPDATE CONTROL LOGIC HERE              */
                /*                                                   */
                auto [vx, x, y, theta] = logic.odometry.update_odom();
                logic.controller.set_current_velocity(vx);

                auto [target_angle, target_speed] = logic.controller.update();
                wheel_angle = target_angle;
                velocity = target_speed;
                /*                                                   */
                /*===================================================*/

                velocity = m_to_px(velocity);

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
                /*      TODO: UPDATE SENSOR LOGIC HERE               */
                /*                                                   */
                logic.odometry.imu_callback(yaw_rate);
                logic.odometry.rpm_callback(motor_rpm);
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
                [[maybe_unused]] auto detection_front = simulation.car.detect_front(simulation.map.data);
                [[maybe_unused]] auto detection_rear = simulation.car.detect_rear(simulation.map.data);

                /*===================================================*/
                /*      TODO: UPDATE SENSOR LOGIC HERE               */
                /*                                                   */
                logic.controller.set_detection_front(detection_front);
                logic.controller.set_detection_rear(detection_rear);
                /*                                                   */
                /*===================================================*/

                std::this_thread::sleep_for(std::chrono::milliseconds(25)); // 40 Hz
            }
        });

    std::thread thread_visualization(
        [&]()
        {
            while (true)
            {
                logic.signal_sender.send_telemetry();
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
            }
        });

    std::thread thread_simulation(
        [&]()
        {
            while (true)
            {
                simulation.update(wheel_angle, velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 30Hz
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

        float odom_x = m_to_px(0.0f);
        float odom_y = m_to_px(0.0f);
        float odom_theta = 0.0f;

        /*===================================================*/
        /*      TODO: UPDATE ODOMETRY VISUALIZATION HERE     */
        /*                                                   */
        odom_x = m_to_px(logic.odometry.x_t);
        odom_y = m_to_px(logic.odometry.y_t);
        odom_theta = logic.odometry.theta_t;
        /*                                                   */
        /*===================================================*/

        odom_sprite.setPosition(odom_x, odom_y);
        odom_sprite.setRotation(odom_theta * 180 / M_PI + 90);
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
    thread_visualization.join();
    thread_simulation.join();

    return EXIT_SUCCESS;
}
