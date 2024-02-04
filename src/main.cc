#include <SFML/Graphics.hpp>
#include <iostream>
#include <thread>

#include "gui.hxx"
#include "simulation.hxx"

/*===================================================*/
/*      TODO: INCLUDE YOUR HEADERS HERE              */
/*                                                   */
#include "JLB/logic.hxx"
/*                                                   */
/*===================================================*/

int main(int, char **)
{
    float wheel_angle = 0.0f;
    float velocity    = 0.0f;

    rsim::Simulation      simulation{rsim::START_X, rsim::START_Y, rsim::START_ORIENTATION};
    [[maybe_unused]] auto x_t0     = px_to_m(rsim::START_X);
    [[maybe_unused]] auto y_t0     = px_to_m(rsim::START_Y);
    [[maybe_unused]] auto theta_t0 = rsim::START_ORIENTATION;

    // rsim::Simulation      simulation{rsim::FAST_START_X, rsim::FAST_START_Y, rsim::FAST_START_ORIENTATION};
    // [[maybe_unused]] auto x_t0     = px_to_m(rsim::FAST_START_X);
    // [[maybe_unused]] auto y_t0     = px_to_m(rsim::FAST_START_Y);
    // [[maybe_unused]] auto theta_t0 = rsim::FAST_START_ORIENTATION;

    /*===================================================*/
    /*      TODO: INITIALIZE YOUR LOGIC HERE             */
    /*                                                   */
    jlb::Logic logic;
    // logic.set_states({jlb::LabyrinthState::START});
    // logic.set_states({jlb::FastState::OUT_ACCEL_ZONE});
    /*                                                   */
    /*===================================================*/

    std::atomic<bool> terminate_threads(false);
    std::atomic<bool> pause_threads(false);

    std::thread thread_control(
        [&]()
        {
            while (!terminate_threads)
            {
                [[maybe_unused]] auto motor_rpm                               = simulation.car.noisy_motor_rpm();
                [[maybe_unused]] auto yaw_rate                                = simulation.car.noisy_yaw_rate();
                [[maybe_unused]] auto safety_car_range                        = simulation.car.detect_object(simulation.safety_car.state);
                [[maybe_unused]] auto [detection_front, line_positions_front] = simulation.car.detect_front(simulation.map.data);
                [[maybe_unused]] auto [detection_rear, line_positions_rear]   = simulation.car.detect_rear(simulation.map.data);
                [[maybe_unused]] auto under_gate                              = simulation.car_under_gate;
                [[maybe_unused]] auto at_cross_section                        = simulation.car_at_cross_section;
                [[maybe_unused]] auto flood                                   = simulation.flood;
                [[maybe_unused]] auto [prev_node, next_node, after_next_node, section_percentage] = simulation.pirate_interface;

                /*===================================================*/
                /*      TODO: UPDATE SENSOR LOGIC HERE               */
                /*                                                   */
                logic.set_object_range(safety_car_range);
                logic.set_detection_front(detection_front, line_positions_front);
                logic.set_detection_rear(detection_rear, line_positions_rear);
                logic.set_under_gate(under_gate);
                logic.set_at_cross_section(at_cross_section);
                logic.set_flood(flood);
                logic.imu_callback(0.0f, 0.0f, yaw_rate, 0.0f, 0.0f, 0.0f);
                logic.rpm_callback(motor_rpm);
                logic.pirate_callback(prev_node, next_node, after_next_node, section_percentage);
                /*                                                   */
                /*===================================================*/

                /*===================================================*/
                /*      TODO: UPDATE CONTROL LOGIC HERE              */
                /*                                                   */
                auto [target_angle, target_speed] = logic.update();
                wheel_angle                       = target_angle;
                velocity                          = target_speed;
                /*                                                   */
                /*===================================================*/

                std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz

                while (pause_threads) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
            }
        });

    std::thread thread_visualization(
        [&]()
        {
            while (!terminate_threads)
            {
                logic.send_telemetry();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10 Hz

                while (pause_threads) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
            }
        });

    std::thread thread_simulation(
        [&]()
        {
            while (!terminate_threads)
            {
                simulation.update(wheel_angle, velocity);

                std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz

                while (pause_threads) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
            }
        });

    ///////////////////////////////////////////////////////////////////////////
    //
    //      GUI
    //

    if (!rsim::RUN_HEADLESS)
    {
        sf::RenderWindow window(sf::VideoMode(rsim::env::MAP_WIDTH, rsim::env::MAP_HEIGHT), "RobonAUT Simulator");
        window.setPosition(sf::Vector2i(500, 50));
        window.setFramerateLimit(60);

        /*===================================================*/
        /*      LOAD TEXTURES                                */
        /*===================================================*/

        sf::Texture car_texture;
        if (!car_texture.loadFromFile("/home/humdalab/RobonAUT/robonaut-simulator/assets/car.png"))
        {
            std::cout << "Error loading /home/humdalab/RobonAUT/robonaut-simulator/assets/car.png" << std::endl;
            return EXIT_FAILURE;
        }
        sf::Sprite car_sprite;
        car_sprite.setTexture(car_texture);
        car_sprite.setOrigin(8.0f, 8.0f);

        sf::Texture odom_texture;
        if (!odom_texture.loadFromFile("/home/humdalab/RobonAUT/robonaut-simulator/assets/odom.png"))
        {
            std::cout << "Error loading /home/humdalab/RobonAUT/robonaut-simulator/assets/odom.png" << std::endl;
            return EXIT_FAILURE;
        }
        sf::Sprite odom_sprite;
        odom_sprite.setTexture(odom_texture);
        odom_sprite.setOrigin(8.0f, 8.0f);

        sf::Texture pirate_texture;
        if (!pirate_texture.loadFromFile("/home/humdalab/RobonAUT/robonaut-simulator/assets/pirate.png"))
        {
            std::cout << "Error loading /home/humdalab/RobonAUT/robonaut-simulator/assets/pirate.png" << std::endl;
            return EXIT_FAILURE;
        }
        sf::Sprite pirate_sprite;
        pirate_sprite.setTexture(pirate_texture);
        pirate_sprite.setOrigin(8.0f, 8.0f);

        sf::Texture safety_car_texture;
        if (!safety_car_texture.loadFromFile("/home/humdalab/RobonAUT/robonaut-simulator/assets/safety_car.png"))
        {
            std::cout << "Error loading /home/humdalab/RobonAUT/robonaut-simulator/assets/safety_car.png" << std::endl;
            return EXIT_FAILURE;
        }
        sf::Sprite safety_car_sprite;
        safety_car_sprite.setTexture(safety_car_texture);
        safety_car_sprite.setOrigin(8.0f, 8.0f);

        sf::Font font;
        if (!font.loadFromFile("/home/humdalab/RobonAUT/robonaut-simulator/assets/Arial.ttf"))
        {
            std::cout << "Error loading /home/humdalab/RobonAUT/robonaut-simulator/assets/Arial.ttf" << std::endl;
            return EXIT_FAILURE;
        }

        /*===================================================*/
        /*      DRAW GUI                                     */
        /*===================================================*/

        rsim::gui::Button floodButton(
            window, sf::Vector2f(10, 10), sf::Vector2f(100, 25), "     Flood      ", font, 12, sf::Color::Black, sf::Color{200, 200, 200});
        rsim::gui::Button resetButton(
            window, sf::Vector2f(10, 45), sf::Vector2f(100, 25), "Reset", font, 12, sf::Color::Black, sf::Color{200, 200, 200});
        rsim::gui::Button safetyCarStartButton(
            window, sf::Vector2f(120, 10), sf::Vector2f(100, 25), "Safety Car start", font, 12, sf::Color::Black, sf::Color{200, 200, 200});
        rsim::gui::Button safetyCarResetButton(
            window, sf::Vector2f(120, 45), sf::Vector2f(100, 25), "Safety Car reset", font, 12, sf::Color::Black, sf::Color{200, 200, 200});
        rsim::gui::Button pauseButton(
            window, sf::Vector2f(230, 10), sf::Vector2f(100, 25), "Pause", font, 12, sf::Color::Black, sf::Color{200, 200, 200});
        rsim::gui::Button startButton(
            window, sf::Vector2f(230, 45), sf::Vector2f(100, 25), "Start", font, 12, sf::Color::Black, sf::Color{200, 200, 200});

        sf::Vector2f mousePos;

        while (window.isOpen())
        {
            sf::Event event;
            while (window.pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
                {
                    terminate_threads = true;
                    thread_control.join();
                    thread_visualization.join();
                    thread_simulation.join();

                    window.close();

                    return EXIT_SUCCESS;
                }

                mousePos = sf::Vector2f(sf::Mouse::getPosition(window));

                // Handle mouse button pressed events
                if (event.type == sf::Event::MouseButtonPressed)
                {
                    if (event.mouseButton.button == sf::Mouse::Left)
                    {
                        if (floodButton.isMouseOver())
                        {
                            if (!simulation.flood)
                            {
                                simulation.flood = true;
                                std::cout << "███████████████████ Flood ███████████████████" << std::endl;
                            }
                        }
                        if (pauseButton.isMouseOver())
                        {
                            if (!pause_threads)
                            {
                                pause_threads = true;
                                std::cout << "███████████████████ Pause ███████████████████" << std::endl;
                            }
                            else
                            {
                                pause_threads = false;
                                std::cout << "███████████████████ Resume ███████████████████" << std::endl;
                            }
                        }
                        if (startButton.isMouseOver())
                        {
                            /*===================================================*/
                            /*      TODO: HANDLE START SEQEUNCE HERE             */
                            /*                                                   */
                            logic.start_signal();
                            /*                                                   */
                            /*===================================================*/
                            simulation.start();
                            std::cout << "███████████████████ Start ███████████████████" << std::endl;
                        }
                        if (resetButton.isMouseOver())
                        {
                            /*===================================================*/
                            /*      TODO: HANDLE RESET SEQEUNCE HERE             */
                            /*                                                   */
                            logic.reset_signal({jlb::LabyrinthState::START});
                            /*                                                   */
                            /*===================================================*/
                            simulation.reset();
                            std::cout << "███████████████████ Reset ███████████████████" << std::endl;
                        }
                    }
                }
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

                sf::Text text;
                text.setFont(font);
                text.setStyle(sf::Text::Bold);
                text.setString(gate.name);
                text.setCharacterSize(14);
                text.setFillColor(sf::Color::Black);
                text.setPosition(gate.x + 8, gate.y + 4);
                window.draw(text);
            }

            for (auto cross_section : simulation.map.cross_sections)
            {
                sf::CircleShape square(8, 4);
                square.setFillColor(sf::Color(150, 150, 200));
                square.setPosition(cross_section.x - 8, cross_section.y - 8);
                window.draw(square);

                sf::Text text;
                text.setFont(font);
                text.setStyle(sf::Text::Bold);
                text.setString(cross_section.name);
                text.setCharacterSize(14);
                text.setFillColor(sf::Color::Black);
                text.setPosition(cross_section.x + 8, cross_section.y + 4);
                window.draw(text);
            }

            /*===================================================*/
            /*      MAP                                         */
            /*===================================================*/

            // balancer end point
            sf::CircleShape circle(rsim::BALANCER_END_RADIUS);
            circle.setFillColor(sf::Color{200, 200, 200});
            circle.setPosition(rsim::BALANCER_END_CENTER_X - rsim::BALANCER_END_RADIUS, rsim::BALANCER_END_CENTER_Y - rsim::BALANCER_END_RADIUS);
            window.draw(circle);

            sf::Text text;
            text.setFont(font);
            text.setStyle(sf::Text::Bold);
            text.setFillColor(sf::Color::Black);
            text.setCharacterSize(14);

#ifndef Q2
            text.setString('X');
            text.setPosition(96 + 8, 448 + 4);
#else
            text.setString('Z');
            text.setPosition(1792 + 8, 224 + 4);
#endif
            window.draw(text);

            text.setFont(font);
            text.setStyle(sf::Text::Bold);
            text.setCharacterSize(14);
            text.setFillColor(sf::Color::Black);
#ifndef Q2
            text.setString('Y');
            text.setPosition(96 + 8, 576 + 4);
#else
            text.setString('[');
            text.setPosition(1920 + 8, 224 + 4);
#endif
            window.draw(text);

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

            float odom_x     = 0.0f;
            float odom_y     = 0.0f;
            float odom_theta = 0.0f;

            /*===================================================*/
            /*      TODO: UPDATE ODOMETRY VISUALIZATION HERE     */
            /*                                myButton                   */
            [[maybe_unused]] auto [vx_t, x_t, y_t, theta_t, distance_local] = logic.get_odometry();
            odom_x                                                          = x_t;
            odom_y                                                          = y_t;
            odom_theta                                                      = theta_t;
            /*                                                   */
            /*===================================================*/

            odom_sprite.setPosition(m_to_px(odom_x), m_to_px(odom_y));
            odom_sprite.setRotation(odom_theta * 180 / M_PI + 90);
            window.draw(odom_sprite);

            car_sprite.setPosition(simulation.car.state.x, simulation.car.state.y);
            car_sprite.setRotation(simulation.car.state.orientation * 180 / M_PI + 90);
            window.draw(car_sprite);

            pirate_sprite.setPosition(simulation.pirate.state.x, simulation.pirate.state.y);
            pirate_sprite.setRotation(simulation.pirate.state.orientation * 180 / M_PI + 90);
            window.draw(pirate_sprite);

            safety_car_sprite.setPosition(simulation.safety_car.state.x, simulation.safety_car.state.y);
            safety_car_sprite.setRotation(simulation.safety_car.state.orientation * 180 / M_PI + 90);
            window.draw(safety_car_sprite);

            floodButton.draw();
            resetButton.draw();
            safetyCarStartButton.draw();
            safetyCarResetButton.draw();
            pauseButton.draw();
            startButton.draw();

            // bottom right corner display mouse position
            sf::Text mousePosText;
            mousePosText.setPosition(10, rsim::env::MAP_HEIGHT - 20);
            mousePosText.setFont(font);
            mousePosText.setCharacterSize(12);
            mousePosText.setFillColor(sf::Color::Black);
            mousePosText.setString(std::to_string(static_cast<int>(mousePos.x)) + ", " + std::to_string(static_cast<int>(mousePos.y)));
            window.draw(mousePosText);

            window.display();
        }
    }

    //
    //      END GUI
    //
    ///////////////////////////////////////////////////////////////////////////

    terminate_threads = true;
    thread_control.join();
    thread_visualization.join();
    thread_simulation.join();

    return EXIT_SUCCESS;
}
