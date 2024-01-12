#ifndef GUI_HXX
#define GUI_HXX

#include <SFML/Graphics.hpp>
#include <codecvt>
#include <iostream>
#include <locale>

namespace rsim
{

    namespace gui
    {

        class Button
        {
        public:
            Button(sf::RenderWindow&                   window,
                   const sf::Vector2f&                 position,
                   const sf::Vector2f&                 size,
                   [[maybe_unused]] const std::string& text,
                   const sf::Font&                     font,
                   const int                           font_size,
                   const sf::Color&                    font_color,
                   const sf::Color&                    button_color)
                : window(window), font(font)
            {
                button.setSize(size);
                button.setPosition(position);
                button.setFillColor(button_color);

                buttonText.setFont(font);
                std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
                std::wstring                                     sf_text = converter.from_bytes(text);
                buttonText.setString(sf_text);
                buttonText.setCharacterSize(font_size);
                buttonText.setFillColor(font_color);

                // Center the text within the button
                sf::FloatRect textBounds = buttonText.getLocalBounds();
                buttonText.setOrigin(textBounds.left + textBounds.width / 2.0f, textBounds.top + textBounds.height / 2.0f);
                buttonText.setPosition(position.x + size.x / 2.0f, position.y + size.y / 2.0f);
            }

            void draw()
            {
                window.draw(button);
                window.draw(buttonText);
            }

            bool isMouseOver()
            {
                sf::Vector2f mousePos = sf::Vector2f(sf::Mouse::getPosition(window));
                return button.getGlobalBounds().contains(mousePos);
            }

        private:
            sf::RenderWindow&  window;
            sf::RectangleShape button;
            sf::Text           buttonText;
            sf::Font           font;
        };

    }  // namespace gui

}  // namespace rsim

#endif  // GUI_HXX