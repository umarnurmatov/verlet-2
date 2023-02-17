#include "imgui.h"
#include "imgui-SFML.h"

#include <SFML/Graphics.hpp>
#include <iostream>
#include "physics.hpp"
#include "renderer.hpp"

using namespace std;

int main() {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 4;
    sf::RenderWindow window(sf::VideoMode::getFullscreenModes()[0], "Verlet-2", sf::Style::Default, settings);
    window.setFramerateLimit(60);
    bool imgui_sfml_init = ImGui::SFML::Init(window);

    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);

    Solver solver;
    solver.addRectangle(window.getSize().x, window.getSize().y / 4, 0, window.getSize().y * 3 / 4);

    Renderer renderer(&window);

    sf::Clock deltaClock;
    sf::Clock solverClock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);

            if (event.type == sf::Event::Closed || (event.key.code == sf::Keyboard::Q && event.key.control)) {
                window.close();
            }
            if(event.type == event.MouseButtonPressed && event.mouseButton.button == sf::Mouse::Button::Left)
            {
                sf::Vector2f mouse_pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                solver.addRectangle(100.f, 100.f, mouse_pos.x, mouse_pos.y); 
            }
        }

        ImGui::SFML::Update(window, deltaClock.restart());


        solver.update(solverClock.restart().asSeconds());

        window.clear();
        renderer.render(solver);
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();

    return 0;
}