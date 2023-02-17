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

    // std::vector<int> v = {3, 6, 3, 5, 6};
    // auto p = [&](){ for(auto i : v) std::cout << i << " "; std::cout << std::endl; };
    // std::vector<int>::pointer pn = &v[2];
    // p();
    // cout << *pn << endl;
    // std::sort(v.begin(), v.end());
    // p();
    // cout << *pn << endl;

    // int a = 5;
    // int& p = a;
    // int* pp = &p;
    // std::cout << *pp << endl;
    // *pp = 10;
    // cout << a << endl;

    Solver solver;
    //solver.addRectangle(100.f, 100.f, 100.f, 100.f);
    solver.addRectangle(100.f, 100.f, 120.f, 100.f); 
    solver.addRectangle(100.f, 100.f, 100.f, 300.f); 

    const auto& vert = solver.getVertexes();
    for(const auto& v : vert)
    {
        std::cout << v.position.x << " " << v.position.y << endl;
    }
    cout << endl;

    const auto& edg = solver.getEdges();
    for(const auto& e : edg)
    {
        std::cout << e.v1->position.x << " " << e.v1->position.y << endl;
        std::cout << e.v2->position.x << " " << e.v2->position.y << endl;
        cout << "--" << endl;
    }

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
            // if(event.MouseButtonPressed && event.mouseButton.button == sf::Mouse::Button::Left)
            // {
            //     sf::Vector2f mouse_pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            //     cout << mouse_pos.x << " " << mouse_pos.y << endl;
            //     solver.addRectangle(100.f, 100.f, mouse_pos.x, mouse_pos.y); 
            // }
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