#include <iostream>

#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include "solver.hpp"
#include "renderer.hpp"

using namespace std;

std::vector<float> fps;

int main() {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 4;
    sf::RenderWindow window(sf::VideoMode::getFullscreenModes()[0], "Verlet-2", sf::Style::Default, settings);
    window.setFramerateLimit(60);
    bool imgui_sfml_init = ImGui::SFML::Init(window);

    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);

    Solver solver;
    //solver.addRectangle(window.getSize().x, window.getSize().y / 4, 0, window.getSize().y * 3 / 4);


    Renderer renderer(&window);

    sf::Clock deltaClock;
    sf::Clock solverClock;
    sf::Clock fpsClock;
    float current_fps;
    int iterations = 8;
    bool isSoft = false;
    
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);

            if (event.type == sf::Event::Closed || (event.key.code == sf::Keyboard::Q && event.key.control)) {
                window.close();
            }
            sf::Vector2f mouse_pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            if(event.type == event.MouseButtonPressed && event.mouseButton.button == sf::Mouse::Button::Left)
            {
                solver.addTriangle(20.f, mouse_pos.x, mouse_pos.y, 100.f, false);
            }
            else if(event.type == event.MouseButtonPressed && event.mouseButton.button == sf::Mouse::Button::Right)
            {
                solver.addRectangle(20.f, 20.f, mouse_pos.x, mouse_pos.y, 100.f, false); 
            }
            else if(event.type == event.MouseButtonPressed && event.mouseButton.button == sf::Mouse::Button::Middle)
            {
                solver.addCircle(100.f, mouse_pos.x, mouse_pos.y, 20, 100.f, false, isSoft);
            }
        }

        ImGui::SFML::Update(window, deltaClock.restart());


        solver.update(solverClock.restart().asSeconds());

        ImGui::Text(std::to_string(current_fps).c_str());
        ImGui::PlotLines("FPS", fps.data(), fps.size());
        ImGui::Separator();
        ImGui::Text("Body count: "); ImGui::SameLine();
        ImGui::Text((std::to_string(solver.getPolygons().size())).c_str());
        ImGui::Separator();
        ImGui::SliderInt("Physcis Iterations count", &iterations, 1, 16);
        solver.set_iteration_count(iterations);
        ImGui::Separator();
        
        if(ImGui::Button("Soft circles or hard"))
        {
            isSoft = !isSoft;
        }
        ImGui::SameLine();
        ImGui::Text(isSoft ? "Soft" : "Hard");
        

        window.clear();
        renderer.render(solver);
        ImGui::SFML::Render(window);
        window.display();

        current_fps = 1.f / fpsClock.restart().asSeconds();

        if(fps.size() < 300)

            fps.push_back(current_fps);
        else
        {
            fps.erase(fps.begin());
            fps.push_back(current_fps);
        }
    }

    ImGui::SFML::Shutdown();

    return 0;
}