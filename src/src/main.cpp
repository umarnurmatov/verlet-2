#include <iostream>
#include <chrono>

#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include "solver.hpp"
#include "renderer.hpp"
#include "constants.hpp"

using namespace std;

std::vector<float> fps;

int main() {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 4;
    sf::RenderWindow window(sf::VideoMode::getFullscreenModes()[0], "Verlet-2", sf::Style::Default, settings);
    // window.setFramerateLimit(60);
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
    int iterations = START_ITERATIONS;
    bool isSoft = false;

    ImGuiIO io;
    
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            io = ImGui::GetIO();

            if(!io.WantCaptureKeyboard)
                if (event.type == sf::Event::Closed || (event.key.code == sf::Keyboard::Q && event.key.control)) 
                    window.close();
                
            if(!io.WantCaptureMouse)
            {
                sf::Vector2f mouse_pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                if (event.type == event.MouseButtonPressed && event.mouseButton.button == sf::Mouse::Button::Left)
                {
                    solver.addTriangle(20.f, mouse_pos.x, mouse_pos.y, 100.f, false);
                }
                else if (event.type == event.MouseButtonPressed && event.mouseButton.button == sf::Mouse::Button::Right)
                {
                    solver.addRectangle(20.f, 20.f, mouse_pos.x, mouse_pos.y, 100.f, false);
                }
                else if (event.type == event.MouseButtonPressed && event.mouseButton.button == sf::Mouse::Button::Middle)
                {
                    solver.addCircle(20.f, mouse_pos.x, mouse_pos.y, 20, 100.f, false, isSoft);
                }
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
        ImGui::SliderInt("Physcis Iterations count", &iterations, MIN_ITERATIONS, MAX_ITERATIONS);
        solver.set_iteration_count(iterations);
        ImGui::Separator();
        
        if(ImGui::Button("Soft circles or hard")) isSoft = !isSoft;
        ImGui::SameLine();
        ImGui::Text(isSoft ? "Soft" : "Hard");

        static bool spam_circles = false, spam_rect = false;
        static auto timer_circles = std::chrono::steady_clock::now();
        static auto timer_rect = std::chrono::steady_clock::now();
        ImGui::Checkbox("Spam hard circles", &spam_circles);
        ImGui::Checkbox("Spam rectangles", &spam_rect);
        if(spam_circles && (std::chrono::steady_clock::now() - timer_circles).count() > 10e7)
        {
            solver.addCircle(20.f, solver.getWidth()/2.f, solver.getHeight()/2.f, 20, 100.f, false, false);
            timer_circles = std::chrono::steady_clock::now();
        }
        if(spam_rect && (std::chrono::steady_clock::now() - timer_rect).count() > 10e7)
        {
            solver.addRectangle(20.f, 20.f, solver.getWidth()/2.f, solver.getHeight()/2.f, 100.f, false); 
            timer_rect = std::chrono::steady_clock::now();
        }

        

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