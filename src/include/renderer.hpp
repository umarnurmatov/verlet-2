#pragma once
#include "solver.hpp"
#include "SFML/Graphics.hpp"

class Renderer
{
public:
    Renderer(sf::RenderTarget *rt);
    Renderer(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;

    void render(Solver& solver);

private:
    sf::RenderTarget *m_renderTarget;
};