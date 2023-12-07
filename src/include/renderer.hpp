#pragma once
#include "solver.hpp"
#include "SFML/Graphics.hpp"

class Renderer
{
    sf::RenderTarget *m_renderTarget;
public:
    Renderer(sf::RenderTarget *rt);
    void render(Solver& solver);
};