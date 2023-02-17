#include "renderer.hpp"
#include <iostream>
using namespace std;

Renderer::Renderer(sf::RenderTarget *rt)
    : m_renderTarget{rt}
{
}

void Renderer::render(Solver &solver)
{
    sf::CircleShape circle;
    circle.setPointCount(64);
    const auto& vertexes = solver.getVertexes();
    float radius = 5.f;

    for(const Vertex* v : vertexes)
    {
        circle.setRadius(radius);
        circle.setOrigin(sf::Vector2f(radius, radius));
        circle.setPosition(v->position);
        circle.setFillColor(sf::Color::White);
        m_renderTarget->draw(circle);
    }

    sf::VertexArray line(sf::PrimitiveType::Lines, 2);
    const auto& edges = solver.getEdges();
    for(const Edge* edge : edges)
    {
        line[0] = sf::Vertex(edge->v1->position);
        line[1] = sf::Vertex(edge->v2->position);
        m_renderTarget->draw(line);
    }
}
