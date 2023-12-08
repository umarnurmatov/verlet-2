#include <iostream>

#include "constants.hpp"
#include "renderer.hpp"

Renderer::Renderer(sf::RenderTarget *rt)
    : m_renderTarget{rt}
    , m_line{sf::PrimitiveType::Lines, 2}
{
    m_circle.setPointCount(RENDERER_PRIMITIVE_CIRCLE_POINTS);
    m_circle.setRadius(RENDERER_PRIMITIVE_CIRCLE_RADIUS);
    m_circle.setOrigin(sf::Vector2f(RENDERER_PRIMITIVE_CIRCLE_RADIUS, RENDERER_PRIMITIVE_CIRCLE_RADIUS));
}

void Renderer::render(Solver &solver)
{
    const auto& polygons = solver.getPolygons();

    for(const auto& polygon : polygons)
    {
        for(const auto& vert : polygon.vertexes)
        {
            m_circle.setPosition(vert.position);
            m_circle.setFillColor(sf::Color::White);
            m_renderTarget->draw(m_circle);
        }
    }

    for(const auto& polygon : polygons)
    {
        for(const auto& edge : polygon.edges)
        {
            m_line[0] = sf::Vertex(edge.v1->position);
            m_line[1] = sf::Vertex(edge.v2->position);
            m_renderTarget->draw(m_line);
        }
    }
}
