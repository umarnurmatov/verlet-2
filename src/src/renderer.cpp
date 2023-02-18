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
    const auto& polygons = solver.getPolygons();
    const auto& vertexes = solver.getVertexes();
    const auto& edges = solver.getEdges();

    float default_radius = 5.f;
    float radius = default_radius;
    for(const auto &v : vertexes)
    {
        radius = v.radius == 0.f ? default_radius : v.radius;
        circle.setRadius(radius);
        circle.setOrigin(sf::Vector2f(radius, radius));
        circle.setPosition(v.position);
        circle.setFillColor(sf::Color::White);
        m_renderTarget->draw(circle);
    }

    for(const auto& polygon : polygons)
    {
        for(const auto v : polygon.vertexes)
        {
            radius = v.radius == 0.f ? default_radius : v.radius;
            circle.setRadius(radius);
            circle.setOrigin(sf::Vector2f(radius, radius));
            circle.setPosition(v.position);
            circle.setFillColor(sf::Color::White);
            m_renderTarget->draw(circle);
        }
    }

    sf::VertexArray line(sf::PrimitiveType::Lines, 2);
    for(const auto& edge : edges)
    {
        line[0] = sf::Vertex(edge.v1->position);
        line[1] = sf::Vertex(edge.v2->position);
        m_renderTarget->draw(line);
    }

    for(const auto& polygon : polygons)
    {
        for(const auto& edge : polygon.edges)
        {
            line[0] = sf::Vertex(edge.v1->position);
            line[1] = sf::Vertex(edge.v2->position);
            m_renderTarget->draw(line);
        }
    }
}
