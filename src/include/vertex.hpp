#pragma once
#include <cmath>
#include <SFML/Graphics/Rect.hpp>

using sf::Vector2f;

struct Vertex
{
    Vector2f position;
    Vector2f acceleration;
    Vector2f prev_position;
    float    mass;
    bool     fixed;

    Vertex(float _x, float _y, float _mass, bool _fixed)
        : position{_x, _y}
        , prev_position{_x, _y}
        , acceleration{0.f, 0.f}
        , mass{_mass}
        , fixed{_fixed}
    {
    }

    void updateVerlet(float &dt)
    {
        // Verlet integration
        if (fixed) return;
        Vector2f temp = position;
        position += position - prev_position + acceleration * dt * dt;
        prev_position = temp;
    }

    void applyForce(Vector2f &force)
    {
        acceleration = force / mass;
    }

    float distance(Vertex *v2)
    {
        return (position - v2->position).length();
    }
};
