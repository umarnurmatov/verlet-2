#pragma once
#include "vertex.hpp"
#include "convex_polygon.hpp"

using sf::Vector2f;
struct ConvexPolygon;

struct Edge
{
    Vertex *v1, *v2;
    float  length;
    bool   noCollision;

    ConvexPolygon* parent;

    Edge(Vertex *_v1, Vertex *_v2, float _length, ConvexPolygon *_parent, bool _noCollision)
        : v1{_v1}
        , v2{_v2}
        , length{_length}
        , parent{_parent}
        , noCollision{_noCollision}
    {
    }

    void update()
    {
        Vector2f v1v2 = v2->position - v1->position;
        float diff = v1v2.length() - length;
        v1v2 = v1v2.normalized();

        // push vertices apart by half of diff
        v1->position += v1v2 * diff * 0.5f;
        v2->position -= v1v2 * diff * 0.5f;
    }
};

