#pragma once
#include <list>

#include <SFML/System.hpp>

#include "vertex.hpp"
#include "edge.hpp"

using sf::Vector2f;

// Consists of several Vertexes (they are acting like points) and edges between them
struct ConvexPolygon
{
    Vector2f center; // center of mass
    float mass;
    sf::FloatRect boundBox;

    std::list<Vertex> vertexes;
    std::vector<Edge> edges;

    // axis NEED to be normalized
    std::pair<float, float> ProjectToAxis(Vector2f &axis) const;
    void calculateMassCenter_andBoundingBox();

    bool operator==(ConvexPolygon &b);

    void makeRectangle(float w, float h, float x, float y, float mass, bool fixed);
    void makeTriangle(float a, float x, float y, float mass, bool fixed);
    void makeSoftCircle(float r, float x, float y, unsigned int resolution, float mass, bool fixed);
    void makeHardCircle(float r, float x, float y, unsigned int resolution, float mass, bool fixed);
};