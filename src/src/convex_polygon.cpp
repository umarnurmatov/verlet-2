#include "convex_polygon.hpp"

std::pair<float, float> ConvexPolygon::ProjectToAxis(Vector2f &axis) const
{
    float min, max;
    float dot = axis.dot((*vertexes.begin()).position);
    min = max = dot;

    for(auto& vertex : vertexes)
    {
        dot = axis.dot(vertex.position);
        min = std::min(min, dot);
        max = std::max(max, dot);
    }

    return std::make_pair(min, max);
}

void ConvexPolygon::calculateMassCenter_andBoundingBox()
{
    center = Vector2f(0.f, 0.f);
    mass = 0.f;

    Vector2f min, max;

    for(auto& v : vertexes)
    {
        min.x = std::min(min.x, v.position.x);
        min.y = std::min(min.y, v.position.y);
        max.x = std::max(max.x, v.position.x);
        max.y = std::max(max.y, v.position.y);
        

        center += v.position * v.mass;
        mass += v.mass;
    }

    boundBox = sf::FloatRect(min, max - min);

    center /= mass;
}

bool operator==(const Vertex &v1, const Vertex &v2)
{
    return (v1.position == v2.position) && (v1.prev_position == v2.prev_position);
}

bool ConvexPolygon::operator==(ConvexPolygon &b)
{
    return vertexes == b.vertexes;
}

void ConvexPolygon::makeRectangle(float w, float h, float x, float y, float mass, bool fixed)
{
    float v_mass = mass / 4.f;
    auto v1 = &vertexes.emplace_back(Vertex(x,     y,     v_mass, fixed));
    auto v2 = &vertexes.emplace_back(Vertex(x + w, y,     v_mass, fixed));
    auto v3 = &vertexes.emplace_back(Vertex(x,     y + h, v_mass, fixed));
    auto v4 = &vertexes.emplace_back(Vertex(x + w, y + h, v_mass, fixed));
        
    edges.push_back(Edge(v1, v2, v1->distance(v2), this, false));
    edges.push_back(Edge(v2, v4, v2->distance(v4), this, false));
    edges.push_back(Edge(v3, v4, v3->distance(v4), this, false));
    edges.push_back(Edge(v1, v3, v1->distance(v3), this, false));
    edges.push_back(Edge(v1, v4, v1->distance(v4), this, true));
    edges.push_back(Edge(v2, v3, v2->distance(v3), this, true));
}

void ConvexPolygon::makeTriangle(float a, float x, float y, float mass, bool fixed)
{
    static float x1 = a * cos(M_PI / 6.f);
    static float x2 = a * sin(M_PI / 6.f);
    float v_mass = mass / 3.f;

    auto v1 = &vertexes.emplace_back(Vertex(x + x2,  y,      v_mass, fixed));
    auto v2 = &vertexes.emplace_back(Vertex(x + a,   y + x1, v_mass, fixed));
    auto v3 = &vertexes.emplace_back(Vertex(x,       y + x1, v_mass, fixed));

    edges.push_back(Edge(v1, v2, v1->distance(v2), this, false));
    edges.push_back(Edge(v2, v3, v2->distance(v3), this, false));
    edges.push_back(Edge(v1, v3, v1->distance(v3), this, false));
}

void ConvexPolygon::makeSoftCircle(float r, float x, float y, unsigned int resolution, float mass, bool fixed)
{
    float v_mass = mass / static_cast<float>(resolution);
    unsigned int res = static_cast<unsigned int>(resolution / 2);
    float da = 2.f * M_PI / static_cast<float>(resolution);
    float a = 0.f;
    
    Vector2f vv0(x, y);
    Vector2f vv1, vv2;
    Vertex* prev_v1 = nullptr, *prev_v2 = nullptr;
    for(size_t i = 0; i < res; i++)
    {
        vv1.x = cos(a); vv1.y = sin(a);
        vv1 = vv1 * r;
        vv2 = - vv1;
        vv1 += vv0;
        vv2 += vv0;
        auto v1 = &vertexes.emplace_back(Vertex(vv1.x, vv1.y, v_mass, fixed));
        auto v2 = &vertexes.emplace_back(Vertex(vv2.x, vv2.y, v_mass, fixed));
        edges.push_back(Edge(v1, v2, 2 * r, this, true));
        if(prev_v1) edges.push_back(Edge(v1, prev_v1, v1->distance(prev_v1), this, false));
        if(prev_v2) edges.push_back(Edge(v2, prev_v2, v2->distance(prev_v2), this, false));
        prev_v1 = v1; prev_v2 = v2;
        a += da;
    }

    auto vbegin = vertexes.begin();
    auto vend = --vertexes.end(); 

    auto* start1 = &(*(  vbegin ));
    auto* start2 = &(*(++vbegin ));
    auto* end1 =   &(*(  vend   ));
    auto* end2 =   &(*(--vend   ));

    edges.push_back(Edge(start1, end1, start1->distance(end1), this, false));
    edges.push_back(Edge(start2, end2, start2->distance(end2), this, false));
}

/// @param resolution must be even - number of vertexes in circle
void ConvexPolygon::makeHardCircle(float r, float x, float y, unsigned int resolution, float mass, bool fixed)
{
    float v_mass = mass / static_cast<float>(resolution);
    unsigned int res = static_cast<unsigned int>(resolution / 2);
    float da = 2.f * M_PI / static_cast<float>(resolution);
    float a = 0.f;
    
    Vector2f vv0(x, y);
    Vector2f vv1, vv2;
    Vertex* prev_v1 = nullptr, *prev_v2 = nullptr;
    for(size_t i = 0; i < res; i++)
    {
        vv1.x = cos(a); vv1.y = sin(a);
        vv1 = vv1 * r;
        vv2 = - vv1;
        vv1 += vv0;
        vv2 += vv0;
        auto v1 = &vertexes.emplace_back(Vertex(vv1.x, vv1.y, v_mass, fixed));
        auto v2 = &vertexes.emplace_back(Vertex(vv2.x, vv2.y, v_mass, fixed));
        //edges.push_back(Edge(v1, v2, 2 * r, this, true));
        if(prev_v1) edges.push_back(Edge(v1, prev_v1, v1->distance(prev_v1), this, false));
        if(prev_v2) edges.push_back(Edge(v2, prev_v2, v2->distance(prev_v2), this, false));
        prev_v1 = v1; prev_v2 = v2;
        a += da;
    }

    auto vbegin = vertexes.begin();
    auto vend = --vertexes.end(); 

    auto* start1 = &(*(  vbegin ));
    auto* start2 = &(*(++vbegin ));
    auto* end1 =   &(*(  vend   ));
    auto* end2 =   &(*(--vend   ));

    edges.push_back(Edge(start1, end1, start1->distance(end1), this, false));
    edges.push_back(Edge(start2, end2, start2->distance(end2), this, false));

    for(auto& v : vertexes)
        for(auto& vv : vertexes)
            if(v != vv) edges.push_back(Edge(&v, &vv, v.distance(&vv), this, true));
}
