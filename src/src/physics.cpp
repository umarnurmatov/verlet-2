#include "physics.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

Vertex::Vertex(float _x, float _y, float _mass, bool _fixed)
    : position      {_x,  _y},
      prev_position {_x,  _y},
      acceleration  {0.f, 0.f},
      mass          {_mass},
      fixed         {_fixed}
{
}

void Vertex::updateVerlet(float &dt)
{
    // Verlet integration
    if(fixed) return;
    Vector2f temp = position;
    position += position - prev_position + acceleration * dt * dt;
    prev_position = temp;
}

void Vertex::applyForce(Vector2f &force)
{
    acceleration = force / mass;
}

float Vertex::distance(Vertex *v2)
{
    return sqrt(pow(position.x - v2->position.x, 2) + pow(position.y - v2->position.y, 2));
}

void Edge::update()
{
    Vector2f v1v2 = v2->position - v1->position;
    float diff = v1v2.length() - length;
    v1v2 = v1v2.normalized();

    // push vertices apart by half of diff
    v1->position += v1v2 * diff * 0.5f;
    v2->position -= v1v2 * diff * 0.5f;
}

Edge::Edge(Vertex *_v1, Vertex *_v2, float _length, ConvexPolygon *_parent)
    : v1{_v1},
      v2{_v2},
      length{_length},
      parent{_parent}
{
}

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
    vertexes.push_back(Vertex(x,     y,     v_mass, fixed));
    vertexes.push_back(Vertex(x + w, y,     v_mass, fixed));
    vertexes.push_back(Vertex(x,     y + h, v_mass, fixed));
    vertexes.push_back(Vertex(x + w, y + h, v_mass, fixed));

    std::vector<Vertex*> v;
    for(auto& vertex : vertexes)
    {
        v.push_back(&vertex);
    }
        
    for(size_t i = 0; i < 4; i++)
        for(size_t j = i + 1; j < 4; j++)
            edges.push_back(Edge(v[i], v[j], v[i]->distance(v[j]), this));
}

void ConvexPolygon::makeTriangle(float a, float x, float y, float mass, bool fixed)
{
}

float Solver::intervalDistance(float minA, float maxA, float minB, float maxB)
{
    if(minA < minB)
        return minB - maxA;
    else
        return minA - maxB;
}

template<typename T> 
int Solver::sgn(T x)
{
    return (x > static_cast<T>(0)) - (x < static_cast<T>(0));
}

void Solver::updateVerlet(float dt)
{
    for(auto& b : m_bodies)
        for(auto& v : b.vertexes)
            v.updateVerlet(dt);

}

void Solver::updateEdges()
{
    for(auto& b : m_bodies)
        for(auto& e : b.edges)
            e.update();
}

void Solver::applyForces()
{
    for(auto& b : m_bodies)
        for(auto& v : b.vertexes)
            v.applyForce(m_gravity);
}

void Solver::applyConstrain()
{
    for(auto& body : m_bodies) {
        for(auto& v : body.vertexes) {
            v.position.x = std::max( std::min( v.position.x, (float)GWidth  ), 0.0f );
            v.position.y = std::max( std::min( v.position.y, (float)GHeight ), 0.0f );
        }
    }
}

// Separate axis theorem
bool Solver::detectCollision(ConvexPolygon *b1, ConvexPolygon *b2)
{
    float minDistance = std::numeric_limits<float>::max();  // init length of collision vector (relatively large value) 
    for(size_t i = 0; i < b1->edges.size() + b2->edges.size(); i++)
    {
        // choose the edge
        Edge* e;
        if(i < b1->edges.size()) 
            e = &b1->edges[i];
        else
            e = &b2->edges[i - b1->edges.size()];

        // axis perpendicular to the edge and normalized
        Vector2f axis = e->v2->position - e->v1->position;
        axis = axis.perpendicular().normalized();

        auto [minA, maxA] = b1->ProjectToAxis(axis);
        auto [minB, maxB] = b2->ProjectToAxis(axis);

        float distance = intervalDistance(minA, maxA, minB, maxB);

        if(distance > 0.f)  // no collision
            return false;

        // collision

        else if(std::abs(distance) < minDistance) // another shortest way to push bodies apart
        {
            minDistance = std::abs(distance);
            m_collisionInfo.normal = axis;
            m_collisionInfo.e = e;
        }
    }

    m_collisionInfo.depth = minDistance;

    // the next thing is to identify which vertex (and how) participates in collision
    // and separate it from others

    if(*m_collisionInfo.e->parent != *b2)
    {
        ConvexPolygon* temp = b2;
        b2 = b1;
        b1 = temp;
    }

    int sign = sgn(m_collisionInfo.normal.dot(b1->center - b2->center));

    // ensure that collision normal points to b1
    if(sign != 1)
        m_collisionInfo.normal = -m_collisionInfo.normal;
    
    float smallestDist = std::numeric_limits<float>::max();
    for(auto& v : b1->vertexes)
    {
        float distance = m_collisionInfo.normal.dot(v.position - b2->center);
        if(distance < smallestDist)
        {
            smallestDist = distance;
            m_collisionInfo.v = &v;
        }
    }
    return true; 
}

void Solver::resolveCollision()
{
    Vector2f collisionVector = m_collisionInfo.normal * m_collisionInfo.depth;

    Vertex* v1 = m_collisionInfo.e->v1;
    Vertex* v2 = m_collisionInfo.e->v2;

    float T; // T = (V - e->v1) / (e->v2 - e->v1); where on the edge lies collision vertex
    if(std::abs(v1->position.x - v2->position.x) > std::abs(v1->position.y - v2->position.y))
        T = (m_collisionInfo.v->position.x - collisionVector.x - v1->position.x) / (v2->position.x - v1->position.x);
    else
        T = (m_collisionInfo.v->position.y - collisionVector.y - v1->position.y) / (v2->position.y - v1->position.y);
    
    float lambda = 1.0f / (T*T + (1 - T)*(1 - T));


    // TODO понять что за хрень происходит
    if(!v1->fixed)
        v1->position -= 0.5f * collisionVector * (1 - T) * lambda;
    if(!v2->fixed)
        v2->position -= 0.5f * collisionVector *  T      * lambda;

    if(!m_collisionInfo.v->fixed)
        m_collisionInfo.v->position += collisionVector * 0.5f;

}

void Solver::iterateCollisions()
{
    //A small 'hack' that keeps the vertices inside the screen. You could of course implement static objects and create
    //four to serve as screen boundaries, but the max/min method is faster
    applyConstrain();
    
    updateEdges();

    for(auto& body : m_bodies)
    {
        body.calculateMassCenter_andBoundingBox();
    }

    for(auto& b1 : m_bodies)
    {
        for(auto& b2 : m_bodies)
        {
            if(b1 != b2)
                if(b1.boundBox.findIntersection(b2.boundBox) != std::nullopt)
                    if(detectCollision(&b1, &b2))
                        resolveCollision();
                    
        }
    }
}

void Solver::update(float dt)
{
    
    float sub_dt = dt / m_iterations;
    applyForces();
    for(size_t i = 0; i < m_iterations; i++)
    {
        updateVerlet(sub_dt);
        iterateCollisions();
    }
}

Solver::Solver()
    : m_gravity{0.f, 1000.f},
      m_iterations{8}
{
}

void Solver::addRectangle(float w, float h, float x, float y, float mass, bool fixed)
{
    m_bodies.push_back(ConvexPolygon());

    ConvexPolygon* body = &m_bodies[m_bodies.size() - 1];

    body->makeRectangle(w, h, x, y, mass, fixed);
}

std::vector<ConvexPolygon> &Solver::getPolygons()
{
    return m_bodies;
}
