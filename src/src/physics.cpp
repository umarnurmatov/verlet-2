#include "physics.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

Vertex::Vertex(float _x, float _y, float _mass)
    : position{_x, _y},
      prev_position{_x, _y},
      acceleration{0.f, 0.f},
      mass{_mass}
{
}

float Vertex::distance(Vertex *v2)
{
    return sqrt(pow(position.x - v2->position.x, 2) + pow(position.y - v2->position.y, 2));
}

Edge::Edge(Vertex* _v1, Vertex* _v2, float _length, PhysicsBody* _parent)
    : v1{_v1},
      v2{_v2},
      length{_length},
      parent{_parent}
{
}

std::pair<int, int> PhysicsBody::ProjectToAxis(Vector2f &axis) const
{
    float min, max;
    float dot = axis.dot(vertexes[0].position);
    min = max = dot;

    for(size_t i = 1; i < vertexes.size(); i++)
    {
        dot = axis.dot(vertexes[i].position);
        min = std::min(min, dot);
        max = std::max(max, dot);
    }

    return std::make_pair(min, max);
}

void PhysicsBody::calculateMassCenter_andBoundingBox()
{
    center = Vector2f(0.f, 0.f);
    mass = 0.f;
    for(auto& v : vertexes)
    {
        center += v.position * v.mass;
        mass += v.mass;
    }
    center /= mass;
}

bool operator==(const Vertex &v1, const Vertex &v2)
{
    return (v1.position == v2.position) && (v1.prev_position == v2.prev_position);
}

bool PhysicsBody::operator==(PhysicsBody &b)
{
    return vertexes == b.vertexes;
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
    for(Vertex* v : m_vertexes)
    {
        // Verlet integration
        Vector2f temp = v->position;
        v->position += v->position - v->prev_position + v->acceleration * dt * dt;
        v->prev_position = temp;
    }

}

void Solver::updateEdges()
{
    for(Edge* edge : m_edges)
    {
        Vector2f v1v2 = edge->v2->position - edge->v1->position;
        float diff = v1v2.length() - edge->length;
        v1v2 = v1v2.normalized();

        // push vertices apart by half of diff
        edge->v1->position += v1v2 * diff * 0.5f;
        edge->v2->position -= v1v2 * diff * 0.5f;
    }
}

void Solver::applyForces()
{
    for(Vertex* v : m_vertexes)
    {
        v->acceleration = m_gravity;
    }
}

// Separate axis theorem
bool Solver::detectCollision(PhysicsBody &b1, PhysicsBody &b2)
{
    float minDistance = std::numeric_limits<float>::max();  // init length of collision vector (relatively large value) 
    for(size_t i = 0; i < b1.edges.size() + b2.edges.size(); i++)
    {
        // choose the edge
        Edge* e;
        if(i < b1.edges.size()) 
            e = &b1.edges[i];
        else
            e = &b2.edges[i - b1.edges.size()];

        // axis perpendicular to the edge and normalized
        Vector2f axis = e->v2->position - e->v1->position;
        axis = axis.perpendicular().normalized();

        auto [minA, maxA] = b1.ProjectToAxis(axis);
        auto [minB, maxB] = b2.ProjectToAxis(axis);

        float distance = intervalDistance(minA, maxA, minB, maxB);
        std::cout << "dist " << distance << std::endl;
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

    if(*m_collisionInfo.e->parent != b2)
    {
        PhysicsBody& temp = b2;
    }

    int sign = sgn(m_collisionInfo.normal.dot(b1.center - b2.center));

    // ensure that collision normal points to b1
    if(sign != 1)
        m_collisionInfo.normal = -m_collisionInfo.normal;
    
    float smallestDist = std::numeric_limits<float>::max();
    for(auto& v : b1.vertexes)
    {
        float distance = m_collisionInfo.normal.dot(v.position - b2.center);
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

    v1->position -= 0.5f * collisionVector * (1 - T) * lambda;
    v2->position -= 0.5f * collisionVector * T       * lambda;

    m_collisionInfo.v->position += collisionVector * 0.5f;

}

void Solver::iterateCollisions()
{
    //A small 'hack' that keeps the vertices inside the screen. You could of course implement static objects and create
    //four to serve as screen boundaries, but the max/min method is faster
    for(Vertex* v : m_vertexes) {
        v->position.x = std::max( std::min( v->position.x, (float)GWidth  ), 0.0f );
        v->position.y = std::max( std::min( v->position.y, (float)GHeight ), 0.0f );
    }

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
            {
                if(detectCollision(b1, b2))
                {
                    resolveCollision();
                }
            }
        }
    }
}

void Solver::update(float dt)
{
    
    float sub_dt = dt / m_iterations;
    for(size_t i = 0; i < m_iterations; i++)
    {
        applyForces();
        updateVerlet(sub_dt);
        iterateCollisions();
    }
}

Solver::Solver()
    : m_gravity{0.f, 1000.f},
      m_iterations{4}
{
}

void Solver::addRectangle(float w, float h, float x, float y)
{
    float mass = 1.f;

    m_bodies.push_back(PhysicsBody());

    PhysicsBody* body = &m_bodies[m_bodies.size() - 1];

    body->vertexes.push_back(Vertex(x, y, mass));
    body->vertexes.push_back(Vertex(x + w, y, mass));
    body->vertexes.push_back(Vertex(x, y + h, mass));
    body->vertexes.push_back(Vertex(x + w, y + h, mass));

    size_t s = body->vertexes.size();
    Vertex* v1 = &body->vertexes[s - 4];
    Vertex* v2 = &body->vertexes[s - 3];
    Vertex* v3 = &body->vertexes[s - 2];
    Vertex* v4 = &body->vertexes[s - 1];

    body->edges.push_back(Edge(v1, v2, v1->distance(v2), body));
    body->edges.push_back(Edge(v2, v3, v2->distance(v3), body));
    body->edges.push_back(Edge(v3, v4, v3->distance(v4), body));
    body->edges.push_back(Edge(v4, v1, v4->distance(v1), body));
    body->edges.push_back(Edge(v1, v3, v1->distance(v3), body));
    body->edges.push_back(Edge(v2, v4, v2->distance(v4), body));

    for(size_t i = 4; i >= 1; i--)
    {
        m_vertexes.push_back(&body->vertexes[s - i]);    
    }

    s = body->edges.size();
    for(size_t i = 6; i >= 1; i--)
    {
        m_edges.push_back(&body->edges[s - i]);
    }
    
}

std::vector<Vertex*> &Solver::getVertexes()
{
    return m_vertexes;
}

std::vector<Edge*> &Solver::getEdges()
{
    return m_edges;
}