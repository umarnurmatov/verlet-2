#include <algorithm>
#include <cmath>
#include <iostream>

#include "constants.hpp"
#include "solver.hpp"


Solver::Solver()
    : m_gravity{GRAVITY_VEC_X, GRAVITY_VEC_Y}
    , m_iterations{START_ITERATIONS}
    , m_thread_pool{SOLVER_THREADS}
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
    for(auto& body : m_bodies) 
    {
        for(auto& v : body.vertexes) 
        {
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

        if(e->noCollision) continue;

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

void Solver::addRectangle(float w, float h, float x, float y, float mass, bool fixed)
{
    m_bodies.push_back(ConvexPolygon());

    ConvexPolygon* body = &m_bodies[m_bodies.size() - 1];

    body->makeRectangle(w, h, x, y, mass, fixed);
}

void Solver::addTriangle(float a, float x, float y, float mass, bool fixed)
{
    m_bodies.push_back(ConvexPolygon());

    ConvexPolygon* body = &m_bodies[m_bodies.size() - 1];

    body->makeTriangle(a, x, y, mass, fixed);
}

void Solver::addCircle(float r, float x, float y, unsigned int resolution, float mass, bool fixed, bool isSoft)
{
    m_bodies.push_back(ConvexPolygon());

    ConvexPolygon* body = &m_bodies[m_bodies.size() - 1];

    isSoft ? body->makeSoftCircle(r, x, y, resolution, mass, fixed) : body->makeHardCircle(r, x, y, resolution, mass, fixed);
}

std::vector<ConvexPolygon> &Solver::getPolygons()
{
    return m_bodies;
}

