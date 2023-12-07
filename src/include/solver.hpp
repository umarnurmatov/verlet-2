//https://www.gamedev.net/articles/programming/math-and-physics/a-verlet-based-approach-for-2d-game-physics-r2714/
//https://github.com/rraallvv/VerletTutorial/
#pragma once
#include <vector>

#include <SFML/System.hpp>
#include <SFML/Graphics/Rect.hpp>

#include "thread_pool.hpp"
#include "convex_polygon.hpp"

using sf::Vector2f;

class Solver
{
    std::vector<ConvexPolygon> m_bodies;
    size_t                     m_iterations;
    Vector2f                   m_gravity;

    ThreadPool thread_pool;

    // TODO fix this
    int GWidth = 1920, GHeight = 600;

    struct 
    {
        float    depth;
        Vector2f normal; // depth * normal = collisionVector
        Edge*    e;      // collision edge
        Vertex*  v;      // collision vertex
    } m_collisionInfo;

    // distance between two intervals
    float intervalDistance(float minA, float maxA, float minB, float maxB);

    // sign(x)
    template<typename T> int sgn(T x);


    void updateVerlet(float dt);
    void updateEdges();
    void applyForces();
    void applyConstrain();
    bool detectCollision(ConvexPolygon *b1, ConvexPolygon *b2);
    void resolveCollision();
    void iterateCollisions();
    
public:
    Solver();
    void update(float dt);
    void addRectangle(float w, float h, float x, float y, float mass, bool fixed = false);
    void addTriangle(float a, float x, float y, float mass, bool fixed);
    void addCircle(float r, float x, float y, unsigned int resolution, float mass, bool fixed, bool isSoft);
    void set_iteration_count(size_t iterations) { m_iterations = iterations; }


    std::vector<ConvexPolygon>& getPolygons();
};