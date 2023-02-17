//https://www.gamedev.net/articles/programming/math-and-physics/a-verlet-based-approach-for-2d-game-physics-r2714/
//https://github.com/rraallvv/VerletTutorial/
#pragma once
#include <SFML/System.hpp>
#include <vector>
using sf::Vector2f;

struct Vertex;
struct Edge;
struct PhysicsBody;

struct Vertex
{
    Vector2f position;
    Vector2f acceleration;
    Vector2f prev_position;
    float mass;

    Vertex(float _x, float _y, float _mass);
    float distance(Vertex *v2);
};

struct Edge
{
    Vertex *v1, *v2;
    float length;

    PhysicsBody* parent;
public:
    Edge(Vertex* _v1, Vertex* _v2, float _length, PhysicsBody* _parent);
};

struct PhysicsBody
{
    Vector2f center; // center of mass
    float mass;

    std::vector<Vertex> vertexes;
    std::vector<Edge> edges;

    std::pair<int, int> ProjectToAxis(Vector2f &axis) const;
    void calculateMassCenter();

    bool operator==(PhysicsBody &b);
};

class Solver
{
    std::vector<Vertex> m_vertexes;
    std::vector<Edge> m_edges;
    std::vector<PhysicsBody> m_bodies;
    size_t m_iterations = 8;
    Vector2f m_gravity;

    // TODO fix this
    int GWidth = 1920, GHeight = 600;

    struct {
        float depth;
        Vector2f normal; 
        Edge* e; // collision edge
        Vertex* v;
    } m_collisionInfo;

    // distance between two intervals
    float intervalDistance(float minA, float maxA, float minB, float maxB);

    // sign(x)
    template<typename T> int sgn(T x);


    void updateVerlet(float dt);
    void updateEdges();
    void applyForces();
    bool detectCollision(PhysicsBody &b1, PhysicsBody &b2);
    void resolveCollision();
    void iterateCollisions();
    
public:
    Solver();
    void update(float dt);
    void addRectangle(float w, float h, float x, float y);
    std::vector<Vertex>& getVertexes();
    std::vector<Edge>& getEdges();
    std::vector<PhysicsBody>& getBodies();
};


