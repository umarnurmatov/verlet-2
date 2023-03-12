//https://www.gamedev.net/articles/programming/math-and-physics/a-verlet-based-approach-for-2d-game-physics-r2714/
//https://github.com/rraallvv/VerletTutorial/
#pragma once
#include <SFML/System.hpp>
#include <SFML/Graphics/Rect.hpp>
#include <vector>
#include <list>
using sf::Vector2f;

struct Vertex;
struct Edge;
struct ConvexPolygon;

struct Vertex
{
    Vector2f position;
    Vector2f acceleration;
    Vector2f prev_position;
    float    mass;
    bool     fixed;
    

    Vertex(float _x, float _y, float _mass, bool _fixed = false);

    void updateVerlet(float &dt);
    void applyForce(Vector2f &force);
    float distance(Vertex *v2);
};

struct Edge
{
    Vertex *v1, *v2;
    float  length;
    bool   noCollision;

    ConvexPolygon* parent;

    void update();

    Edge(Vertex* _v1, Vertex* _v2, float _length, ConvexPolygon* _parent, bool _noCollision);
};


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

class Solver
{
    std::vector<ConvexPolygon> m_bodies;
    size_t                     m_iterations;
    Vector2f                   m_gravity;

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


