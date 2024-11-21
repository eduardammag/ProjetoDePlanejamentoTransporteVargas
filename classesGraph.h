#include <random>
#include <cmath>

// Classe Vertex
class Vertex {
public:
    bool m_isMetroStation;
    int m_id;

    // Construtor
    Vertex(bool isMetroStation, int id);

    // Métodos de acesso
    bool isMetroStation() const;
    int id() const;
};

// Classe Edge
class Edge {
public:
    Vertex* m_vertex1;
    Vertex* m_vertex2;
    int m_distance;
    int m_zipCode;
    int m_street;
    float m_trafficRate;
    float m_excavationCost;

    int generateRandomCode(int numDigits);

    // Construtor
    Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate);

    // Métodos de acesso
    Vertex* vertex1() const;
    Vertex* vertex2() const;
    int distance() const;
    int id_zipCode() const;
    int id_street() const;
    float trafficRate() const;
    float excavationCost() const;
};

