#ifndef VERTEXANDEDGE_H
#define VERTEXANDEDGE_H

#include <unordered_map>

// Classe Vertex

class Vertex {
private:
    bool m_isMetroStation; 
    int m_id;

public:
    // Construtor
    Vertex(bool isMetroStation, int id);

    // Setter para m_isMetroStation
    void setMetroStation(bool isMetroStation);

    // Getters
    bool isMetroStation() const;
    int id() const;
};

// Classe Edge
class Edge {
private:
    // Atributos privados
    int m_idEdge;
    Vertex* m_vertex1;
    Vertex* m_vertex2;
    int m_distance;
    int m_zipCode;
    int m_street;
    float m_trafficRate;
    float m_excavationCost;

public:
    // Construtor
    Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate, int idEdge, int id_zipCode);

    // Getters 
    Vertex* vertex1() const;
    Vertex* vertex2() const;
    int distance() const;
    int id_street() const;
    float trafficRate() const;
    float excavationCost() const;
    int idEdge() const;
    int id_zipCode();
};

#endif
