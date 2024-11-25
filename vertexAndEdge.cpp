#include "vertexAndEdge.h"
#include <random>

using namespace std; 

//classe Vertex
Vertex::Vertex(bool isMetroStation, int id)
    : m_isMetroStation(isMetroStation), m_id(id) {}


    // Setter para metro
    void Vertex::setMetroStation(bool isMetroStation) 
    {
        m_isMetroStation = isMetroStation;
    }

    bool Vertex::isMetroStation() const 
    {
        return m_isMetroStation;
    }
    int Vertex::id() const 
    {
        return m_id;
    }

//classe Edge
Edge::Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate, int idEdge, int id_zipCode)
    : m_distance(distance),
      m_vertex1(vertex1),
      m_vertex2(vertex2),
      m_trafficRate(trafficRate),
      m_zipCode(id_zipCode),
      m_idEdge(idEdge) 
{
    m_excavationCost = distance / 2.0f; // Define o custo de escavação como metade da distância
}

//Getters
Vertex* Edge::vertex1() const
{
    return m_vertex1;
}

Vertex* Edge::vertex2() const
{
    return m_vertex2;
}

int Edge::distance() const
{
    return m_distance;
}

int Edge::id_zipCode()
{
    return m_zipCode;
}

int Edge::id_street() const
{
    return m_street;
}

float Edge::trafficRate() const
{
    return m_trafficRate;
}

float Edge::excavationCost() const
{
    return m_excavationCost;
}

int Edge::idEdge() const
{
    return m_idEdge;
}
