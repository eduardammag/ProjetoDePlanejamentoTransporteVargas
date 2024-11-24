#include "vertexAndEdge.h"
#include <random> // Para o gerador de números aleatórios
#include <cmath>  // Para a função pow

using namespace std; 

// Implementação da classe Vertex

// Construtor da classe Vertex
// Inicializa os atributos com os valores passados
Vertex::Vertex(bool isMetroStation, int id)
    : m_isMetroStation(isMetroStation), m_id(id) {}

// Método que retorna se o vértice é uma estação de metrô
bool Vertex::isMetroStation() const {
    return m_isMetroStation;
}

// Método que retorna o ID do vértice
int Vertex::id() const {
    return m_id;
}

// Implementação da classe Edge

// Construtor da classe Edge
// Inicializa os atributos e calcula o custo de escavação e códigos aleatórios
Edge::Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate, int idEdge)
    : m_distance(distance),
      m_vertex1(vertex1),
      m_vertex2(vertex2),
      m_trafficRate(trafficRate),
      m_idEdge(idEdge) {
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

int Edge::id_zipCode() const
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