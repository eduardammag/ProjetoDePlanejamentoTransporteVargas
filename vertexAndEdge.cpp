#include "vertexAndEdge.h"
#include <random> // Incluído para possíveis funcionalidades relacionadas a valores aleatórios.

using namespace std;

// Implementação da classe Vertex.

// Construtor: Inicializa o vértice com o estado de estação de metrô e o ID.
Vertex::Vertex(bool isMetroStation, int id)
    : m_isMetroStation(isMetroStation), m_id(id) {}

// Setter: Define se o vértice é uma estação de metrô.
void Vertex::setMetroStation(bool isMetroStation) {
    m_isMetroStation = isMetroStation;
}

// Getter: Retorna se o vértice é uma estação de metrô.
bool Vertex::isMetroStation() const {
    return m_isMetroStation;
}

// Getter: Retorna o identificador único do vértice.
int Vertex::id() const {
    return m_id;
}

// Implementação da classe Edge.

// Construtor: Inicializa a aresta com os parâmetros fornecidos.
Edge::Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate, int idEdge, int id_zipCode)
    : m_distance(distance),
      m_vertex1(vertex1),
      m_vertex2(vertex2),
      m_trafficRate(trafficRate),
      m_zipCode(id_zipCode),
      m_idEdge(idEdge) 
{
    m_excavationCost = distance / 2.0f; // Define o custo de escavação como metade da distância.
}

// Getter: Retorna o ponteiro para o primeiro vértice.
Vertex* Edge::vertex1() const {
    return m_vertex1;
}

// Getter: Retorna o ponteiro para o segundo vértice.
Vertex* Edge::vertex2() const {
    return m_vertex2;
}

// Getter: Retorna a distância entre os dois vértices.
int Edge::distance() const {
    return m_distance;
}

// Getter: Retorna o código postal associado à aresta.
int Edge::id_zipCode() {
    return m_zipCode;
}

// Getter: Retorna o ID da rua associada à aresta.
int Edge::id_street() const {
    return m_street;
}

// Getter: Retorna a taxa de tráfego na aresta.
float Edge::trafficRate() const {
    return m_trafficRate;
}

// Getter: Retorna o custo de escavação da aresta.
float Edge::excavationCost() const {
    return m_excavationCost;
}

// Getter: Retorna o identificador único da aresta.
int Edge::idEdge() const {
    return m_idEdge;
}
