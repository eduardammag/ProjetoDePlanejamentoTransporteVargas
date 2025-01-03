#include "vertexAndEdge.h"
#include <random> // Para funcionalidades aleatórias, caso seja necessário em outras partes do código.

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
Edge::Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate, int idEdge, int id_zipCode, int id_street)
    : m_distance(distance),
      m_vertex1(vertex1),
      m_vertex2(vertex2),
      m_trafficRate(trafficRate),
      m_zipCode(id_zipCode),
      m_idEdge(idEdge),
      m_street(id_street)
    {
    // Cálculo do custo de escavação: por enquanto, estamos assumindo que a metade da distância é o custo.
        m_excavationCost = (distance / 2.0f);
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
int Edge::id_zipCode() const {
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

// Setter: Permite definir o custo de escavação.
void Edge::setExcavationCost(float cost) {
    m_excavationCost = cost;
}