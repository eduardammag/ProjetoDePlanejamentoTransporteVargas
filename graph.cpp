#include "Graph.h"
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
Edge::Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate)
    : m_distance(distance),
      m_vertex1(vertex1),
      m_vertex2(vertex2),
      m_trafficRate(trafficRate) {
    m_excavationCost = distance / 2.0f; // Define o custo de escavação como metade da distância
    m_zipCode = generateRandomCode(4); // Gera um código postal aleatório de 4 dígitos
    m_street = generateRandomCode(8);  // Gera um identificador de rua aleatório de 8 dígitos
}

// Método privado para gerar um código aleatório com um número fixo de dígitos
int Edge::generateRandomCode(int numDigits) {
    random_device rd; // Gera uma semente para o gerador de números aleatórios
    mt19937 gen(rd()); // Motor de geração de números pseudo-aleatórios
    uniform_int_distribution<int> dist(pow(10, numDigits - 1), pow(10, numDigits) - 1); // Intervalo de valores
    return dist(gen); // Retorna o número aleatório gerado
}

// Método que retorna o primeiro vértice conectado pela aresta
Vertex* Edge::vertex1() const {
    return m_vertex1;
}

// Método que retorna o segundo vértice conectado pela aresta
Vertex* Edge::vertex2() const {
    return m_vertex2;
}

// Método que retorna a distância entre os vértices
int Edge::distance() const {
    return m_distance;
}

// Método que retorna o código postal gerado para a aresta
int Edge::id_zipCode() const {
    return m_zipCode;
}

// Método que retorna o identificador da rua gerado para a aresta
int Edge::id_street() const {
    return m_street;
}

// Método que retorna a taxa de tráfego na aresta
float Edge::trafficRate() const {
    return m_trafficRate;
}

// Método que retorna o custo estimado de escavação para a aresta
float Edge::excavationCost() const {
    return m_excavationCost;
}
