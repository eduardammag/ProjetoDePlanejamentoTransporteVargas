#ifndef VERTEXANDEDGE_H
#define VERTEXANDEDGE_H

#include <unordered_map>

// Classe Vertex: Representa um vértice (ou nó) no grafo.
class Vertex {
public:
    bool m_isMetroStation; // Indica se o vértice é uma estação de metrô.
    int m_id; // Identificador único do vértice.

    // Construtor: Inicializa um vértice com a informação se é uma estação de metrô e seu ID.
    Vertex(bool isMetroStation, int id);

    // Setter: Altera o valor de m_isMetroStation para indicar se é uma estação de metrô.
    void setMetroStation(bool isMetroStation);

    // Getter: Retorna se o vértice é uma estação de metrô.
    bool isMetroStation() const;

    // Getter: Retorna o identificador do vértice.
    int id() const;
};

// Classe Edge: Representa uma aresta (ou ligação) entre dois vértices no grafo.
class Edge {
private:
    int m_idEdge; // Identificador único da aresta.
    Vertex* m_vertex1; // Ponteiro para o primeiro vértice da aresta.
    Vertex* m_vertex2; // Ponteiro para o segundo vértice da aresta.
    int m_distance; // Distância entre os dois vértices.
    int m_zipCode; // Código postal associado à aresta.
    int m_street; // ID da rua associada à aresta (se necessário, adicione um getter/setter).
    float m_trafficRate; // Taxa de tráfego na aresta.
    float m_excavationCost; // Custo de escavação para construir ou modificar a aresta.

public:
    // Construtor: Inicializa uma aresta com os parâmetros necessários.
    Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate, int idEdge, int id_zipCode, int id_street);

    // Getter: Retorna o ponteiro para o primeiro vértice.
    Vertex* vertex1() const;

    // Getter: Retorna o ponteiro para o segundo vértice.
    Vertex* vertex2() const;

    // Getter: Retorna a distância entre os vértices.
    int distance() const;

    // Getter: Retorna o ID da rua associada à aresta.
    int id_street() const;

    // Getter: Retorna a taxa de tráfego na aresta.
    float trafficRate() const;

    // Getter: Retorna o custo de escavação.
    float excavationCost() const;

    // Setter: Permite definir o custo de escavação da aresta.
    void setExcavationCost(float cost);

    // Getter: Retorna o identificador único da aresta.
    int idEdge() const;

    // Getter: Retorna o código postal associado à aresta.
    int id_zipCode() const;
};

#endif