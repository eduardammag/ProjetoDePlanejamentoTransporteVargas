#include "graph.h"
#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

// PUBLICOS
// Getters
vector<vector<Edge*>>& Graph::adjacencyMatrix()
{
   // Se ainda não gerou, gerar agora
    if (m_adjacencyMatrix.empty()) 
    {
        generateAdjacencyMatrix();
    }
    return m_adjacencyMatrix;
}

vector<list<tuple<int, Edge*>>>& Graph::adjacencyList()
{
    if (m_adjacencyList.empty()) 
    {
        generateAdjacencyList(); 
    }
    return m_adjacencyList;
}

const vector<Vertex*>& Graph::vertices()
{
    return m_vertices;
}

const vector<Edge*>& Graph::edges()
{
    return m_edges;
}

//Adiciona vértice ao grafo, o qual possui duas informações: seu id e se é estação de metrô
void Graph::addVertex(bool isMetroStation, int id) 
{
    m_vertices.push_back(new Vertex(isMetroStation, id));
}

//Adiciona aresta ao grafo, conectando dois vértices v1 e v2. Cada aresta tem distância e taxa de tráfego
void Graph::addEdge(int vertex1Id, int vertex2Id, int distance, float trafficRate, int idEdge) 
{
    //Em nossa cidade Vargas não tem "laços de ruas"
    if (vertex1Id == vertex2Id) {
        std::cout << "Erro: Laços não são permitidos.\n";
        return;
    }
    Vertex* vertex1 = nullptr;
    Vertex* vertex2 = nullptr;

    for (auto v : vertices()) {
        if (v->id() == vertex1Id) vertex1 = v;
        if (v->id() == vertex2Id) vertex2 = v;
    }

    if (vertex1 && vertex2) {
        Edge* edge = new Edge(distance, vertex1, vertex2, trafficRate, idEdge);
        m_edges.push_back(edge);
    }
}

//Exibe a lista de adjacência
void Graph::printAdjacencyList() 
{
    if (adjacencyList().empty()) 
    {
        generateAdjacencyList(); 
    }
    
    cout << "Lista de Adjacência:\n";
    for (size_t i = 0; i < adjacencyList().size(); ++i) 
    {
        cout << "Vértice " << i << ":";
        for (const auto& [neighborId, edge] : adjacencyList()[i]) 
        {
            cout << " -> (" << neighborId << ", E("
                      << edge->vertex1()->id() << ", "
                      << edge->vertex2()->id() << ", Distância: "
                      << edge->distance() << "))";
        }
        cout << "\n";
    }
    cout << endl;
}

//Exibe a matriz de adjacência
void Graph::printAdjacencyMatrix() 
{
    if (adjacencyMatrix().empty()) 
    {
        generateAdjacencyMatrix();
    }
    
    cout << "Matriz de Adjacência:\n";
    for (size_t i = 0; i < adjacencyMatrix().size(); ++i) 
    {
        for (size_t j = 0; j < adjacencyMatrix()[i].size(); ++j) 
        {
            if (adjacencyMatrix()[i][j]) 
            {
                cout << "E(" << adjacencyMatrix()[i][j]->vertex1()->id() << ", "
                          << adjacencyMatrix()[i][j]->vertex2()->id() << ", Distância: "
                          << adjacencyMatrix()[i][j]->distance() << ") ";
            } 
            else 
            {
                cout << "0 ";
            }
        }
        cout << "\n";
    }
}


//PRIVADOS
//Gera a lista de adjacência, um vetor com listas de tuplas com id do vetor destino e o ponteiro pra aresta
void Graph::generateAdjacencyList() {
    m_adjacencyList.clear();
    //O tamanho é igual ao número de vértices,cada elemento é associado a um vértice
    m_adjacencyList.resize(vertices().size()); 

    for (auto e : edges()) {
        int vertex1Id = e->vertex1()->id();
        int vertex2Id = e->vertex2()->id();

        //adiciona as tuplas em cada lista
        m_adjacencyList[vertex1Id].emplace_back(vertex2Id, e);
        m_adjacencyList[vertex2Id].emplace_back(vertex1Id, e);
    }
}

//Constrói a matriz de adjacência, com ponteiros nulo ou de uma aresta 
void Graph::generateAdjacencyMatrix() {
    m_adjacencyMatrix.clear();
    m_adjacencyMatrix.resize(vertices().size(), vector<Edge*>(vertices().size(), nullptr));

    for (auto e : edges()) {
        int vertex1Id = e->vertex1()->id();
        int vertex2Id = e->vertex2()->id();

        m_adjacencyMatrix[vertex1Id][vertex2Id] = e;
        m_adjacencyMatrix[vertex2Id][vertex1Id] = e;
    }
}