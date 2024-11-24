#ifndef GRAPH_H
#define GRAPH_H

#include "vertexAndEdge.h"
#include <vector>
#include <list>
#include <tuple>

using namespace std;
class Graph {
public:
    void addVertex(bool isMetroStation, int id);
    void addEdge(int vertex1Id, int vertex2Id, int distance, float trafficRate, int idEdge);
    void printAdjacencyList();
    void printAdjacencyMatrix();
    
    // Getters
    const vector<Vertex*>& vertices();
    const vector<Edge*>& edges();
    vector<vector<Edge*>>& adjacencyMatrix();
    vector<list<tuple<int, Edge*>>>& adjacencyList();

private:
    // Estruturas internas
    vector<Vertex*> m_vertices;
    vector<Edge*> m_edges;
    vector<list<tuple<int, Edge*>>> m_adjacencyList;
    vector<vector<Edge*>> m_adjacencyMatrix;

    // Métodos para gerar as representações
    void generateAdjacencyList();
    void generateAdjacencyMatrix();
};

#endif
