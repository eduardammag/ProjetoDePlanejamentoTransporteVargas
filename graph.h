#ifndef GRAPH_H
#define GRAPH_H

#include "vertexAndEdge.h"
#include <vector>
#include <list>
#include <tuple>

class Graph {
public:
    std::vector<Vertex*> vertices;
    std::vector<Edge*> edges;
    std::vector<std::list<std::tuple<int, Edge*>>> adjacencyList;
    std::vector<std::vector<Edge*>> adjacencyMatrix;

    void addVertex(bool isMetroStation, int id);
    void addEdge(int vertex1Id, int vertex2Id, int distance, float trafficRate);
    void generateAdjacencyList();
    void generateAdjacencyMatrix();
    void printAdjacencyList();
    void printAdjacencyMatrix();
};

#endif
