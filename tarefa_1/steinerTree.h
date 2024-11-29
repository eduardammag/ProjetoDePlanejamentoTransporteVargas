#ifndef STEINER_TREE_H
#define STEINER_TREE_H

#include "vertexAndEdge.h"
#include "buildGraph.h"
#include <vector>
#include <map>
#include <queue>
#include <climits>
#include <algorithm>

using namespace std;

// Declaração da classe UnionFind
class UnionFind {
public:
    UnionFind(int n);
    int find(int x);
    void unite(int x, int y);

private:
    vector<int> parent;
    vector<int> rank;
};

// Declaração da função para calcular o menor caminho usando Dijkstra
vector<int> dijkstra(const vector<vector<tuple<int, Edge*>>>& adjacencyList, int start);

// Declaração da função para reconstruir o caminho mais curto
vector<int> reconstructPath(const vector<int>& parent, int destination);

// Declaração da função principal para calcular a Árvore de Steiner
std::vector<Edge*> steinerTree(const std::vector<Vertex*>& vertices, 
                               const std::vector<std::vector<std::tuple<int, Edge*>>>& adjacencyList, 
                               const std::vector<Vertex*>& terminals);
#endif // STEINER_TREE_H
