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

// Classe UnionFind para o algoritmo de Kruskal
class UnionFind {
public:
    UnionFind(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }

    int find(int x) {
        if (parent[x] != x)
            parent[x] = find(parent[x]); // Caminho de compressão
        return parent[x];
    }

    void unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        if (rootX != rootY) {
            if (rank[rootX] < rank[rootY]) parent[rootX] = rootY;
            else if (rank[rootX] > rank[rootY]) parent[rootY] = rootX;
            else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
        }
    }

private:
    vector<int> parent;
    vector<int> rank;
};

// Função para calcular o menor caminho usando Dijkstra
vector<int> dijkstra(const vector<vector<tuple<int, Edge*>>>& adjacencyList, int start);

// Função para reconstruir o caminho mais curto
vector<int> reconstructPath(const vector<int>& parent, int destination);

// Função principal para calcular a Árvore de Steiner
Graph steinerTree(const vector<Vertex*>& vertices, const vector<vector<tuple<int, Edge*>>>& adjacencyList, const vector<Vertex*>& terminals);

// Algoritmo de Kruskal para encontrar a MST
Graph kruskal(const Graph& g);

#endif // STEINER_TREE_H
