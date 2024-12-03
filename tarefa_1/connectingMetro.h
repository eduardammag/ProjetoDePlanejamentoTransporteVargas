#ifndef STEINER_TREE_H
#define STEINER_TREE_H

#include "vertexAndEdge.h"
#include "buildGraph.h"
#include <vector>
#include <tuple>
#include <queue>
#include <climits>
#include <algorithm>

using namespace std;

// Declaração da classe UnionFind
class UnionFind {
public:
    vector<int> parent, rank;

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
};

// Declaração da função para calcular o menor caminho usando Dijkstra
vector<int> dijkstra(const vector<vector<tuple<int, Edge*>>>& adjacencyList, int start);

// Declaração da função para reconstruir o caminho mais curto
vector<int> reconstructPath(const vector<int>& parent, int destination);

// Declaração da função para reconstruir o caminho entre dois vértices (com origem e destino especificados)
vector<int> reconstructPath(int source, int target, const vector<int>& parent);

// Declaração da função para gerar os pares de terminais
void generateTerminalPairs(const std::vector<Vertex*>& terminals, std::vector<std::pair<int, int>>& terminalPairs);

// Declaração da função para calcular a Árvore de Steiner
std::vector<Edge*> conect_metro(const std::vector<Vertex*>& vertices, 
                                const std::vector<std::vector<std::tuple<int, Edge*>>>& adjacencyList, 
                                const std::vector<Vertex*>& terminals, 
                                std::vector<std::vector<Edge*>>& detailedPaths);
                                
// Declaração da função para construir a Árvore Geradora Mínima (MST) usando Kruskal
vector<Edge*> kruskal(int numVertices, const vector<Edge*>& edges);

// Calculando o custo total, contabilizando uma vez arestas em comum
int totalCostSubway(vector<Edge*> bestRoutes, vector<vector<Edge*>> detailedPaths);
// Exibindo a melhor rota que conecta as estações
void printBestRoutes(vector<Edge*> bestRoutes, vector<vector<Edge*>> detailedPaths);
#endif // STEINER_TREE_H
