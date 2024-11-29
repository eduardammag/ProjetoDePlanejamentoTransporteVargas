#include "steinerTree.h"
#include <climits>
#include <queue>
#include <algorithm>

// Implementação da classe UnionFind
UnionFind::UnionFind(int n) : parent(n), rank(n, 0) {
    for (int i = 0; i < n; ++i) parent[i] = i;
}

int UnionFind::find(int x) {
    if (parent[x] != x)
        parent[x] = find(parent[x]); // Caminho de compressão
    return parent[x];
}

void UnionFind::unite(int x, int y) {
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

// Função para calcular o menor caminho usando Dijkstra
vector<int> dijkstra(const vector<vector<tuple<int, Edge*>>>& adjacencyList, int start) {
    int n = adjacencyList.size();
    vector<int> dist(n, INT_MAX);
    vector<int> parent(n, -1);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int u = pq.top().second;
        int d = pq.top().first;
        pq.pop();

        if (d > dist[u]) continue;

        for (const auto& adj : adjacencyList[u]) {
            int v = get<0>(adj);
            Edge* edge = get<1>(adj);
            int weight = edge->distance();   // Supondo que 'distance' seja o peso da aresta

            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    return parent;
}

// Função para reconstruir o caminho mais curto
vector<int> reconstructPath(const vector<int>& parent, int destination) {
    vector<int> path;
    for (int v = destination; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

// Função principal para calcular a Árvore de Steiner
vector<Edge*> steinerTree(const vector<Vertex*>& vertices, const vector<vector<tuple<int, Edge*>>>& adjacencyList, const vector<Vertex*>& terminals) {
    vector<Edge*> steinerEdges;

    // Passo 1: Calcular o caminho mais curto entre todos os terminais usando Dijkstra
    vector<vector<int>> allShortestPaths(terminals.size());
    for (size_t i = 0; i < terminals.size(); ++i) {
        int terminalId = terminals[i]->id();  // Supondo que cada terminal tenha um ID
        allShortestPaths[i] = dijkstra(adjacencyList, terminalId);
    }

    // Passo 2: Construir uma nova rede com os terminais e suas conexões mais curtas
    vector<Edge*> mstEdges;

    // Inicializando a estrutura UnionFind para evitar ciclos
    UnionFind uf(vertices.size());

    // Simulação do algoritmo de Kruskal para conectar os terminais
    for (size_t i = 0; i < terminals.size(); ++i) {
        int terminalId = terminals[i]->id();
        for (size_t j = i + 1; j < terminals.size(); ++j) {
            int otherTerminalId = terminals[j]->id();
            vector<int> path = reconstructPath(allShortestPaths[i], otherTerminalId);

            // Conectar os terminais através das arestas do caminho
            for (size_t k = 1; k < path.size(); ++k) {
                int u = path[k-1];
                int v = path[k];

                // Suponha que a aresta entre u e v já exista na lista de adjacência
                for (const auto& adj : adjacencyList[u]) {
                    if (get<0>(adj) == v) {
                        Edge* edge = get<1>(adj);
                        
                        // Verifica se os vértices estão em componentes diferentes antes de adicionar a aresta
                        if (uf.find(u) != uf.find(v)) {
                            steinerEdges.push_back(edge);
                            uf.unite(u, v);
                        }
                    }
                }
            }
        }
    }

    return steinerEdges;
}
