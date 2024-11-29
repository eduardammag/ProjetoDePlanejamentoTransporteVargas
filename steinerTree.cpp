#include "steinerTree.h"

// Função para calcular o menor caminho usando Dijkstra
vector<int> dijkstra(const vector<vector<tuple<int, Edge*>>>& adjacencyList, int start) {
    int n = adjacencyList.size();
    vector<int> dist(n, INT_MAX);  // Distâncias iniciais como infinito
    dist[start] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;  // Min-heap
    pq.push({0, start});  // Inicia a fila com o vértice de origem

    while (!pq.empty()) {
        int u = pq.top().second;
        int currentDist = pq.top().first;
        pq.pop();

        // Se a distância atual for maior do que a já encontrada, pula
        if (currentDist > dist[u]) continue;

        // Explorar os vizinhos
        for (const auto& adj : adjacencyList[u]) {
            int v = get<0>(adj);
            int weight = get<1>(adj)->weight();  // Supondo que você tenha um método weight() para obter o peso

            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    return dist;
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
Graph steinerTree(const vector<Vertex*>& vertices, const vector<vector<tuple<int, Edge*>>>& adjacencyList, const vector<Vertex*>& terminals) {
    // Passo 1: Calcular menor caminho de cada terminal para todos os vértices
    map<int, vector<int>> MenorCaminho;
    for (const auto& terminal : terminals) {
        MenorCaminho[terminal->id()] = dijkstra(adjacencyList, terminal->id());
    }

    // Passo 2: Criar um subgrafo contendo caminhos curtos entre pares de terminais
    Graph SubgrafoSteiner;
    for (size_t i = 0; i < terminals.size(); ++i) {
        for (size_t j = i + 1; j < terminals.size(); ++j) {
            int u = terminals[i]->id();
            int v = terminals[j]->id();
            vector<int> caminho = reconstructPath(MenorCaminho[u], v);
            for (size_t k = 1; k < caminho.size(); ++k) {
                int u = caminho[k - 1];
                int v = caminho[k];
                // Adicionar a aresta ao subgrafo
                SubgrafoSteiner.addEdge(u, v);
            }
        }
    }

    // Passo 3: Construir a MST no subgrafo
    Graph SteinerTree = kruskal(SubgrafoSteiner);

    return SteinerTree;
}

// Algoritmo de Kruskal para encontrar a MST
Graph kruskal(const Graph& g) {
    vector<Edge> edges = g.getEdges();
    sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return a.weight() < b.weight(); // Ordenar as arestas pelo peso
    });

    UnionFind uf(g.numVertices());
    Graph mst;

    for (const auto& edge : edges) {
        int u = edge.u();
        int v = edge.v();
        if (uf.find(u) != uf.find(v)) {
            uf.unite(u, v);
            mst.addEdge(u, v, edge.weight());
        }
    }

    return mst;
}
