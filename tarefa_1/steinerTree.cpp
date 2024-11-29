#include "steinerTree.h"
#include <climits>
#include <queue>
#include <algorithm>
#include <vector>
#include <tuple>

using namespace std;


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

// Função de Kruskal para construir a MST
vector<Edge*> kruskal(int numVertices, const vector<Edge*>& edges) {
    vector<Edge*> mstEdges;
    UnionFind uf(numVertices);

    // Ordena as arestas pelo peso
    vector<Edge*> sortedEdges = edges;
    sort(sortedEdges.begin(), sortedEdges.end(), [](Edge* a, Edge* b) {
        return a->excavationCost() < b->excavationCost(); // Supondo que 'distance()' retorne o peso
    });

    // Adiciona arestas ao MST enquanto evita ciclos
    for (Edge* edge : sortedEdges) {
        // Alterar para usar o método adequado de acesso aos vértices
    int u = edge->vertex1()->id();  // Método correto para acessar o vértice de origem
    int v = edge->vertex2()->id();  // Método correto para acessar o vértice de destino
        if (uf.find(u) != uf.find(v)) {
            mstEdges.push_back(edge);
            uf.unite(u, v);
        }
    }

    return mstEdges;
}

// Função principal para calcular a Árvore de Steiner com custos agregados
vector<Edge*> steinerTree(const vector<Vertex*>& vertices, 
                          const vector<vector<tuple<int, Edge*>>>& adjacencyList, 
                          const vector<Vertex*>& terminals) {
    vector<Edge*> allEdges;

    int new_id_edge = 0;
    
    // Calcula os caminhos mais curtos entre terminais
    vector<vector<int>> allShortestPaths(terminals.size());
    for (size_t i = 0; i < terminals.size(); ++i) {
        allShortestPaths[i] = dijkstra(adjacencyList, terminals[i]->id());
    }

    // Constrói arestas do subgrafo
    for (size_t i = 0; i < terminals.size(); ++i) {
        for (size_t j = i + 1; j < terminals.size(); ++j) {
            vector<int> path = reconstructPath(allShortestPaths[i], terminals[j]->id());

            // Soma os custos das arestas entre os dois vértices
            float totalExcavationCost = 0.0f;  // Use float, pois o custo de escavação pode ser decimal
            for (size_t k = 1; k < path.size(); ++k) {
                int u = path[k - 1];
                int v = path[k];

                // Verifica se existe uma aresta entre os vértices u e v
                for (const auto& adj : adjacencyList[u]) {
                    if (get<0>(adj) == v) {
                        totalExcavationCost += get<1>(adj)->excavationCost(); // Soma o custo de escavação
                    }
                }
            }
            
            // Agora, em vez de adicionar várias arestas, adiciona uma única aresta com o custo total
            if (totalExcavationCost > 0) {
                // Cria a aresta agregada com o custo total de escavação
                Edge* newEdge = new Edge(new_id_edge++, terminals[i], terminals[j], 0.0f, 0, 0); 
                // Aqui você pode configurar o ID da aresta ou outros valores se necessário
                newEdge->setExcavationCost(totalExcavationCost);    
                allEdges.push_back(newEdge);
            }
        }
    }

    // Aplica Kruskal para obter a MST do subgrafo com os custos agregados
    return kruskal(vertices.size(), allEdges);
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

// Função para reconstruir o caminho entre dois vértices
vector<int> reconstructPath(int source, int target, const vector<int>& parent) {
    vector<int> path;
    for (int v = target; v != source; v = parent[v]) {
        if (v == -1) { // Caso não haja caminho
            path.clear();
            break;
        }
        path.push_back(v);
    }
    path.push_back(source); // Adiciona a origem ao final
    reverse(path.begin(), path.end()); // Reverte o caminho para que fique na ordem correta
    return path;
}
