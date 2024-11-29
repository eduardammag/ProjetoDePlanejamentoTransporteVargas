#include "findStation.h"
#include <queue>
#include <climits>
#include <tuple>
#include <algorithm>


using namespace std;

// Implementação da função cptDijkstraFast
void cptDijkstraFast(Vertex* v0, vector<int>& parent, vector<int>& distance, const vector<vector<tuple<int, Edge*>>> adjacencyList) {
    int numVertices = adjacencyList.size();
    vector<bool> checked(numVertices, false);

    parent.assign(numVertices, -1);
    distance.assign(numVertices, INT_MAX);

    // Priority queue para gerenciar os vértices a serem processados
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

    parent[v0->id()] = v0->id();
    distance[v0->id()] = 0;
    pq.emplace(0, v0->id()); // Inicializa o heap com o vértice inicial

    while (!pq.empty()) {
        int v1 = pq.top().second;
        pq.pop();

        if (checked[v1]) continue;
        checked[v1] = true;

        // Explora as arestas saindo de v1
        for (const auto& tupleData : adjacencyList[v1]) {
            int v2 = get<0>(tupleData); // ID do vértice vizinho
            Edge* edge = get<1>(tupleData); // Ponteiro para a aresta
            int cost = edge->distance();

            if (distance[v1] + cost < distance[v2]) {
                parent[v2] = v1;
                distance[v2] = distance[v1] + cost;
                pq.emplace(distance[v2], v2); // Atualiza o heap com a nova distância
            }
        }
    }
}

// Implementação da função findOptimalVertexFast
Vertex* findOptimalVertexFast(const vector<vector<Edge*>>& regionVertices, const vector<vector<tuple<int, Edge*>>>& adjacencyList) {
    int numVertices = adjacencyList.size();
    int minMaxDist = INT_MAX;
    Vertex* optimalVertex = nullptr;

    for (const auto& vertexList : regionVertices) {
        for (Edge* edge : vertexList) {
            Vertex* vertex = edge->vertex1(); // Ou vertex2(), dependendo da lógica
            vector<int> parent(numVertices);
            vector<int> distance(numVertices);

            cptDijkstraFast(vertex, parent, distance, adjacencyList);

            int maxDist = INT_MIN;
            for (const auto& innerVertexList : regionVertices) {
                for (Edge* innerEdge : innerVertexList) {
                    Vertex* target = innerEdge->vertex1(); // Ou vertex2()
                    maxDist = max(maxDist, distance[target->id()]);
                }
            }

            if (maxDist < minMaxDist) {
                minMaxDist = maxDist;
                optimalVertex = vertex;
            }
        }
    }

    // Atualiza o atributo do vértice ótimo, se encontrado
    if (optimalVertex) {
        optimalVertex->setMetroStation(true); // Marca o vértice ótimo como estação de metrô
    }

    return optimalVertex;


}
