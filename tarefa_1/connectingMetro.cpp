#include "connectingMetro.h"
#include <climits>        // Para trabalhar com valores de infinito (INT_MAX)
#include <queue>          // Para usar fila de prioridade (priority_queue)
#include <algorithm>      // Para usar funções como sort()
#include <vector>         // Para usar containers como vector
#include <tuple>          // Para usar a tupla, útil para armazenar múltiplos valores
#include <iostream> 
#include <unordered_set>
using namespace std;


// Função para calcular o menor caminho entre os vértices usando o algoritmo de Dijkstra
vector<int> dijkstra(const vector<vector<tuple<int, Edge*>>>& adjacencyList, int start) {
    int n = adjacencyList.size(); // Número de vértices no grafo
    vector<int> dist(n, INT_MAX);  // Distâncias para cada vértice, inicialmente infinitas
    vector<int> parent(n, -1);    // Vetor para armazenar o pai de cada vértice no caminho mais curto
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // Fila de prioridade para escolher o vértice com a menor distância

    dist[start] = 0;              // A distância até o vértice inicial é 0
    pq.push({0, start});          // Coloca o vértice inicial na fila de prioridade

    while (!pq.empty()) {
        int u = pq.top().second;  // Vértice com a menor distância
        int d = pq.top().first;   // Distância associada ao vértice
        pq.pop();                  // Remove o vértice da fila de prioridade

        if (d > dist[u]) continue; // Se a distância é maior que a já registrada, ignora (pois é um valor obsoleto)

        // Percorre todos os vizinhos de u
        for (const auto& adj : adjacencyList[u]) {
            int v = get<0>(adj);   // Vértice adjacente
            Edge* edge = get<1>(adj); // Aresta que conecta u a v
            int weight = edge->distance(); // Peso da aresta (supondo que 'distance' retorna o custo da aresta)

            // Se o caminho passando por u para v é mais curto
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;  // Atualiza a distância para v
                parent[v] = u;               // Atualiza o pai de v
                pq.push({dist[v], v});       // Coloca v na fila de prioridade com a nova distância
            }
        }
    }

    return parent; // Retorna o vetor de pais que permite reconstruir o caminho mais curto
}

// Função de Kruskal para construir a Árvore Geradora Mínima (MST)
vector<Edge*> kruskal(int numVertices, const vector<Edge*>& edges) {
    vector<Edge*> mstEdges;         // Vetor para armazenar as arestas da MST
    UnionFind uf(numVertices);      // Estrutura de dados Union-Find para evitar ciclos

    // Ordena as arestas pelo peso (excavationCost)
    vector<Edge*> sortedEdges = edges;
    sort(sortedEdges.begin(), sortedEdges.end(), [](Edge* a, Edge* b) {
        return a->excavationCost() < b->excavationCost(); // Ordenação por custo de escavação
    });

    // Adiciona as arestas à MST, evitando ciclos
    for (Edge* edge : sortedEdges) {
        // Acessa os vértices de origem e destino da aresta
        int u = edge->vertex1()->id();  // Vértice de origem
        int v = edge->vertex2()->id();  // Vértice de destino
        
        // Se os vértices não estão conectados, adiciona a aresta à MST
        if (uf.find(u) != uf.find(v)) {
            mstEdges.push_back(edge);   // Adiciona a aresta à MST
            uf.unite(u, v);             // Une os conjuntoshttps://www.onlinegdb.com/#_editor_760118211 de u e v
        }
    }

    return mstEdges; // Retorna as arestas da Árvore Geradora Mínima
}

// Função principal para calcular a Árvore de Steiner com custos agregados
vector<Edge*> conect_metro(const vector<Vertex*>& vertices, 
                          const vector<vector<tuple<int, Edge*>>>& adjacencyList, 
                          const vector<Vertex*>& terminals, 
                          vector<vector<Edge*>>& detailedPaths) {
    vector<Edge*> allEdges;         // Vetor para armazenar todas as arestas calculadas
    detailedPaths.clear();          // Limpa as informações dos caminhos anteriores

    int new_id_edge = 0;            // ID para novas arestas

    // Calcula os caminhos mais curtos entre todos os pares de terminais
    vector<vector<int>> allShortestPaths(terminals.size());
    for (size_t i = 0; i < terminals.size(); ++i) {
        allShortestPaths[i] = dijkstra(adjacencyList, terminals[i]->id()); // Chama a função de Dijkstra para cada terminal
    }

    // Constrói as arestas agregadas e registra os caminhos detalhados
    for (size_t i = 0; i < terminals.size(); ++i) {
        for (size_t j = i + 1; j < terminals.size(); ++j) {
            vector<int> path = reconstructPath(allShortestPaths[i], terminals[j]->id()); // Reconstruir o caminho entre terminais

            vector<Edge*> pathEdges; // Vetor para armazenar as arestas do caminho
            float totalExcavationCost = 0.0f; // Custo total de escavação do caminho
            for (size_t k = 1; k < path.size(); ++k) {
                int u = path[k - 1];       // Vértice anterior
                int v = path[k];           // Vértice atual

                bool found = false;
                // Percorre a lista de adjacência para encontrar a aresta entre u e v
                for (const auto& adj : adjacencyList[u]) {
                    if (get<0>(adj) == v) {
                        totalExcavationCost += get<1>(adj)->excavationCost();  // Soma o custo de escavação
                        pathEdges.push_back(get<1>(adj));  // Adiciona a aresta ao caminho
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    cout << "Aresta não encontrada entre " << u << " e " << v << endl;
                }
            }

            // Se o custo de escavação for positivo, cria uma nova aresta agregada
            if (totalExcavationCost > 0) {
                Edge* newEdge = new Edge(totalExcavationCost, terminals[i], terminals[j], 0.0f, new_id_edge++, 0);
                newEdge->setExcavationCost(totalExcavationCost); // Define o custo de escavação da nova aresta
                allEdges.push_back(newEdge); // Adiciona a nova aresta à lista

                detailedPaths.push_back(pathEdges); // Armazena o caminho detalhado
            }
        }
    }

    // Aplica o algoritmo de Kruskal para calcular a MST das arestas agregadas
    vector<Edge*> connectedEdges = kruskal(vertices.size(), allEdges);

    return connectedEdges; // Retorna as arestas da Árvore de Steiner
}


// Função para reconstruir o caminho mais curto a partir dos pais
vector<int> reconstructPath(const vector<int>& parent, int destination) {
    vector<int> path;
    for (int v = destination; v != -1; v = parent[v]) {
        path.push_back(v);  // Adiciona cada vértice ao caminho
    }
    reverse(path.begin(), path.end());  // Reverte o caminho para a ordem correta
    return path;  // Retorna o caminho reconstruído
}

// Calcular o custo total das melhores rotas encontradas
// contabilizando uma vez as arestas em comum
int totalCostSubway(vector<Edge*> bestRoutes, vector<vector<Edge*>> detailedPaths)
{
    unordered_set<int> processedEdges;
    int costTotal = 0;
    for (size_t i = 0; i < bestRoutes.size(); ++i) 
    {
        for (const auto& edge : detailedPaths[i]) 
        {
            if (processedEdges.count(edge->idEdge())) continue;
            else
            {
                processedEdges.insert(edge->idEdge());
                costTotal += edge->excavationCost();
            }
        }
    }
    
    return costTotal;
}

void printBestRoutes(vector<Edge*> bestRoutes, vector<vector<Edge*>> detailedPaths)
{
    
    // Imprimir caminhos detalhados correspondentes às arestas agregadas
    cout << "\nCaminhos detalhados:\n";
    for (size_t i = 0; i < bestRoutes.size(); ++i) 
    {
        cout << "Aresta entre vértice ";
        for (const auto& edge : detailedPaths[i]) 
        {
            cout  << edge->vertex1()->id() << " " << edge->vertex2()->id() << ", ";
        }
        cout << endl;
    }
}