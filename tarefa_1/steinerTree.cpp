#include "steinerTree.h"
#include <climits>        // Para trabalhar com valores de infinito (INT_MAX)
#include <queue>          // Para usar fila de prioridade (priority_queue)
#include <algorithm>      // Para usar funções como sort()
#include <vector>         // Para usar containers como vector
#include <tuple>          // Para usar a tupla, útil para armazenar múltiplos valores
#include <iostream> 
#include <unordered_set>
using namespace std;

//Imprime a Árvore de Steiner agregada
void printAggregatedTree(const vector<Edge*>& steinerEdges) {
    unordered_map<string, double> aggregatedEdges;

    for (const auto& edge : steinerEdges) {
        int u = min(edge->vertex1()->id(), edge->vertex2()->id());
        int v = max(edge->vertex1()->id(), edge->vertex2()->id());
        double weight = edge->excavationCost(); // Obtém o custo de escavação da aresta

        string edgeKey = to_string(u) + "-" + to_string(v); // Chave única como string
        aggregatedEdges[edgeKey] += weight; // Soma o peso da aresta
    }

    cout << "\nÁrvore de Steiner Agregada:\n";
    for (const auto& [edgeKey, totalWeight] : aggregatedEdges) {
        cout << "Aresta (" << edgeKey << "), Peso Agregado: " << totalWeight << endl;
    }
}


// Função que gera todos os pares de terminais a partir de um vetor de terminais.
vector<pair<int, int>> generateTerminalPairs(const vector<Vertex*>& terminals) {
    // Vetor para armazenar os pares de terminais.
    vector<pair<int, int>> terminalPairs;

    // Laço para percorrer todos os terminais.
    for (size_t i = 0; i < terminals.size(); ++i) {
        // Laço aninhado para criar pares de terminais. O segundo índice começa em 'i + 1'
        // para garantir que cada par seja único (não repetido) e evitar pares reversos.
        for (size_t j = i + 1; j < terminals.size(); ++j) {
            // Adiciona o par de terminais (IDs dos terminais) ao vetor 'terminalPairs'.
            terminalPairs.push_back({terminals[i]->id(), terminals[j]->id()});
        }
    }

    // Retorna o vetor contendo todos os pares de terminais gerados.
    return terminalPairs;
}


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
            uf.unite(u, v);             // Une os conjuntos de u e v
        }
    }

    return mstEdges; // Retorna as arestas da Árvore Geradora Mínima
}

// Função principal para calcular a Árvore de Steiner com custos agregados
vector<vector<Edge*>> steinerTree(const vector<Vertex*>& vertices, 
                          const vector<vector<tuple<int, Edge*>>>& adjacencyList, 
                          const vector<Vertex*>& terminals, 
                          vector<vector<Edge*>>& detailedPaths) {
    vector<Edge*> allEdges;         // Vetor para armazenar todas as arestas calculadas
    detailedPaths.clear();          // Limpa as informações dos caminhos anteriores

    int new_id_edge = 0;            // ID para novas arestas

    // Calcula os caminhos mais curtos entre todos os pares de terminais
    vector<vector<int>> allShortestPaths(terminals.size());
    for (size_t i = 0; i < terminals.size(); ++i) {
        cout << "Calculando caminho mais curto a partir do terminal " << terminals[i]->id() << endl;
        allShortestPaths[i] = dijkstra(adjacencyList, terminals[i]->id()); // Chama a função de Dijkstra para cada terminal
    }

    // Constrói as arestas agregadas e registra os caminhos detalhados
    for (size_t i = 0; i < terminals.size(); ++i) {
        for (size_t j = i + 1; j < terminals.size(); ++j) {
            vector<int> path = reconstructPath(allShortestPaths[i], terminals[j]->id()); // Reconstruir o caminho entre terminais

            if (path.empty()) {
                cout << "Nenhum caminho encontrado entre " << terminals[i]->id() << " e " << terminals[j]->id() << endl;
                continue; // Se não houver caminho, pula para o próximo par de terminais
            }

            cout << "Reconstruindo caminho entre " << terminals[i]->id() << " e " << terminals[j]->id() << endl;
            cout << "Caminho: ";
            for (int vertex : path) {
                cout << vertex << " ";
            }
            cout << endl;

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
                Edge* newEdge = new Edge(totalExcavationCost, terminals[i], terminals[j], 0.0f, ++new_id_edge, 0);
                newEdge->setExcavationCost(totalExcavationCost); // Define o custo de escavação da nova aresta
                cout << "teste id 1 " << newEdge->idEdge() << endl;
                allEdges.push_back(newEdge); // Adiciona a nova aresta à lista

                detailedPaths.push_back(pathEdges); // Armazena o caminho detalhado
            }
        }
    }

    // Aplica o algoritmo de Kruskal para calcular a MST das arestas agregadas
    vector<Edge*> bestAgregatedEdges = kruskal(vertices.size(), allEdges);
    vector<vector<Edge*>> MSTDetailedPaths;
    for (const auto& bestEdge : bestAgregatedEdges)
    {   
        int bestId = bestEdge->idEdge();
        cout << "TEste id" <<  bestId << endl;
        MSTDetailedPaths.push_back(detailedPaths[bestId]);
    }

    return MSTDetailedPaths; // Retorna as arestas da Árvore de Steiner
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

// Função para reconstruir o caminho entre dois vértices dados os pais
vector<int> reconstructPath(int source, int target, const vector<int>& parent) {
    vector<int> path;
    for (int v = target; v != source; v = parent[v]) {
        if (v == -1) {  // Caso não haja caminho válido
            path.clear();  // Limpa o caminho
            break;
        }
        path.push_back(v);  // Adiciona vértices ao caminho
    }
    path.push_back(source);  // Adiciona a origem ao final do caminho
    reverse(path.begin(), path.end());  // Reverte para garantir a ordem correta
    return path;  // Retorna o caminho entre os dois vértices
}

/*
// Função para imprimir os caminhos detalhados entre os terminais, 
void printDetailedPaths(const std::vector<std::vector<Edge*>>& detailedPaths, 
                        const std::vector<Vertex*>& terminals) {
    size_t n = terminals.size(); // Obtém o número total de terminais.

    // Imprime o número esperado de caminhos entre os pares de terminais.
    cout << "Número esperado de caminhos: " 
         << (n * (n - 1)) / 2 << endl;

    // Imprime o número de caminhos presentes no vetor 'detailedPaths'.
    cout << "Número de caminhos em detailedPaths: " << detailedPaths.size() << endl;

    // Laço para percorrer todos os pares de terminais.
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            int terminal1 = terminals[i]->id(); // ID do primeiro terminal do par.
            int terminal2 = terminals[j]->id(); // ID do segundo terminal do par.

            // Imprime informações sobre o caminho entre o par de terminais.
            cout << "Caminho otimizado entre os dois vértices " << terminal1 << " e " << terminal2 << ":\n";

            // Calcula o índice no vetor 'detailedPaths' que corresponde ao par de terminais atual.
            size_t index = (n * (n - 1)) / 2 - ((n - i) * (n - i - 1)) / 2 + (j - i - 1);

            // Verifica se o índice calculado está dentro dos limites do vetor 'detailedPaths'.
            if (index >= detailedPaths.size()) {
                cout << "Erro: Índice calculado (" << index << ") fora dos limites do vetor detailedPaths.\n";
                cout << "Par: (" << terminal1 << ", " << terminal2 << "), "
                     << "i: " << i << ", j: " << j << ", nTerminals: " << n << endl;
                cout << "---------------------------------\n";
                continue; // Pula para o próximo par de terminais se o índice estiver fora dos limites.
            }

            // Obtém o caminho detalhado entre os dois terminais.
            const auto& path = detailedPaths[index];

            // Verifica se o caminho está vazio.
            if (path.empty()) {
                cout << "Nenhum caminho encontrado entre " << terminal1 << " e " << terminal2 << "\n";
            } else {
                cout << "Caminho: ";
                std::unordered_set<int> seenVertices; // Conjunto para armazenar os vértices já visitados.
                bool first = true; // Variável para controlar se é o primeiro vértice a ser impresso.

                // Laço para percorrer as arestas do caminho.
                for (const auto& edge : path) {
                    int vertex = edge->vertex2()->id(); // Obtém o ID do vértice destino da aresta.

                    // Verifica se o vértice já foi visto, para não imprimir vértices repetidos.
                    if (seenVertices.find(vertex) == seenVertices.end()) {
                        // Se não for o primeiro vértice, imprime a seta (->).
                        if (!first) {
                            cout << " -> ";
                        }
                        cout << vertex; // Imprime o ID do vértice.
                        seenVertices.insert(vertex); // Marca o vértice como visitado.
                        first = false; // Depois de imprimir o primeiro vértice, define 'first' como false.
                    }
                }
                cout << endl; // Finaliza a linha do caminho.
            }
            cout << "---------------------------------\n"; // Separa os resultados dos pares de terminais.
        }
    }
}
*/

