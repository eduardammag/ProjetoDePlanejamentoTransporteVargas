#include "fastRoute.h"
#include "vertexAndEdge.h"
#include <iostream> 
#include <stack>
#include <queue>
#include <iostream>

using namespace std; 

// Implementação da função cptDijkstraFast
pair<vector<Edge*>, int> dijkstraFoot(Vertex* start,  Vertex* destination, const vector<vector<tuple<int, Edge*>>> adjacencyList) 
{
    // inicializa as estruturas auxilares
    int numVertices = adjacencyList.size();
    vector<bool> checked(numVertices, false);
    vector<pair<int, Edge*>> parent(numVertices, {-1, nullptr});
    vector<int> distance(numVertices, 10000);

    // Priority queue para gerenciar os vértices a serem processados
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

    // Inicializa o heap com o vértice inicial
    parent[start->id()] = {start->id(), nullptr};
    distance[start->id()] = 0;
    pq.emplace(0, start->id()); 

    // itera pelo heap
    while (!pq.empty()) {
        int v1 = pq.top().second;
        pq.pop();

        if (checked[v1]) continue;
        checked[v1] = true;

        // Explora as arestas saindo de v1
        for (const auto& tupleData : adjacencyList[v1]) {
            // ID do vértice vizinho
            int v2 = get<0>(tupleData); 
            // Ponteiro para a aresta
            Edge* edge = get<1>(tupleData); 
            int cost = edge->distance();

            if (distance[v1] + cost < distance[v2]) {
                parent[v2] = {v1, edge};
                distance[v2] = distance[v1] + cost;
                // Atualiza o heap com a nova distância
                pq.emplace(distance[v2], v2); 
            }
        }
    }
    
    // Tempo percorrendo a distância total com a velocidade de 83 metros por minutos
    int tempo = distance[destination->id()] / 83.33;
    
    // Reconstrução do caminho
    stack<Edge*> edgeStack;
    for (int v = destination->id(); v != start->id(); v = parent[v].first) {
        if (v == -1) {
            cout<< "Caminho não encontrado." << endl;
            return {{}, -1};
        }
        edgeStack.push(parent[v].second); // Adiciona a aresta usada para chegar ao vértice atual
    }
    
    // Converte a pilha em uma lista
    vector<Edge*> edgeList;
    while (!edgeStack.empty()) {
        edgeList.push_back(edgeStack.top());
        edgeStack.pop();
    }
    
    // Retorna a lista de arestas e o tempo em minutos
    return {edgeList, tempo};
}

// Função para encontrar a aresta correspondente a um número de imóvel em uma rua.
Edge* findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges) {
    int current_distance = 0; // Distância acumulada ao longo da rua.

    // Ordena as arestas para garantir a sequência crescente dos IDs dos vértices.
    vector<Edge*> sorted_edges = edges;
    sort(sorted_edges.begin(), sorted_edges.end(), [](Edge* a, Edge* b) {
        return min(a->vertex1()->id(), a->vertex2()->id()) < min(b->vertex1()->id(), b->vertex2()->id());
    });

    for (Edge* edge : sorted_edges) {
        // cout << "Verificando aresta: Rua ID " << edge->id_street() << ", CEP " << edge->id_zipCode() << endl;
    
        if (edge->id_street() == street && edge->id_zipCode() == id_zipCode) {
            int id_start = min(edge->vertex1()->id(), edge->vertex2()->id());
            int id_end = max(edge->vertex1()->id(), edge->vertex2()->id());
    
            int start_number = current_distance + 1;
            int end_number = current_distance + edge->distance();
    
            // cout << "Aresta encontrada: Vértices (" << id_start << ", " << id_end << ")" << endl;
            // cout << "Intervalo de números na aresta: [" << start_number << ", " << end_number << "]" << endl;
    
            if (number_build >= start_number && number_build <= end_number) {
                // cout << "Número do imóvel encontrado nesta aresta!" << endl;
                return edge;
            }
    
            current_distance = end_number; // Atualiza a distância acumulada.
        } else {
            // cout << "Aresta ignorada: não corresponde à rua ou ao CEP." << endl;
        }
    }


    return nullptr; // Retorna nullptr caso nenhuma aresta seja encontrada.
}


// findSameRigion( ){
//     vai o maximo de taxi dentro do orçamento e o resto a  pe de graça
//     ou só a pé
// }

// vector<tuple<str, Edge*>> findFastRoute( int street, int id_zipCode, int number_build, int max_cost, int hour){
//     taxi completo = preço tempo usando dijkstra direcionado transito*velocidade
//     a pé = 0 reais tempo grafo não direcionado velocidade constante
//     se custo< max_custo : break;

//     testa para a estação da mesma região, qual o mais rápido: taxi  ou a pé
//     Se no final ultrapassar o orçamento, tentamos a péaté a primeira
//         testa até a primeira estação no caminho para maximo de metro, a estação da região do destino:
//             Testa taxi e a pé, qual o mais rápido

//         Testa mais uma:
//             Testa taxi e a pé, qual o mais rapido para o destino final dentro do orçamento
//             Se o taxi não está no orçamento e a pé é mais devagar que taxi, e tiver outro metro, testa o metro para a próxima 

//         no maximo de metro, a estação da região do destino
//         se está na mesma região, não vai usar metrô


// }