#include "fastRoute.h"
#include "vertexAndEdge.h"
#include <iostream> 
#include <stack>
#include <queue>
#include <iostream>
#include <cfloat>
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
pair<Edge*, Vertex*> findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges) {
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
    
            // Verifica se o número do imóvel está no intervalo desta aresta
            if (number_build >= start_number && number_build <= end_number) {
                // Calcula a posição relativa do número do imóvel na aresta
                int relative_position = number_build - start_number;
                int middle_point = edge->distance() / 2;
                
                Vertex* closest_vertex;
                if (relative_position <= middle_point)
                {
                    closest_vertex = edge->vertex1();
                }
                else
                {
                    closest_vertex = edge->vertex2();
                }
                
                return {edge, closest_vertex};
            }
    
            current_distance = end_number; // Atualiza a distância acumulada.
        }
    }

    return {nullptr, nullptr}; // Retorna nullptr caso nenhuma aresta seja encontrada.
}

// Função que calcula o menor caminho e custo usando Dijkstra para um táxi
tuple<float, float, vector<Edge*>> dijkstraTaxi(
    const vector<vector<tuple<int, Edge*>>>& directedAdj, // Lista de adjacência representando o grafo dirigido
    int start, // Vértice inicial
    int destination // Vértice de destino
) {
    // Parâmetros relacionados ao táxi
    float taxiRatePerMeter = 0.01f;  // Tarifa do táxi por metro percorrido
    float maxSpeedKmH = 30.0f;      // Velocidade máxima permitida (em km/h)

    // Número de vértices no grafo
    int n = directedAdj.size();

    // Vetores de controle para armazenar informações durante o cálculo
    vector<float> minTime(n, FLT_MAX);  // Menor tempo de viagem para cada vértice
    vector<float> totalCost(n, FLT_MAX);  // Menor custo total de viagem para cada vértice
    vector<int> parent(n, -1);  // Para rastrear o caminho percorrido
    vector<Edge*> edgeUsed(n, nullptr);  // Para armazenar as arestas utilizadas no caminho

    // Fila de prioridade para o Dijkstra, ordenada pelo menor tempo
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> pq;

    // Inicialização do vértice de início
    minTime[start] = 0;
    totalCost[start] = 0;
    pq.push({0, start});  // Insere o vértice inicial na fila de prioridade

    // Algoritmo principal do Dijkstra
    while (!pq.empty()) {
        // Extrai o vértice com o menor tempo acumulado
        float currTime = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // Para cada aresta saindo do vértice atual
        for (const auto& [v, edge] : directedAdj[u]) {
            // Verifica se a taxa de tráfego é válida
            if (edge->trafficRate() <= 0) continue;

            // Calcula a velocidade real considerando a taxa de tráfego
            float realSpeedKmH = maxSpeedKmH * edge->trafficRate();  // Velocidade em km/h
            float realSpeedMPS = realSpeedKmH * (1000.0f / 3600.0f);  // Velocidade em metros por segundo

            // Calcula o tempo necessário para percorrer a aresta (em minutos)
            float edgeTime = edge->distance() / (realSpeedMPS * 60.0f);

            // Relaxamento: verifica se encontrou um caminho mais rápido
            if (currTime + edgeTime < minTime[v]) {
                minTime[v] = currTime + edgeTime;  // Atualiza o menor tempo
                totalCost[v] = totalCost[u] + edge->distance() * taxiRatePerMeter;  // Atualiza o custo total
                parent[v] = u;  // Atualiza o vértice pai
                edgeUsed[v] = edge;  // Armazena a aresta utilizada
                pq.push({minTime[v], v});  // Insere o vértice na fila de prioridade
            }
        }
    }

    // Reconstrução do caminho e das arestas percorridas
    vector<Edge*> edges;  // Vetor para armazenar as arestas do caminho
    if (minTime[destination] < FLT_MAX) {  // Verifica se há caminho até o destino
        for (int v = destination; v != start; v = parent[v]) 
        {
            if (v != -1) edges.push_back(edgeUsed[v]);  // Armazena a aresta utilizada
        }
        reverse(edges.begin(), edges.end());  // Inverte para a ordem correta
    }

    // Impressão dos resultados
    if (minTime[destination] < FLT_MAX) {
        cout << "Custo total da viagem de táxi: R$ " << totalCost[destination] << endl;
        cout << "Tempo total de viagem de táxi: " << minTime[destination] << " minutos" << endl;
        cout << "Arestas percorridas (IDs): ";
        for (Edge* edgePath : edges) {
            cout << edgePath->idEdge() << " ";
        }
        cout << endl;
    } else {
        cout << "Não foi possível encontrar um caminho válido." << endl;
    }

    // Retorna o custo total, tempo total e as arestas percorridas
    return {totalCost[destination], minTime[destination], edges};
}

//Função para verificar se a aresta destino está na mesma região que a aresta inicial
bool isTheSameRegion(const Edge* edge1, const Edge* edge2) {
    return edge1->id_zipCode() == edge2->id_zipCode();
}

//Função para verificar o melhor caminho entre dois vértices (de táxi ou a pé), considerando o orçamento
vector<Edge*> findBestPath(pair<Edge*, Vertex*> start, pair<Edge*, Vertex*> destination, 
                           const vector<vector<tuple<int, Edge*>>>& adjacencyList, 
                           const vector<vector<tuple<int, Edge*>>>& directedAdj, float budget) 
{
    Edge* startEdge = start.first;
    Vertex* startVertex = start.second;
    Vertex* destinationVertex = destination.second;

    // Caminho de carro usando Dijkstra
    auto [carCost, carTime, carPath] = dijkstraTaxi(directedAdj, startVertex->id(), destinationVertex->id());
    
    if (carCost <= budget) {
        cout << "Recomendado ir de carro, custo: R$ " << carCost << ", tempo: " << carTime << " minutos." << endl;
        return carPath;
    }

    // Caminho a pé usando Dijkstra
    auto [footPath, footTime] = dijkstraFoot(startVertex, destinationVertex, adjacencyList);
    if (!footPath.empty() && footTime < carTime) {
        cout << "Recomendado ir a pé, tempo: " << footTime << " minutos." << endl;
        return footPath;
    }

    cout << "Nenhum caminho a pé mais rápido que o de carro foi encontrado, mas o custo de carro excede o orçamento." << endl;
    return {};
}