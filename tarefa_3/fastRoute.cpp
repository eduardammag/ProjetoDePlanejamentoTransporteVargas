#include <iostream> 
#include <stack>
#include <queue>
#include <cfloat>
#include <climits> 
#include <unordered_set>
#include <string>

#include "fastRoute.h"
#include "vertexAndEdge.h"

using namespace std; 

// Calculando o melhor caminho a pé 
pair<vector<Edge*>, int> dijkstraFoot(Vertex* start,  Vertex* destination, const vector<vector<tuple<int, Edge*>>> adjacencyList) 
{
    // inicializa as estruturas auxilares
    int numVertices = adjacencyList.size();
    vector<bool> checked(numVertices, false);
    vector<pair<int, Edge*>> parent(numVertices, {-1, nullptr});
    vector<int> distance(numVertices, INT_MAX);

    // Priority queue para gerenciar os vértices a serem processados
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

    // Inicializa o heap com o vértice inicial
    parent[start->id()] = {start->id(), nullptr};
    distance[start->id()] = 0;
    pq.emplace(0, start->id()); 

    // itera pelo heap
    while (!pq.empty()) 
    {
        int v1 = pq.top().second;
        pq.pop();

        if (checked[v1]) continue;
        checked[v1] = true;

        // Explora as arestas saindo de v1
        for (const auto& tupleData : adjacencyList[v1]) 
        {
            // ID do vértice vizinho
            int v2 = get<0>(tupleData); 
            // Ponteiro para a aresta
            Edge* edge = get<1>(tupleData); 
            int cost = edge->distance();

            if (distance[v1] + cost < distance[v2]) 
            {
                parent[v2] = {v1, edge};
                distance[v2] = distance[v1] + cost;
                // Atualiza o heap com a nova distância
                pq.emplace(distance[v2], v2); 
            }
        }
    }
    
    // Tempo percorrendo a distância total com a velocidade de 83 metros por minutos (5Km\h)
    int tempo = distance[destination->id()] / 83.33;
    
    // Reconstrução do caminho
    stack<Edge*> edgeStack;
    for (int v = destination->id(); v != start->id(); v = parent[v].first) 
    {
        if (v == -1) 
        {
            cout<< "Caminho não encontrado." << endl;
            return {{}, -1};
        }
        // Adiciona a aresta usada para chegar ao vértice atual
        edgeStack.push(parent[v].second); 
    }
    
    // Converte a pilha em uma lista
    vector<Edge*> edgeList;
    while (!edgeStack.empty()) 
    {
        edgeList.push_back(edgeStack.top());
        edgeStack.pop();
    }
    
    // Retorna a lista de arestas e o tempo em minutos
    return {edgeList, tempo};
}

// Função para encontrar a aresta correspondente a um número de imóvel em uma rua.
pair<Edge*, Vertex*> findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges) 
{
    // Distância acumulada ao longo da rua.
    int current_distance = 0; 

    // Ordena as arestas para garantir a sequência crescente dos IDs dos vértices.
    vector<Edge*> sorted_edges = edges;
    sort(sorted_edges.begin(), sorted_edges.end(), [](Edge* a, Edge* b) {
        return min(a->vertex1()->id(), a->vertex2()->id()) < min(b->vertex1()->id(), b->vertex2()->id());
    });

    for (Edge* edge : sorted_edges) 
    {
        if (edge->id_street() == street && edge->id_zipCode() == id_zipCode) 
        {
            int id_start = min(edge->vertex1()->id(), edge->vertex2()->id());
            int id_end = max(edge->vertex1()->id(), edge->vertex2()->id());
    
            int start_number = current_distance + 1;
            int end_number = current_distance + edge->distance();
    
            // Verifica se o número do imóvel está no intervalo desta aresta
            if (number_build >= start_number && number_build <= end_number) 
            {
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
    
            // Atualiza a distância acumulada.
            current_distance = end_number; 
        }
    }

    return {nullptr, nullptr}; // Retorna nullptr caso nenhuma aresta seja encontrada.
}

// Função que calcula o menor caminho e custo usando Dijkstra se locomovendo com táxi
tuple<float, float, vector<Edge*>> dijkstraTaxi(
    const vector<vector<tuple<int, Edge*>>>& directedAdj,
    int start, int destination) 
{
    // Tarifa do táxi por metro percorrido
    float taxiRatePerMeter = 0.01f;  
    // Velocidade máxima (em km/h)
    float maxSpeedKmH = 30.0f;      
    
    // Número de vértices no grafo
    int n = directedAdj.size();

    // Vetores de controle 
    vector<float> minTime(n, FLT_MAX);  
    vector<float> totalCost(n, FLT_MAX);  
    vector<int> parent(n, -1);  
    vector<Edge*> edgeUsed(n, nullptr); 

    // Fila de prioridade para o Dijkstra, ordenada pelo menor tempo
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> pq;

    // Inicialização do vértice de início
    minTime[start] = 0;
    totalCost[start] = 0;
    pq.push({0, start}); 

    // Algoritmo principal do Dijkstra
    while (!pq.empty())
    {
        // Extrai o vértice com o menor tempo acumulado
        float currTime = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        for (const auto& [v, edge] : directedAdj[u])
        {
            // Verifica se a taxa de tráfego é válida
            if (edge->trafficRate() <= 0) continue;

            // Calcula a velocidade real considerando a taxa de tráfego
            float realSpeedKmH = maxSpeedKmH * edge->trafficRate();
            float realSpeedMPS = realSpeedKmH * (1000.0f / 3600.0f);  // Velocidade em metros por segundo

            float edgeTime = edge->distance() / (realSpeedMPS * 60.0f);

            // Relaxamento
            if (currTime + edgeTime < minTime[v])
            {   // Atualizando os parâmetros
                minTime[v] = currTime + edgeTime;  
                totalCost[v] = totalCost[u] + edge->distance() * taxiRatePerMeter;  
                parent[v] = u;  
                // Armazena a aresta utilizada
                edgeUsed[v] = edge;  
                pq.push({minTime[v], v}); 
            }
        }
    }

    // Reconstrução do caminho e das arestas percorridas
    vector<Edge*> edges;  
    if (minTime[destination] < FLT_MAX)
    {  
        for (int v = destination; v != start; v = parent[v]) 
        {
            // Armazena a aresta utilizada
            if (v != -1) edges.push_back(edgeUsed[v]);  
        }
        // Inverte para a ordem
        reverse(edges.begin(), edges.end());  
    }

    // Vértice não alcançável
    if (! minTime[destination] < FLT_MAX) 
    {
        cout << "Não foi possível encontrar um caminho válido." << endl;
        return {FLT_MAX, FLT_MAX, {}};
        
    } 

    // Retorna o custo total, tempo total e as arestas percorridas
    return {totalCost[destination], minTime[destination], edges};
}

//Função para verificar se a aresta destino está na mesma região que a aresta inicial
bool isTheSameRegion(const Edge* edge1, const Edge* edge2) 
{
    return edge1->id_zipCode() == edge2->id_zipCode();
}

//Função para verificar o melhor caminho entre dois vértices (de táxi ou a pé), considerando o orçamento
tuple<vector<Edge*>, float, int, string> findBestPath(Vertex* start, Vertex* destination, 
                           const vector<vector<tuple<int, Edge*>>>& adjacencyList, 
                           const vector<vector<tuple<int, Edge*>>>& directedAdj, float budget) 
{
    Vertex* startVertex = start;
    Vertex* destinationVertex = destination;

    // Caminho de táxi usando Dijkstra
    auto [carCost, carTime, carPath] = dijkstraTaxi(directedAdj, startVertex->id(), destinationVertex->id());
    // Caminho a pé usando Dijkstra
    auto [footPath, footTime] = dijkstraFoot(startVertex, destinationVertex, adjacencyList);
    // Se a pé for mais rápido do que de táxi é sinal que estamos perto
    
    if (!footPath.empty() && footTime < carTime) 
    {
        return {footPath, 0, footTime, "foot"};
    }
    
    // Se couber no orçamento
    if (carCost <= budget) 
    {
        return {carPath, carCost, carTime, "cab"};
    }


    // Caso em que a pé é demmorado e táxi não cabe no orçamento
    return {{},0,0, "nulo"};
}


//Função que encontra o caminho entre as estações de duas regiões específicas e 
// retorna a tupla contendo IDs de vértices no caminho e as arestas, os custos entre vértices, 
// Vetor com os IDs dos vértices das estações do caminho
tuple<vector<pair<int, Edge*>>, vector<int>, vector<int>> findPathBetweenStation(
    const vector<vector<tuple<int, Edge*>>>& mstadj, 
    int region1CEP, int region2CEP) 
    {
    // Caminho completo (vértices visitados e arestas)
    vector<pair<int, Edge*>> path;
    // Custos entre estações de metrô
    vector<int> segmentDistances; 
    // Apenas estações no caminho
    vector<int> stations; 

    // Localiza as estações de metrô associadas aos CEPs fornecidos
    Vertex* station1 = nullptr;
    Vertex* station2 = nullptr;

    // Busca as estações de metrô diretamente na lista de adjacência
    for (size_t i = 0; i < mstadj.size(); ++i) 
    {
        for (const auto& [neighbor, edge] : mstadj[i]) 
        {
            if (edge->id_zipCode() == region1CEP) 
            {
                if (edge->vertex1()->isMetroStation())
                {
                    station1 = edge->vertex1();
                } else if (edge->vertex2()->isMetroStation()) 
                {
                    station1 = edge->vertex2();
                }
            }
            if (edge->id_zipCode() == region2CEP) 
            {
                if (edge->vertex1()->isMetroStation()) 
                {
                    station2 = edge->vertex1();
                } else if (edge->vertex2()->isMetroStation()) 
                {
                    station2 = edge->vertex2();
                }
            }
            if (station1 && station2) break; // Se ambas as estações foram encontradas, encerra a busca
        }
        if (station1 && station2) break;
    }

    // Verifica se ambas as estações foram encontradas
    if (!station1 || !station2) {
        throw runtime_error("Não foi possível encontrar estações para os CEPs fornecidos.");
    }

    // DFS para encontrar o caminho
    // Para reconstruir o caminho
    vector<pair<int, Edge*>> parent(mstadj.size(), {-1, nullptr}); 
    unordered_set<int> visited; 

    // Função auxiliar para DFS
    function<bool(int)> dfs = [&](int current) -> bool 
    {
        visited.insert(current);

        // Se encontrou o destino, retorna verdadeiro
        if (current == station2->id()) return true; 

        for (const auto& [neighbor, edge] : mstadj[current]) 
        {
            if (!visited.count(neighbor)) 
            { 
                parent[neighbor] = {current, edge}; 
                if (dfs(neighbor)) return true; 
            }
        }
        return false; // Não encontrou o destino nesse caminho
    };

    // Inicializa a DFS a partir da estação inicial
    parent[station1->id()] = {-1, nullptr};
    if (!dfs(station1->id())) 
    {
        throw runtime_error("Não foi possível encontrar um caminho entre as estações fornecidas.");
    }

    // Reconstrói o caminho a partir do mapa de pais
    int current = station2->id();
    while (current != -1) 
    {
        path.push_back(parent[current]);
        current = parent[current].first;
    }
    // Reverte o caminho para a ordem correta
    reverse(path.begin(), path.end()); 

    // Filtra apenas as estações de metrô no caminho
    for (auto vertexPath : path)
    {
        int vertex  = get<0>(vertexPath);
        for (const auto& [neighbor, edge] : mstadj[vertex]) 
        {
            if ((edge->vertex1()->id() == vertex && edge->vertex1()->isMetroStation()) ||
                (edge->vertex2()->id() == vertex && edge->vertex2()->isMetroStation())) 
                {
                if (stations.empty() || stations.back() != vertex) 
                {
                    // Evita duplicar a mesma estação
                    stations.push_back(vertex); 
                }
                break;
            }
        }
    }

    // Calcula os custos entre estações de metrô
    for (size_t i = 1; i < stations.size(); ++i) 
    {
        int stationStart = stations[i - 1];
        int stationEnd = stations[i];
        int segmentCost = 0;

        for (const auto& [vertex, edge] : path) 
        {
            if (edge && (vertex == stationStart || vertex == stationEnd || segmentCost > 0)) 
            {
                segmentCost += edge->distance();
            }
            if (vertex == stationEnd) break;
        }
        segmentDistances.push_back(segmentCost);
    }
    
    // Retorna o caminho com vértces e arestas visitados, distâncias entre estções e estções presentes no caminho
    return {path, segmentDistances, stations};
}


tuple<vector<pair<Edge*, string>>, int, float> fastestRoute(Vertex* startVertex, Vertex* destinationVertex, 
                    Edge* startEdge, Edge* destEdge, int hour, float budget, 
                    const vector<vector<tuple<int, Edge*>>>&adjList, 
                    const vector<vector<tuple<int, Edge*>>>&dirAdjList, 
                    const vector<vector<tuple<int, Edge*>>>& mstadj)
{
    // Preço para pegar o metro 
    int subwayTicket = 10;
    int duration = 0;
    // Percurso e locomoção percorrida a cada aresta
    vector<pair<Edge*, string>> route;
    // Analisando se conseguimos pagar todo o percurso de táxi ou
    // se estamos tão próximo que é melhor ir de a pé
    auto [wholePath, wholeCost, wholeTime, locomation] = findBestPath( startVertex, destinationVertex, adjList, dirAdjList, budget);

    // Se achou uma locomoção válida
    if (!wholePath.empty())
    {
        budget -= wholeCost;
        for (Edge* edge: wholePath)
        {
            // Cada aresta é associada ao meio de locomoção
            route.push_back({edge, locomation});
        }
        return {route, wholeTime, budget};
    }

    // Se estivemors na mesma região então não devemos considerar o metrô
    // e como não temos orçamento para táxi deve-se ir a pé
    // ou se não tivermos orçamento nem para metrô
    if (isTheSameRegion(startEdge, destEdge) || budget < subwayTicket)
    {
        auto [footPath, footTime] = dijkstraFoot(startVertex, destinationVertex, adjList);
        for (Edge* edge: footPath)
        {
            route.push_back({edge, "foot"});
        }
        return {route, footTime, budget};
    }

    // Pega o metro
    budget -= subwayTicket;

    // Indo ao metro da mesmo região 
    auto [path, segmentDistances, stations] = findPathBetweenStation(mstadj, startEdge->id_zipCode(), destEdge->id_zipCode());

    Vertex* regionSubway =  new Vertex(true, stations[0]);

    // Verificando se tem orçamento para ir até o metrô de táxi
    auto [carCost, carTime, carPath] = dijkstraTaxi(dirAdjList, startVertex->id(), regionSubway->id());
    if (budget - carCost >= 0)
    {
        duration += carTime;
        budget -= carCost;
        for (Edge* edge: carPath)
        {
            route.push_back({edge, "cab"});
        }
    }
    // Se não vai de a pé até a estação da região
    else
    {
        auto [footPath, footTime] = dijkstraFoot(startVertex, regionSubway, adjList);
        duration += footTime;
        for (Edge* edge: footPath)
        {
            route.push_back({edge, "foot"});
        }
    }

    // Agora chegou na primeira estação para pegar o metro 
    // Trens só chegam em horários divisíveis por 20
    int waiting = (hour + duration) % 20;
    duration += waiting;
    // Estações que devem ser visitados se percorremos até a região destino de metrô
    int stationsLeft = stations.size() - 1;
    int currentStation = 0;

    // Velocidade do metrô em metros por minuto
    double speed = 1166.67;
    
    // Variáveis de distância
    tuple<vector<Edge*>, float, int, string> result;
    vector<Edge*>leftPath;
    float leftCost;
    int leftTime;
    string leftLocomation;
    Vertex* currStatVertex = new Vertex(true, stations[currentStation]);
    
    // Enquanto tiver linhas de metô disponíveis
    while (stationsLeft)
    {
        // Distancia entre estações dividido pela velocidade
        int distanceBetween = segmentDistances[currentStation];
        int subwayDuration = distanceBetween / speed;
        duration += subwayDuration;
        
        // Chega na próxima estação
        currentStation+=1;
        stationsLeft -=1;
        Vertex* currStatVertex = new Vertex(true, stations[currentStation]);
        // Arestas percorrridas de metrô
        int startPath = stations.size() - stationsLeft - 2;
        int stopPath = stations.size() - stationsLeft - 1;
        for (int i = startPath; i< stopPath; i++)
        {
            auto vertexAndEdge = path[i];
            Edge* edge = get<1>(vertexAndEdge);
            route.push_back({edge, "subway"});
        }

        // Verifico se consigo ir de táxi a partir da nova estação até o destino
        // ou pode ir de a pé se for mais rápido
        result = findBestPath(currStatVertex, destinationVertex, adjList, dirAdjList, budget);
        leftPath = std::get<0>(result);
        leftCost = std::get<1>(result);
        leftTime = std::get<2>(result);
        leftLocomation = std::get<3>(result);

        // Se achou uma locomoção válida até o destino final
        if (!leftPath.empty())
        {
            duration += leftTime;
            budget -= leftCost;
            for (Edge* edge: leftPath)
            {
                // Cada aresta associada ao meio de locomoção
                route.push_back({edge, locomation});
            }
            return {route, duration, budget};
        }

        // Ainda não compensa ou não conseguimos ir de táxi 
        // se restar estações continua dentro do laço até a linha de metro acabar
    }

    // Chegou até a estação da região do destino, acabou o metro disponível
    //  Verificamos qual é melhor dentro do orçamento 

    result = findBestPath(currStatVertex, destinationVertex, adjList, dirAdjList, budget);
    leftPath = std::get<0>(result);
    leftCost = std::get<1>(result);
    leftTime = std::get<2>(result);
    leftLocomation = std::get<3>(result);
    
    // Se achou uma locomoção válida
    if (!leftPath.empty())
    {
        duration += leftTime;
        budget -= leftCost;
        for (Edge* edge: leftPath)
        {
            route.push_back({edge, locomation});

        }
        return {route, duration, budget};
    }
    
    // Se não achou nenhuma válida nos resta andar até o destino
    auto [footPath, footTime] = dijkstraFoot(currStatVertex, destinationVertex, adjList);
    duration += footTime;
    for (Edge* edge: footPath)
    {
        route.push_back({edge, "foot"});
    }

    return {route, duration, budget};
}