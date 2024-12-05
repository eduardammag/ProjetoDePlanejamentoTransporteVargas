#include "fastRoute.h"
#include "vertexAndEdge.h"
#include <iostream> 
#include <stack>
#include <queue>
#include <cfloat>
#include <unordered_set>
#include <string>


using namespace std; 

// Calculando o melhor caminho a pé 
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
tuple<vector<Edge*>, float, int, string> findBestPath(Vertex* start, Vertex* destination, 
                           const vector<vector<tuple<int, Edge*>>>& adjacencyList, 
                           const vector<vector<tuple<int, Edge*>>>& directedAdj, float budget) 
{
    // Edge* startEdge = start.first;
    Vertex* startVertex = start;
    Vertex* destinationVertex = destination;

    // Caminho de carro usando Dijkstra
    auto [carCost, carTime, carPath] = dijkstraTaxi(directedAdj, startVertex->id(), destinationVertex->id());
    
    if (carCost <= budget) {
        cout << "Recomendado ir de carro, custo: R$ " << carCost << ", tempo: " << carTime << " minutos." << endl;
        return {carPath, carCost, carTime, "cab"};
    }

    // Caminho a pé usando Dijkstra
    auto [footPath, footTime] = dijkstraFoot(startVertex, destinationVertex, adjacencyList);
    if (!footPath.empty() && footTime < carTime) {
        cout << "Recomendado ir a pé, tempo: " << footTime << " minutos." << endl;
        return {footPath, 0, footTime, "foot"};
    }

    cout << "Nenhum caminho a pé mais rápido que o de carro foi encontrado, mas o custo de carro excede o orçamento." << endl;
    return {{},0,0, "nulo"};
}


//Função que encontra o caminho entre as estações de duas regiões específicas e 
// retorna a tupla contendo IDs de vértices no caminho, os custos entre vértices, 
// Vetor com os IDs dos vértices das estações do caminho
tuple<vector<pair<int, Edge*>>, vector<int>, vector<int>> findPathBetweenStation(
    const vector<vector<tuple<int, Edge*>>>& mstadj, // Lista de adjacência da MST
    int region1CEP, // CEP da região 1
    int region2CEP // CEP da região 2
) {
    vector<pair<int, Edge*>> path;// Caminho completo (vértices visitados)
    vector<int> segmentDistances; // Custos entre estações de metrô
    vector<int> stations; // Apenas estações de metrô no caminho

    // Localiza as estações de metrô associadas aos CEPs fornecidos
    Vertex* station1 = nullptr;
    Vertex* station2 = nullptr;

    // Busca as estações de metrô diretamente na lista de adjacência
    for (size_t i = 0; i < mstadj.size(); ++i) {
        for (const auto& [neighbor, edge] : mstadj[i]) {
            if (edge->id_zipCode() == region1CEP) {
                if (edge->vertex1()->isMetroStation()) {
                    station1 = edge->vertex1();
                } else if (edge->vertex2()->isMetroStation()) {
                    station1 = edge->vertex2();
                }
            }
            if (edge->id_zipCode() == region2CEP) {
                if (edge->vertex1()->isMetroStation()) {
                    station2 = edge->vertex1();
                } else if (edge->vertex2()->isMetroStation()) {
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

    // Busca em profundidade (DFS) para encontrar o caminho
     vector<pair<int, Edge*>> parent(mstadj.size(), {-1, nullptr}); // Para reconstruir o caminho
    unordered_set<int> visited; // Conjunto para marcar vértices visitados

    // Função auxiliar para DFS
    function<bool(int)> dfs = [&](int current) -> bool {
        visited.insert(current); // Marca o vértice atual como visitado

        if (current == station2->id()) return true; // Se encontrou o destino, retorna verdadeiro

        for (const auto& [neighbor, edge] : mstadj[current]) {
            if (!visited.count(neighbor)) { // Se o vizinho ainda não foi visitado
                parent[neighbor] = {current, edge}; // Define o pai do vizinho como o vértice atual
                if (dfs(neighbor)) return true; // Continua a busca recursiva
            }
        }
        return false; // Não encontrou o destino nesse caminho
    };

    // Inicializa a DFS a partir da estação inicial
    // parent[station1->id()] = {-1, nullptr};
    parent[station1->id()] = {-1, nullptr}; // A estação inicial não tem pai
    if (!dfs(station1->id())) {
        throw runtime_error("Não foi possível encontrar um caminho entre as estações fornecidas.");
    }

    // Reconstrói o caminho a partir do mapa de pais
    int current = station2->id();
    while (current != -1) {
        path.push_back(parent[current]);
        current = parent[current].first;
    }
    reverse(path.begin(), path.end()); // Reverte o caminho para a ordem correta

    // Filtra apenas as estações de metrô no caminho
    for (auto vertexPath : path) {
        int vertex  = get<0>(vertexPath);
        for (const auto& [neighbor, edge] : mstadj[vertex]) {
            if ((edge->vertex1()->id() == vertex && edge->vertex1()->isMetroStation()) ||
                (edge->vertex2()->id() == vertex && edge->vertex2()->isMetroStation())) {
                if (stations.empty() || stations.back() != vertex) {
                    stations.push_back(vertex); // Evita duplicar a mesma estação
                }
                break;
            }
        }
    }

    // Calcula os custos entre estações de metrô
    for (size_t i = 1; i < stations.size(); ++i) {
        int stationStart = stations[i - 1];
        int stationEnd = stations[i];
        int segmentCost = 0;

        for (const auto& [vertex, edge] : path) {
            if (edge && (vertex == stationStart || vertex == stationEnd || segmentCost > 0)) {
                segmentCost += edge->distance();
            }
            if (vertex == stationEnd) break;
        }
        segmentDistances.push_back(segmentCost);
    }

    return {path, segmentDistances, stations};
}


tuple<vector<pair<Edge*, string>>, int, float> fastestRoute(Vertex* startVertex, Vertex* destinationVertex, 
                    Edge* startEdge, Edge* destEdge, int hour, float budget, 
                    const vector<vector<tuple<int, Edge*>>>&adjList, 
                    const vector<vector<tuple<int, Edge*>>>&dirAdjList, 
                    const vector<vector<tuple<int, Edge*>>>& mstadj)
{
    cout << "Entrei " << endl;
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
    // e como não temos orçamento para táxi deve-se ir de a pé
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

    Vertex*  regionSubway =  new Vertex(true, stations[0]);

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
    // Vai de a pé até a estaçõ da região
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
    int stationsLeft = stations.size() - 1;
    int currentStation = 0;

    // Velocidade do metrô em metros por minuto
    double speed = 1166.67;
    tuple<vector<Edge*>, float, int, string> result;
    vector<Edge*>leftPath;
    float leftCost;
    int leftTime;
    string leftLocomation;
    Vertex* currStatVertex = new Vertex(true, stations[currentStation]);
    
    while (stationsLeft)
    {
        // Vou para próxima estação
        int distanceBetween = segmentDistances[currentStation];
        // Distancia entre estações dividido pela velocidade
        int subwayDuration = distanceBetween / speed;
        duration += subwayDuration;

        currentStation+=1;
        stationsLeft -=1;
        Vertex* currStatVertex = new Vertex(true, stations[currentStation]);

        // Verifico se consigo ir de táxi a partir de agora até o destino
        // ou pode ir de a pé se for mais rápido
        result = findBestPath(currStatVertex, destinationVertex, adjList, dirAdjList, budget);
        leftPath = std::get<0>(result);
        leftCost = std::get<1>(result);
        leftTime = std::get<2>(result);
        leftLocomation = std::get<3>(result);
        
        int startPath = stations.size() - stationsLeft - 2;
        int stopPath = stations.size() - stationsLeft - 1;
        for (int i = startPath; i< stopPath; i++)
        {
            auto vertexAndEdge = path[i];
            Edge* edge = get<1>(vertexAndEdge);
            route.push_back({edge, "subway"});
        }
        
        

        // Se achou uma locomoção válida
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
        // se restar estações continua até a linha de metro acabar
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
            // Cada aresta associada ao meio de locomoção
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