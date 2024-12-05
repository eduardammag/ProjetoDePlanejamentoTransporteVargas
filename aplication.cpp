#include <iostream>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"
#include "connectingMetro.h"
#include "fastRoute.h"
#include <unordered_set>
#include <vector>
#include <string>

using namespace std;

int main() {
    string jsonFilePath = "city_graph.json";

    vector<Vertex*> vertices;
    vector<Edge*> edges;

    try {
        parseJsonFile(jsonFilePath, vertices, edges);
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    // Gera a lista de adjacência
    vector<vector<tuple<int, Edge*>>> adjacencyList;
    generateAdjacencyList(vertices, edges, adjacencyList);

    unordered_map<int, vector<Edge*>> edgesByCepMap;

    // Agrupa arestas
    for (const auto& edge : edges) {
        edgesByCepMap[edge->id_zipCode()].push_back(edge);
    }

    // Imprime agrupamento
    printEdgesGroupedByCepVector(edgesByCepMap);

    // Converte unordered_map para vector<vector<Edge*>>
    vector<vector<Edge*>> edgesGrouped;
    for (const auto& [key, edgeList] : edgesByCepMap) {
        edgesGrouped.push_back(edgeList);
    }
    
    // Lista para armazenar os vértices ótimos
    vector<Vertex*> optimalVertices;
    
    // Processar cada região individualmente
    for (size_t i = 0; i < edgesGrouped.size(); i++) {
        const vector<Edge*>& regionEdges = edgesGrouped[i]; // Acessa apenas a lista da região atual
    
        // Converte a região atual para o formato esperado por findOptimalVertexFast
        vector<vector<Edge*>> currentRegion = {regionEdges};
    
        // Encontra o vértice ótimo para a região
        Vertex* optimalVertex = findOptimalVertexFast(currentRegion, adjacencyList);
        if (optimalVertex) {
            optimalVertices.push_back(optimalVertex);
        }
    }
    
    
    vector<vector<tuple<int, Edge*>>>  mstadj;
    vector<vector<Edge*>> detailedPaths;
    mstadj =  conect_metro(vertices, adjacencyList, optimalVertices, detailedPaths);
    
    vector<vector<tuple<int, Edge*>>> directedAdj;
    directedAdj = adjacencyList;
    convertToDirected(directedAdj);
    
    
    ///////////////////////////////////////
    // Passando o endereço da aresta
    tuple<vector<pair<Edge*, string>>, int, float> bestRoute;
    Edge* startEdge = edges[418];
    Edge* endEdge = edges[0];
    Vertex* start = startEdge->vertex1();
    Vertex* end = endEdge->vertex1();
    
    
    float budget = 15.0;
    int leavingHour = 317;
    
    bestRoute =  fastestRoute(start, end, startEdge,endEdge, leavingHour, budget, adjacencyList, directedAdj, mstadj);
    vector<pair<Edge*, string>> routeEdges = get<0>(bestRoute);
    
    
    for (auto [edge, locomotion]: routeEdges)
    {
        cout << "Aresta percorrida por meio de " << locomotion << endl;
    }
    cout << get<2>(bestRoute) << endl;
    
    // Limpeza da memória alocada
    for (Vertex* vertex : vertices) delete vertex;
    for (Edge* edge : edges) delete edge;
    
    return 0;
}



