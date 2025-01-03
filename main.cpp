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
            cout << "Região " << i << ": Vértice ótimo = " << optimalVertex->id() << endl;
            cout << "Vértice " << optimalVertex->id() << " agora é uma estação de metrô: "
                 << (optimalVertex->isMetroStation() ? "Sim" : "Não") << endl;
            optimalVertices.push_back(optimalVertex); // Adiciona o vértice ótimo à lista
        } else {
            cout << "Região " << i << ": Não foi possível determinar o vértice ótimo.\n";
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Criação de vértices (representando cruzamentos ou pontos importantes).
    Vertex* v1 = new Vertex(false, 1); // ID: 1
    Vertex* v2 = new Vertex(false, 2); // ID: 2
    Vertex* v3 = new Vertex(false, 3); // ID: 3
    Vertex* v4 = new Vertex(false, 4); // ID: 4

    // Criação de arestas (representando segmentos de rua).
    Edge* e1 = new Edge(10, v1, v2, 1.0f,  1, 1001, 1); // Distância: 10, Rua ID: 1, CEP: 1001
    Edge* e2 = new Edge(15, v2, v3, 1.0f,  2, 1001, 1); // Distância: 15, Rua ID: 1, CEP: 1001
    Edge* e3 = new Edge(20, v3, v4, 1.0f,  3, 1001, 1); // Distância: 20, Rua ID: 1, CEP: 1001
    

    // Vetor de arestas.
    vector<Edge*> edges_test = {e1, e2, e3};

    // Teste: Encontrar uma aresta correspondente a um número de imóvel.
    int street = 3564;       // ID da rua
    int id_zipCode = 213493; // CEP da rua
    int number_build = 3; // Número do imóvel

    pair<Edge*, Vertex*> foundEdge = findEdgeAddress(street, id_zipCode, number_build, edges);

    // Resultado do teste.
    if (foundEdge.first != nullptr && foundEdge.second != nullptr) {
        cout << "Aresta encontrada!" << endl;
        cout << "Rua ID: " << foundEdge.first->id_street()
             << ", Distância: " << foundEdge.first->distance()
             << ", Vértices: (" << foundEdge.first->vertex1()->id()
             << ", " << foundEdge.first->vertex2()->id() << ")" << endl;
    } else {
        cout << "Nenhuma aresta encontrada para o número de imóvel informado." << endl;
    }
    

    // Dijkstra a pé 
    pair<vector<Edge*>, int> teste; 
    
    Vertex* comeco = vertices[0];
    Vertex* fim = vertices[35];
    teste = dijkstraFoot(comeco,  fim, adjacencyList);
    
    auto& edgesDijkstra =  get<0>(teste);
    cout << "Arestas passadas ";
    for (const auto& arestaDak : edgesDijkstra)
    {
        cout << arestaDak->idEdge() << " ";
    }
    
    cout << endl;
    
    vector<vector<tuple<int, Edge*>>>  mstadj;
    vector<vector<Edge*>> detailedPaths;
    mstadj =  conect_metro(vertices, adjacencyList, optimalVertices, detailedPaths);
    cout << "processou" << endl;
    
    printDirectedAdjacencyList(mstadj);
    
  
    //TESTE Melhor Caminho (carro ou táxi)
    
    vector<vector<tuple<int, Edge*>>> directedAdj;
    directedAdj = adjacencyList;
    convertToDirected(directedAdj);

    // Definindo vértices e arestas de início e destino
    Vertex* startVertex = vertices[0];
    Vertex* destinationVertex = vertices.back();
    Edge* startEdge = edges[0];
    Edge* destinationEdge = edges.back();


    // Definição de orçamento máximo
    float budget = 50.0f;
    
    cout << "\n" <<endl;
    
    // Encontrar melhor caminho
    tuple<vector<Edge*>, float, int, string> bestPath = findBestPath(startVertex, destinationVertex, adjacencyList, directedAdj, budget);
    if (!get<0>(bestPath).empty()) {
        cout << "Caminho recomendado (IDs das arestas): ";
        for (Edge* edge : get<0>(bestPath)) {
            cout << edge->idEdge() << " ";
        }
        cout << endl;
    } else {
        cout << "Nenhum caminho viável foi encontrado." << endl;
    }


    tuple<vector<pair<Edge*, string>>, int, float> testefinal;
    Edge* destEdge = edges[0];
    Edge* estatEdge = edges[250];
    cout << estatEdge->id_zipCode() << " " << destEdge->id_zipCode() << endl;
    cout << "A aresta existe" << endl;
    cout << "Vertex válido" << estatEdge->idEdge() <<  endl; 
    
    testefinal =  fastestRoute(estatEdge->vertex1(), destEdge->vertex1(), estatEdge,destEdge, 317, 0, adjacencyList, directedAdj, mstadj);
    vector<pair<Edge*, string>> patheste = get<0>(testefinal);
    for (auto [edge, locomation]: patheste)
    {
        cout << locomation << endl;
    }
    // cout << get<2>(testefinal)
    cout << get<2>(testefinal) << endl;
    
    cout << "processou" << endl;

    // Limpeza da memória alocada
    for (Vertex* vertex : vertices) delete vertex;
    for (Edge* edge : edges) delete edge;
    
    return 0;
}