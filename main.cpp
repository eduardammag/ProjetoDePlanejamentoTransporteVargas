#include <iostream>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"
#include "connectingMetro.h"
#include "fastRoute.h"
#include <unordered_set>
#include <vector>

using namespace std;

int main() {
    string jsonFilePath = "city_graph_3_estacoes.json";

    vector<Vertex*> vertices;
    vector<Edge*> edges;

    try {
        parseJsonFile(jsonFilePath, vertices, edges);
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    // Matrizes e listas
    vector<vector<Edge*>> adjacencyMatrix;
    vector<vector<tuple<int, Edge*>>> adjacencyList;

    // Gera a matriz de adjacência
    generateAdjacencyMatrix(vertices, edges, adjacencyMatrix);

    // Gera a lista de adjacência
    generateAdjacencyList(vertices, edges, adjacencyList);

    // unordered_map<int, vector<Edge*>> edgesByCepMap;

    // // Agrupa arestas
    // for (const auto& edge : edges) {
    //     edgesByCepMap[edge->id_zipCode()].push_back(edge);
    // }

    // // Imprime agrupamento
    // printEdgesGroupedByCepVector(edgesByCepMap);

    // // Converte unordered_map para vector<vector<Edge*>>
    // vector<vector<Edge*>> edgesGrouped;
    // for (const auto& [key, edgeList] : edgesByCepMap) {
    //     edgesGrouped.push_back(edgeList);
    // }
    
    // // Lista para armazenar os vértices ótimos
    // vector<Vertex*> optimalVertices;
    
    // // Processar cada região individualmente
    // for (size_t i = 0; i < edgesGrouped.size(); i++) {
    //     const vector<Edge*>& regionEdges = edgesGrouped[i]; // Acessa apenas a lista da região atual
    
    //     // Converte a região atual para o formato esperado por findOptimalVertexFast
    //     vector<vector<Edge*>> currentRegion = {regionEdges};
    
    //     // Encontra o vértice ótimo para a região
    //     Vertex* optimalVertex = findOptimalVertexFast(currentRegion, adjacencyList);
    //     if (optimalVertex) {
    //         cout << "Região " << i << ": Vértice ótimo = " << optimalVertex->id() << endl;
    //         cout << "Vértice " << optimalVertex->id() << " agora é uma estação de metrô: "
    //              << (optimalVertex->isMetroStation() ? "Sim" : "Não") << endl;
    //         optimalVertices.push_back(optimalVertex); // Adiciona o vértice ótimo à lista
    //     } else {
    //         cout << "Região " << i << ": Não foi possível determinar o vértice ótimo.\n";
    //     }
    // }
    
    // // Gerar os caminhos otimizados
    // vector<vector<Edge*>> detailedPaths;

    // // Calcular a Árvore de Steiner
    // vector<Edge*> steinerEdges = conect_metro(vertices, adjacencyList, optimalVertices, detailedPaths);
    
    // // Exibe as rotas 
    // printBestRoutes(steinerEdges, detailedPaths);
    
    // int custo;
    // custo = totalCostSubway(steinerEdges, detailedPaths);
    
    // cout << "Custo total: " << custo << endl;

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
    int id_zipCode = 833775; // CEP da rua
    int number_build = 3; // Número do imóvel

    Edge* foundEdge = findEdgeAddress(street, id_zipCode, number_build, edges);

    // Resultado do teste.
    if (foundEdge) {
        cout << "Aresta encontrada!" << endl;
        cout << "Rua ID: " << foundEdge->id_street()
             << ", Distância: " << foundEdge->distance()
             << ", Vértices: (" << foundEdge->vertex1()->id()
             << ", " << foundEdge->vertex2()->id() << ")" << endl;
    } else {
        cout << "Nenhuma aresta encontrada para o número de imóvel informado." << endl;
    }
    

    // Dijkstra a pé 
    pair<vector<Edge*>, int> teste; 
    
    Vertex* comeco = vertices[0];
    Vertex* fim = vertices[200];
    teste = dijkstraFoot(comeco,  fim, adjacencyList);
    
    auto& edgesDijkstra =  get<0>(teste);
    cout << "Arestas passadas ";
    for (const auto& arestaDak : edgesDijkstra)
    {
        cout << arestaDak->idEdge() << " ";
    }
    
    cout << endl;





    // IDs dos vértices de origem e destino
    int startVertex = 210;  // Exemplo: origem do táxi (ID do vértice de origem)
    int destinationVertex = 217;  // Exemplo: destino do táxi (ID do vértice de destino)
    
    // Chama a função dijkstraTaxi para calcular o melhor caminho de táxi
    auto [totalCost, totalTime, taxiEdges] = dijkstraTaxi(directedAdjacencyList, startVertex, destinationVertex);
  





    return 0;
}