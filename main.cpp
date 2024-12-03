#include <iostream>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"
#include "steinerTree.h"
#include <unordered_set>
#include <vector>

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

    // Matrizes e listas
    vector<vector<Edge*>> adjacencyMatrix;
    vector<vector<tuple<int, Edge*>>> adjacencyList;

    // Gera a matriz de adjacência
    generateAdjacencyMatrix(vertices, edges, adjacencyMatrix);

    // Gera a lista de adjacência
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
    

    // Gerar os caminhos otimizados
    vector<vector<Edge*>> detailedPaths;

    // Calcular a Árvore de Steiner
    vector<Edge*> steinerEdges = steinerTree(vertices, adjacencyList, optimalVertices, detailedPaths);

    
    // Imprimir caminhos detalhados correspondentes às arestas
    cout << "\nCaminhos detalhados:\n";
    for (size_t i = 0; i < steinerEdges.size(); ++i) {
        cout << "Caminho para a aresta " << steinerEdges[i]->idEdge() << ": ";
        for (const auto& edge : detailedPaths[i]) {
            cout << edge->idEdge() << " ";
        }
        cout << endl;
    }


    return 0;
}