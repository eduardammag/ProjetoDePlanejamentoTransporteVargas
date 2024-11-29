#include <iostream>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"
#include "steinerTree.h"
#include <algorithm>
#include <unordered_map>
using namespace std;

// Função para calcular o custo total entre dois vértices ótimos
float calcularCustoTotal(const vector<tuple<int, Edge*>>& path) {
    float totalCost = 0.0f;
    for (const auto& [_, edge] : path) {
        totalCost += edge->excavationCost();  // Soma o custo da escavação de cada aresta
    }
    return totalCost;
}
vector<vector<Edge*>> detailedPaths;  // Declaração do vetor para armazenar os caminhos detalhados


int main() {
    string jsonFilePath = "city_graph.json";  // Caminho para o arquivo JSON com os dados da cidade

    vector<Vertex*> vertices;  // Vetor de vértices (estações)
    vector<Edge*> allEdges;    // Vetor de todas as arestas (rotas entre as estações)

    try {
        // Parseia o arquivo JSON e preenche os vetores de vértices e arestas
        parseJsonFile(jsonFilePath, vertices, allEdges);
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    // Matrizes e listas para representação do grafo
    vector<vector<Edge*>> adjacencyMatrix;
    vector<vector<tuple<int, Edge*>>> adjacencyList;

    // Gera a matriz de adjacência (representação do grafo)
    generateAdjacencyMatrix(vertices, allEdges, adjacencyMatrix);

    // Gera a lista de adjacência (converte as arestas para o formato necessário)
    adjacencyList.resize(vertices.size());
    for (const auto& edge : allEdges) {
        int u = edge->vertex1()->id();
        int v = edge->vertex2()->id();
        adjacencyList[u].push_back({v, edge});
        adjacencyList[v].push_back({u, edge});
    }

    // Agrupa as arestas por CEP
    unordered_map<int, vector<Edge*>> edgesByCepMap;
    for (const auto& edge : allEdges) {
        edgesByCepMap[edge->id_zipCode()].push_back(edge);
    }

    // Imprime as arestas agrupadas por CEP
    printEdgesGroupedByCepVector(edgesByCepMap);

    // Converte o mapa de arestas agrupadas para um vetor de vetores
    vector<vector<Edge*>> edgesGrouped;
    for (const auto& [key, edgeList] : edgesByCepMap) {
        edgesGrouped.push_back(edgeList);
    }

    // Processa cada região individualmente, buscando o vértice ótimo para cada uma
    vector<Vertex*> optimalVertices;
    for (size_t i = 0; i < edgesGrouped.size(); i++) {
        const vector<Edge*>& regionEdges = edgesGrouped[i];

        // Converte a região para o formato esperado pela função que encontra o vértice ótimo
        vector<vector<Edge*>> currentRegion = {regionEdges};

        // Encontra o vértice ótimo da região
        Vertex* optimalVertex = findOptimalVertexFast(currentRegion, adjacencyList);
        if (optimalVertex) {
            cout << "Região " << i << ": Vértice ótimo = " << optimalVertex->id() << endl;
            cout << "Vértice " << optimalVertex->id() << " agora é uma estação de metrô: "
                 << (optimalVertex->isMetroStation() ? "Sim" : "Não") << endl;
            optimalVertices.push_back(optimalVertex);  // Adiciona o vértice ótimo na lista
        } else {
            cout << "Região " << i << ": Não foi possível determinar o vértice ótimo.\n";
        }
    }

    // Chamando a função steinerTree para os vértices ótimos
    vector<Edge*> steinerEdges = steinerTree(vertices, adjacencyList, optimalVertices, detailedPaths);


    
    
    // Imprime as arestas da Árvore de Steiner
    cout << "\nArestas da Árvore de Steiner:" << endl;
    for (Edge* edge : steinerEdges) {
        cout << "Aresta: " << edge->vertex1()->id() << " - " << edge->vertex2()->id() 
             << " com custo de escavação: " << edge->excavationCost() << endl;
    }

    // Imprime a Árvore de Kruskal com os custos entre as estações ótimas
    cout << "\nÁrvore de Kruskal (custo entre as estações ótimas):" << endl;
    for (size_t i = 0; i < steinerEdges.size(); ++i) {
        Edge* edge = steinerEdges[i];
        int u = edge->vertex1()->id();
        int v = edge->vertex2()->id();

        // Calcula o custo total entre os vértices ótimos
        float totalCost = calcularCustoTotal(adjacencyList[u]);

        cout << "Custo da estação " << u << " para estação " << v 
             << " é: " << totalCost << endl;
    }

    // Variáveis para armazenar as arestas agregadas e os caminhos detalhados
    vector<Edge*> aggregatedEdges; // Renomeie para evitar conflito
    vector<vector<Edge*>> detailedPaths;  // Declaração do vetor para armazenar os caminhos detalhados

    // Chamada para calcular a Árvore de Steiner com caminhos agregados
    aggregatedEdges = steinerTree(vertices, adjacencyList, optimalVertices, detailedPaths);

    // Imprime as arestas agregadas e os caminhos detalhados
    cout << "\nÁrvore de Steiner (arestas agregadas):" << endl;
    for (const auto& edge : aggregatedEdges) {
        cout << "Aresta de " << edge->vertex1()->id() << " a " << edge->vertex2()->id() 
             << " com custo total " << edge->excavationCost() << endl;
    }

    /*
    // Imprime os caminhos detalhados
    cout << "\nCaminhos detalhados entre terminais:" << endl;
    for (size_t i = 0; i < detailedPaths.size(); ++i) {
        cout << "Caminho detalhado para aresta agregada " << i + 1 << ":" << endl;
        for (const Edge* edge : detailedPaths[i]) {
            cout << "  Aresta de " << edge->vertex1()->id() << " a " << edge->vertex2()->id()
                 << " com custo " << edge->excavationCost() << endl;
        }
    }
    */

    // Libera memória
    for (auto& vertex : vertices) delete vertex;
    for (auto& edge : allEdges) delete edge;

    return 0;
}
