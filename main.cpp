#include <iostream>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"
#include "steinerTree.h"
using namespace std;

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
    adjacencyList.resize(vertices.size());  // Tamanho da lista de adjacência igual ao número de vértices
    for (const auto& edge : allEdges) {
        int u = edge->vertex1()->id();  // Supondo que a aresta tenha vértices 1 e 2
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
    for (size_t i = 0; i < edgesGrouped.size(); i++) {
        const vector<Edge*>& regionEdges = edgesGrouped[i];  // Acessa as arestas da região atual
    
        // Converte a região para o formato esperado pela função que encontra o vértice ótimo
        vector<vector<Edge*>> currentRegion = {regionEdges};
    
        // Encontra o vértice ótimo da região
        Vertex* optimalVertex = findOptimalVertexFast(currentRegion, adjacencyList);
        if (optimalVertex) {
            cout << "Região " << i << ": Vértice ótimo = " << optimalVertex->id() << endl;
            cout << "Vértice " << optimalVertex->id() << " agora é uma estação de metrô: "
                 << (optimalVertex->isMetroStation() ? "Sim" : "Não") << endl;
        } else {
            cout << "Região " << i << ": Não foi possível determinar o vértice ótimo.\n";
        }
    }

    // Adicionando lógica para a árvore de Steiner
    vector<Vertex*> terminals;  // Agora é um vetor de ponteiros para Vertex
    for (auto& vertex : vertices) {
        if (vertex->isMetroStation()) { // Adiciona apenas vértices que são estações de metrô
            terminals.push_back(vertex);
        }
    }

    // Chama a função para calcular a árvore de Steiner
    vector<Edge*> steinerEdges = steinerTree(vertices, adjacencyList, terminals);

    // Imprime as arestas da árvore de Steiner
    cout << "\nÁrvore de Steiner:" << endl;
    for (const auto& edge : steinerEdges) {
        cout << "Aresta de " << edge->vertex1()->id() << " a " << edge->vertex2()->id() 
             << " com distância " << edge->distance() << endl;
    }

    // Preencher o vetor "stations" com as estações de metrô
    vector<Vertex*> stations;
    for (auto& vertex : vertices) {
        if (vertex->isMetroStation()) {  // Adiciona apenas as estações de metrô
            stations.push_back(vertex);
        }
    }

    // Calcular a distância entre todas as estações de metrô
    for (size_t i = 0; i < stations.size(); i++) {
        for (size_t j = i + 1; j < stations.size(); j++) {
            // Calcula a distância entre a estação[i] e a estação[j]
            vector<int> dist, parent;
            cptDijkstraFast(stations[i], parent, dist, adjacencyList);
            int distance = dist[stations[j]->id()]; // Distância entre a estação[i] e a estação[j]

            cout << "Distância entre a estação " << stations[i]->id() << " e " << stations[j]->id() 
                 << ": " << distance << " unidades" << endl;
        }
    }

    // Libera memória alocada para vértices e arestas
    for (auto& vertex : vertices) delete vertex;
    for (auto& edge : allEdges) delete edge;

    return 0;
}
