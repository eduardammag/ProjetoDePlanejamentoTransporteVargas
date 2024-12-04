#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <stdexcept>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"

using namespace std;

// Função para medir o tempo de execução para uma região específica
double measureTime(int regionCode, const vector<Edge*>& edges, const vector<Vertex*>& vertices) {
    auto start = chrono::high_resolution_clock::now();

    vector<Edge*> regionEdges;
    vector<vector<Edge*>> regionVertices; // Alterado para o tipo esperado pela função
    vector<vector<tuple<int, Edge*>>> adjacencyList;

    // Filtrar arestas pelo regionCode
    for (const auto& edge : edges) {
        if (edge->id_zipCode() == regionCode) {
            regionEdges.push_back(edge);
        }
    }

    // Adiciona regionEdges como a única região na lista
    regionVertices.push_back(regionEdges);

    // Gerar a lista de adjacência usando apenas as arestas da região
    generateAdjacencyList(vertices, regionEdges, adjacencyList);

    // Chamada da função usando o novo formato
    Vertex* optimalVertex = findOptimalVertexFast(regionVertices, adjacencyList);

    if (optimalVertex) {
        cout << "Vértice ótimo para a região " << regionCode << " = " << optimalVertex->id() << endl;
    } else {
        cout << "Nenhum vértice ótimo encontrado.\n";
    }

    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> duration = end - start;

    return duration.count();
}

int main() {
    // Suponha que você já tenha os caminhos dos arquivos JSON
    vector<Vertex*> vertices;
    vector<Edge*> edges;

    try {
        // Carregar dados de arquivo JSON
        vector<string> filePaths = {"city_graph_30x30.json"};
        
        // Carrega os grafos a partir dos arquivos JSON
        for (const auto& filePath : filePaths) 
        {
            parseJsonFile(filePath, vertices, edges);
        }

        // Processar para todas as regiões
        for (const auto& filePath : filePaths) {
            // Exemplo para extrair o número de regiões
            string filename = filePath.substr(filePath.find_last_of("/\\") + 1);
            int n = stoi(filename.substr(filename.find("city_graph_") + 11, filename.find("x") - 11));
            int numRegions = n / 2;

            // Chama a função para medir o tempo para cada região
            for (int regionCode = 1; regionCode <= numRegions; ++regionCode) {
                measureTime(regionCode, edges, vertices);
            }
        }

    } catch (const exception& e) {
        cout << "Erro: " << e.what() << endl;
    }

    // Limpeza de memória
    for (auto& vertex : vertices) {
        delete vertex;
    }
    for (auto& edge : edges) {
        delete edge;
    }

    return 0;
}
