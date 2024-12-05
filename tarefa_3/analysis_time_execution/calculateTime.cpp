#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <chrono>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"
#include "connectingMetro.h"
#include "fastRoute.h"

using namespace std;
using namespace std::chrono;

void testPerformance() {
    vector<int> gridSizes = {10, 20, 30}; // Tamanhos da grid
    string filePrefix = "city_graph_";       // Prefixo do nome dos arquivos
    string fileSuffix = ".json";             // Sufixo do nome dos arquivos

    ofstream outputFile("execution_time_fastestRout.txt");
    if (!outputFile.is_open()) {
        cerr << "Erro ao abrir o arquivo para salvar os tempos de execução!" << endl;
        return;
    }

    for (int size : gridSizes) {
        string fileName = filePrefix + to_string(size) + "x" + to_string(size) + fileSuffix;

        ifstream testFile(fileName);
        if (!testFile.is_open()) {
            cerr << "Erro ao abrir o arquivo JSON: " << fileName << endl;
            outputFile << "Erro ao abrir o arquivo JSON: " << fileName << endl;
            continue;
        }
        testFile.close();

        cout << "Testando arquivo: " << fileName << endl;

        vector<Vertex*> vertices;
        vector<Edge*> edges;

        try {
            parseJsonFile(fileName, vertices, edges);
        } catch (const exception& e) {
            cerr << "Erro ao processar o arquivo " << fileName << ": " << e.what() << endl;
            outputFile << "Erro ao processar o arquivo " << fileName << ": " << e.what() << endl;
            continue;
        }

        vector<vector<tuple<int, Edge*>>> adjacencyList;
        generateAdjacencyList(vertices, edges, adjacencyList);

        unordered_map<int, vector<Edge*>> edgesByCepMap;
        for (const auto& edge : edges) {
            edgesByCepMap[edge->id_zipCode()].push_back(edge);
        }

        vector<vector<Edge*>> edgesGrouped;
        for (const auto& [key, edgeList] : edgesByCepMap) {
            edgesGrouped.push_back(edgeList);
        }

        vector<Vertex*> optimalVertices;
        for (size_t i = 0; i < edgesGrouped.size(); i++) {
            const vector<Edge*>& regionEdges = edgesGrouped[i];
            vector<vector<Edge*>> currentRegion = {regionEdges};
            Vertex* optimalVertex = findOptimalVertexFast(currentRegion, adjacencyList);

            if (optimalVertex) {
                optimalVertices.push_back(optimalVertex);
            }
        }

        // Simula o cálculo de fastestRoute
        auto startTime = high_resolution_clock::now();

        // Configurar exemplo de origem e destino
        if (vertices.size() > 1) {
            Edge* startEdge = edges.front();
            Edge* destinationEdge = edges.back();
            Vertex* startVertex = startEdge->vertex1();
            Vertex* destinationVertex = destinationEdge->vertex2();

            // Matriz de adjacência orientada
            vector<vector<tuple<int, Edge*>>> directedAdjacencyList = adjacencyList;
            convertToDirected(directedAdjacencyList);

            // Executar a função fastestRoute
            tuple<vector<pair<Edge*, string>>, int, float> result = fastestRoute(
                startVertex, destinationVertex, startEdge, destinationEdge,
                0,  // Cep inicial
                0,  // Cep destino
                adjacencyList, directedAdjacencyList, adjacencyList);

            // Apenas acessar o resultado para verificar o desempenho
            auto& path = get<0>(result);
            for (const auto& [edge, locomotion] : path) {
                // Apenas itera para simular uso da função
                (void)edge;
                (void)locomotion;
            }
        }

        auto endTime = high_resolution_clock::now();
        duration<double> elapsedTime = endTime - startTime;

        cout << "Tempo de execução para fastestRoute no arquivo " << fileName
             << ": " << elapsedTime.count() << " segundos." << endl;
        outputFile << "Tempo de execução para fastestRoute no arquivo " << fileName
                   << ": " << elapsedTime.count() << " segundos." << endl;

        // Limpeza de memória
        for (Vertex* vertex : vertices) delete vertex;
        for (Edge* edge : edges) delete edge;
    }

    outputFile.close();
}

int main() {
    testPerformance();
    return 0;
}
