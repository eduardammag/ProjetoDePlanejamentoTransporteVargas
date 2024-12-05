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

using namespace std;
using namespace std::chrono;

void testPerformance() {
    vector<int> gridSizes = { 40}; // Tamanhos da grid
    string filePrefix = "city_graph_";       // Prefixo do nome dos arquivos
    string fileSuffix = ".json";             // Sufixo do nome dos arquivos

    ofstream outputFile("execution_times_connect_metro.txt");
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

        // Medir o tempo de execução da função conect_metro
        auto startTime = high_resolution_clock::now();

        vector<vector<tuple<int, Edge*>>> mstadj;
        vector<vector<Edge*>> detailedPaths;
        mstadj = conect_metro(vertices, adjacencyList, optimalVertices, detailedPaths);

        auto endTime = high_resolution_clock::now();
        duration<double> elapsedTime = endTime - startTime;

        cout << "processou" << endl;
        cout << "Tempo de execução para conect_metro no arquivo " << fileName 
             << ": " << elapsedTime.count() << " segundos." << endl;
        outputFile << "Tempo de execução para conect_metro no arquivo " << fileName 
                   << ": " << elapsedTime.count() << " segundos." << endl;

        // printDirectedAdjacencyList(mstadj);

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
