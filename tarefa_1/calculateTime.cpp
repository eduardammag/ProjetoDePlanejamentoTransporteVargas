#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <chrono>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"

using namespace std;
using namespace std::chrono;

void testPerformance() {
    vector<int> gridSizes = {10, 20, 30, 40}; // Tamanhos da grid
    string filePrefix = "city_graph_";           // Prefixo do nome dos arquivos
    string fileSuffix = ".json";                 // Sufixo do nome dos arquivos

    // Abre o arquivo para salvar os tempos de execução
    ofstream outputFile("execution_times.txt");
    if (!outputFile.is_open()) {
        cerr << "Erro ao abrir o arquivo para salvar os tempos de execução!" << endl;
        return;
    }

    for (int size : gridSizes) {
        string fileName = filePrefix + to_string(size) + "x" + to_string(size) + fileSuffix;

        // Verifica se o arquivo existe antes de processar
        ifstream testFile(fileName);
        if (!testFile.is_open()) {
            cerr << "Erro ao abrir o arquivo JSON: " << fileName << endl;
            outputFile << "Erro ao abrir o arquivo JSON: " << fileName << endl;
            continue;
        }
        testFile.close();

        cout << "Testando arquivo: " << fileName << endl;
        // outputFile << "Testando arquivo: " << fileName << endl;

        vector<Vertex*> vertices;
        vector<Edge*> edges;

        try {
            parseJsonFile(fileName, vertices, edges);
        } catch (const exception& e) {
            cerr << "Erro ao processar o arquivo " << fileName << ": " << e.what() << endl;
            // outputFile << "Erro ao processar o arquivo " << fileName << ": " << e.what() << endl;
            continue;
        }

        // Gera a lista de adjacência
        vector<vector<tuple<int, Edge*>>> adjacencyList;
        generateAdjacencyList(vertices, edges, adjacencyList);

        unordered_map<int, vector<Edge*>> edgesByCepMap;

        // Agrupa arestas
        for (const auto& edge : edges) {
            edgesByCepMap[edge->id_zipCode()].push_back(edge);
        }

        // Converte unordered_map para vector<vector<Edge*>>
        vector<vector<Edge*>> edgesGrouped;
        for (const auto& [key, edgeList] : edgesByCepMap) {
            edgesGrouped.push_back(edgeList);
        }

        // Medir tempo de execução
        auto startTime = high_resolution_clock::now();

        vector<Vertex*> optimalVertices;
        for (size_t i = 0; i < edgesGrouped.size(); i++) {
            const vector<Edge*>& regionEdges = edgesGrouped[i];
            vector<vector<Edge*>> currentRegion = {regionEdges};
            
            cout << "Procurando vértice ótimo para a região " << i + 1 << "..." << endl;
            // outputFile << "Procurando vértice ótimo para a região " << i + 1 << "..." << endl;

            Vertex* optimalVertex = findOptimalVertexFast(currentRegion, adjacencyList);

            if (optimalVertex) {
                cout << "Vértice ótimo encontrado para a região " << i + 1 
                     << ": ID = " << optimalVertex->id() << endl;
                // outputFile << "Vértice ótimo encontrado para a região " << i + 1 
                //           << ": ID = " << optimalVertex->id() << endl;
                optimalVertices.push_back(optimalVertex);
            } else {
                cout << "Nenhum vértice ótimo encontrado para a região " << i + 1 << "." << endl;
                // outputFile << "Nenhum vértice ótimo encontrado para a região " << i + 1 << "." << endl;
            }
        }

        auto endTime = high_resolution_clock::now();
        duration<double> elapsedTime = endTime - startTime;

        cout << "Tempo de execução para " << fileName << ": " 
             << elapsedTime.count() << " segundos." << endl;
        outputFile << "Tempo de execução para " << fileName << ": " 
                   << elapsedTime.count() << " segundos." << endl;

        // Limpeza de memória
        for (Vertex* vertex : vertices) delete vertex;
        for (Edge* edge : edges) delete edge;
    }

    // Fecha o arquivo de saída
    outputFile.close();
}

int main() {
    testPerformance();
    return 0;
}
