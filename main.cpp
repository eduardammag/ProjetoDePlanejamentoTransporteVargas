#include <iostream>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"
#include "steinerTree.h" 
using namespace std;

// int main() {
//     string jsonFilePath = "city_graph.json";

//     vector<Vertex*> vertices;
//     vector<Edge*> edges;

//     try {
//         parseJsonFile(jsonFilePath, vertices, edges);
//     } catch (const exception& e) {
//         cerr << "Error: " << e.what() << endl;
//         return 1;
//     }

//     vector<vector<Edge*>> adjacencyMatrix;
//     vector<vector<tuple<int, Edge*>>> adjacencyList;

//     generateAdjacencyMatrix(vertices, edges, adjacencyMatrix);
//     generateAdjacencyList(adjacencyMatrix, adjacencyList);
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
    generateAdjacencyList(adjacencyMatrix, adjacencyList);

    // // Imprime a matriz de adjacência
    // cout << "\nAdjacency Matrix (with edge IDs):" << endl;
    // for (const auto& row : adjacencyMatrix) {
    //     for (const auto& value : row) {
    //         if (value != nullptr) {
    //             cout << "Edge ID: " << value << " ";
    //         } else {
    //             cout << "nullptr ";
    //         }
    //     }
    //     cout << endl;
    // }

    // // Imprime a lista de adjacência
    // cout << "\nAdjacency List (with tuples (destination vertex, edge ID)):" << endl;
    // for (size_t i = 0; i < adjacencyList.size(); ++i) {
    //     cout << "Vertex " << i << ": ";
    //     for (const auto& adj : adjacencyList[i]) {
    //         int destination = get<0>(adj);
    //         Edge* edge = get<1>(adj);
    //         cout << "(" << destination << ", Edge pointer: " << edge << ") ";
    //     }
    //     cout << endl;
    // }

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
    
    ///////////////////////////////////////////////////////////////////////////////
    // Agrupar arestas por CEP
    // vector<vector<Edge*>> edgesGrouped = groupEdgesByCepVector(edges);

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
        } else {
            cout << "Região " << i << ": Não foi possível determinar o vértice ótimo.\n";
        }
    }

    // Libera memória
    for (auto& vertex : vertices) delete vertex;
    for (auto& edge : edges) delete edge;


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

    generateAdjacencyMatrix(vertices, edges, adjacencyMatrix);
    generateAdjacencyList(adjacencyMatrix, adjacencyList);

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
        } else {
            cout << "Região " << i << ": Não foi possível determinar o vértice ótimo.\n";
        }
    }

    // Adicionando lógica para a árvore de Steiner
    vector<int> terminals;  // Lista de terminais (estações de metrô)
    terminals.push_back(0); // Exemplo: Adicionar estação 0
    terminals.push_back(1); // Exemplo: Adicionar estação 1
    // Adicione os IDs dos terminais conforme necessário

    // Chama a função de cálculo da árvore de Steiner
    vector<Edge*> steinerEdges = steinerTree(vertices, edges, terminals);

    // Imprime as arestas da árvore de Steiner
    cout << "\nÁrvore de Steiner:" << endl;
    for (const auto& edge : steinerEdges) {
        cout << "Aresta de " << edge->v1 << " a " << edge->v2 << " com distância " << edge->distance << endl;
    }

    // Libera memória
    for (auto& vertex : vertices) delete vertex;
    for (auto& edge : edges) delete edge;

    return 0;
}
