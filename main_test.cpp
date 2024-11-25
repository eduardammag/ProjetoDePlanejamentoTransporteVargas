#include <iostream>
#include "buildGraph.h"
#include "findStation.h"
#include "vertexAndEdge.h"

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
    generateAdjacencyList(adjacencyMatrix, adjacencyList);

    // Imprime a matriz de adjacência
    cout << "\nAdjacency Matrix (with edge IDs):" << endl;
    for (const auto& row : adjacencyMatrix) {
        for (const auto& value : row) {
            if (value != nullptr) {
                cout << "Edge ID: " << value << " ";
            } else {
                cout << "nullptr ";
            }
        }
        cout << endl;
    }

    // Imprime a lista de adjacência
    cout << "\nAdjacency List (with tuples (destination vertex, edge ID)):" << endl;
    for (size_t i = 0; i < adjacencyList.size(); ++i) {
        cout << "Vertex " << i << ": ";
        for (const auto& adj : adjacencyList[i]) {
            int destination = get<0>(adj);
            Edge* edge = get<1>(adj);
            cout << "(" << destination << ", Edge pointer: " << edge << ") ";
        }
        cout << endl;
    }

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

    // Teste estação
    cout << "--------------------TESTE ESTAÇÃO------------------------" << endl;

    for (size_t i = 0; i < edgesGrouped.size(); i++) { // Iterar sobre as regiões
        Vertex* optimalVertex = findOptimalVertexFast(edgesGrouped, adjacencyList);
        if (optimalVertex) {
            cout << "Região " << i << ": Vértice ótimo = " << optimalVertex->id() << endl;
        } else {
            cout << "Região " << i << ": Não foi possível determinar o vértice ótimo.\n";
        }
    }

    // Libera memória
    for (auto& vertex : vertices) {
        delete vertex;
    }
    for (auto& edge : edges) {
        delete edge;
    }

    return 0;
}
