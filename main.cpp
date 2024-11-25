#include <iostream>
#include "vertexAndEdge.h"
#include "buildGraph.h"

int main() {
    std::string jsonFilePath = "city_graph.json"; // Path to your JSON file

    std::vector<Vertex*> vertices;
    std::vector<Edge*> edges;

    try {
        parseJsonFile(jsonFilePath, vertices, edges);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Matrices and lists
    std::vector<std::vector<Edge*>> adjacencyMatrix;
    std::vector<std::vector<std::tuple<int, Edge*>>> adjacencyList;

    // Generate adjacency matrix
    generateAdjacencyMatrix(vertices, edges, adjacencyMatrix);

    // Generate adjacency list
    generateAdjacencyList(adjacencyMatrix, adjacencyList);

    // Display the adjacency matrix
    std::cout << "\nAdjacency Matrix (with edge IDs):" << std::endl;
    for (const auto& row : adjacencyMatrix) {
        for (const auto& value : row) {
            if (value != nullptr) {
                std::cout << "Edge ID: " << value->idEdge() << " ";
            } else {
                std::cout << "nullptr ";
            }
        }
        std::cout << std::endl;
    }

    // Display the adjacency list
    std::cout << "\nAdjacency List (with tuples (destination vertex, edge ID)):" << std::endl;
    for (size_t i = 0; i < adjacencyList.size(); ++i) {
        std::cout << "Vertex " << i << ": ";
        for (const auto& adj : adjacencyList[i]) {
            int destination = std::get<0>(adj);
            Edge* edge = std::get<1>(adj);
            std::cout << "(" << destination << ", Edge ID: " << edge->idEdge() << ") ";
        }
        std::cout << std::endl;
    }

    // Clean up memory
    for (auto& vertex : vertices) {
        delete vertex;
    }
    for (auto& edge : edges) {
        delete edge;
    }

    return 0;
}
