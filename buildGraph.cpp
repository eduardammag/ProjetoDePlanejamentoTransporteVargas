#include "buildGraph.h"
#include "vertexAndEdge.h"
#include <fstream>
#include <stdexcept>
#include <iostream>

void parseJsonFile(const std::string& filePath, std::vector<Vertex*>& vertices, std::vector<Edge*>& edges) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Erro ao abrir o arquivo JSON: " + filePath);
    }

    std::string line;
    bool parsingVertices = false, parsingEdges = false;

    while (std::getline(file, line)) {
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line.find("\"vertex\":") != std::string::npos) {
            parsingVertices = true;
            parsingEdges = false;
            continue;
        }
        if (line.find("\"edges\":") != std::string::npos) {
            parsingVertices = false;
            parsingEdges = true;
            continue;
        }

        if (line.find("]") != std::string::npos) {
            parsingVertices = false;
            parsingEdges = false;
            continue;
        }

        if (parsingVertices && line.find("{") != std::string::npos) {
            int id = -1;
            bool isMetro = false;

            while (std::getline(file, line) && line.find("}") == std::string::npos) {
                line.erase(0, line.find_first_not_of(" \t"));
                line.erase(line.find_last_not_of(" \t") + 1);

                if (line.find("\"id_vertex\":") != std::string::npos) {
                    id = std::stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"isMetroStation\":") != std::string::npos) {
                    isMetro = line.find("true") != std::string::npos;
                }
            }
            vertices.push_back(new Vertex(isMetro, id));
        }

        if (parsingEdges && line.find("{") != std::string::npos) {
            int idEdge = -1, v1 = -1, v2 = -1, distance = 0;
            float trafficRate = 0.0f;

            while (std::getline(file, line) && line.find("}") == std::string::npos) {
                line.erase(0, line.find_first_not_of(" \t"));
                line.erase(line.find_last_not_of(" \t") + 1);

                if (line.find("\"id_edge\":") != std::string::npos) {
                    idEdge = std::stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"v1\":") != std::string::npos) {
                    v1 = std::stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"v2\":") != std::string::npos) {
                    v2 = std::stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"distance\":") != std::string::npos) {
                    distance = std::stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"traficRate\":") != std::string::npos) {
                    trafficRate = std::stof(line.substr(line.find(":") + 1));
                }
            }

            Vertex* vertex1 = nullptr;
            Vertex* vertex2 = nullptr;

            for (const auto& vertex : vertices) {
                if (vertex->id() == v1) vertex1 = vertex;
                if (vertex->id() == v2) vertex2 = vertex;
            }

            if (vertex1 && vertex2) {
                Edge* edge = new Edge(distance, vertex1, vertex2, trafficRate, idEdge);
                edges.push_back(edge);
            }
        }
    }

    file.close();
}

void generateAdjacencyMatrix(const std::vector<Vertex*>& vertices, const std::vector<Edge*>& edges, std::vector<std::vector<Edge*>>& adjacencyMatrix) {
    int n = vertices.size();
    adjacencyMatrix.resize(n, std::vector<Edge*>(n, nullptr));  // Inicializa com nullptr

    // Preenche a matriz de adjacência com ponteiros para as arestas
    for (const auto& edge : edges) {
        int v1_id = edge->vertex1()->id();
        int v2_id = edge->vertex2()->id();
        
        // Grafo não direcionado, então preenche ambos os lugares
        adjacencyMatrix[v1_id][v2_id] = edge;
        adjacencyMatrix[v2_id][v1_id] = edge;
    }
}

void generateAdjacencyList(const std::vector<std::vector<Edge*>>& adjacencyMatrix, std::vector<std::vector<std::tuple<int, Edge*>>>& adjacencyList) {
    int n = adjacencyMatrix.size();
    adjacencyList.resize(n);  // Inicializa a lista de adjacência

    // Preenche a lista de adjacência com tuplas (ID do vértice de destino, ponteiro para a aresta)
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (adjacencyMatrix[i][j] != nullptr) {  // Verifica se existe uma aresta
                adjacencyList[i].push_back(std::make_tuple(j, adjacencyMatrix[i][j]));
            }
        }
    }
}
