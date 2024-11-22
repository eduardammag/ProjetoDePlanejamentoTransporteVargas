#include "graph.h"
#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

//Adiciona vértice ao grafo, o qual possui duas informações: seu id e se é estação de metrô
void Graph::addVertex(bool isMetroStation, int id) {
    vertices.push_back(new Vertex(isMetroStation, id));
}

//Adiciona aresta ao grafo, conectando dois vértices v1 e v2. Cada aresta tem distância e taxa de tráfego
void Graph::addEdge(int vertex1Id, int vertex2Id, int distance, float trafficRate) 
{
    //Em nossa cidade Vargas não tem "laços de ruas"
    if (vertex1Id == vertex2Id) {
        std::cout << "Erro: Laços não são permitidos.\n";
        return;
    }
    Vertex* vertex1 = nullptr;
    Vertex* vertex2 = nullptr;

    for (auto v : vertices) {
        if (v->id() == vertex1Id) vertex1 = v;
        if (v->id() == vertex2Id) vertex2 = v;
    }

    if (vertex1 && vertex2) {
        Edge* edge = new Edge(distance, vertex1, vertex2, trafficRate);
        edges.push_back(edge);
    }
}

//Gera a lista de adjacência do grafo que será um vetor com listas de tuplas contendo o id do vetor destino e o ponteiro pra aresta correspondente
void Graph::generateAdjacencyList() {
    adjacencyList.clear();
    adjacencyList.resize(vertices.size()); //O tamanho é igual ao número de vértices porque cada elemento está associado a um vértice do grafo

    for (auto e : edges) {
        int vertex1Id = e->vertex1()->id();
        int vertex2Id = e->vertex2()->id();

        //adiciona as tuplas em cada lista
        adjacencyList[vertex1Id].emplace_back(vertex2Id, e);
        adjacencyList[vertex2Id].emplace_back(vertex1Id, e);
    }
}

//Constrói a matriz de adjacência, a qual é composta por ponteiros nulo ou de uma aresta correspondente a cada posição na matriz
void Graph::generateAdjacencyMatrix() {
    adjacencyMatrix.clear();
    adjacencyMatrix.resize(vertices.size(), std::vector<Edge*>(vertices.size(), nullptr));

    for (auto e : edges) {
        int vertex1Id = e->vertex1()->id();
        int vertex2Id = e->vertex2()->id();

        adjacencyMatrix[vertex1Id][vertex2Id] = e;
        adjacencyMatrix[vertex2Id][vertex1Id] = e;
    }
}

//Exibe a lista de adjacência de forma intuitiva
void Graph::printAdjacencyList() {
    std::cout << "Lista de Adjacência:\n";
    for (size_t i = 0; i < adjacencyList.size(); ++i) {
        std::cout << "Vértice " << i << ":";
        for (auto [neighborId, edge] : adjacencyList[i]) {
            std::cout << " -> (" << neighborId << ", E("
                      << edge->vertex1()->id() << ", "
                      << edge->vertex2()->id() << ", Distância: "
                      << edge->distance() << "))";
        }
        std::cout << "\n";
    }
}

//Exibe a matriz de adjacência
void Graph::printAdjacencyMatrix() {
    std::cout << "Matriz de Adjacência:\n";
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j]) {
                std::cout << "E(" << adjacencyMatrix[i][j]->vertex1()->id() << ", "
                          << adjacencyMatrix[i][j]->vertex2()->id() << ", Distância: "
                          << adjacencyMatrix[i][j]->distance() << ") ";
            } else {
                std::cout << "0 ";
            }
        }
        std::cout << "\n";
    }
}

// Função auxiliar
// Tira o char do id do Vértice
auto extract_vertex_number(const string& vertex)
{
    return stoi(vertex.substr(1));
}

// Constroi o grafo com uma matriz de adj
Graph Graph::buildGraph(const string& filename) 
{
    ifstream file(filename); 
    string line;
    Graph g;
    // Seed aleatória
    srand(time(0));

    if (!file.is_open()) 
    {
        cerr << "Erro ao abrir o arquivo!" << endl;
        throw runtime_error("Erro ao abrir o arquivo");
    }


    // Adiciona ao grafo os vértices
    while (getline(file, line)) {
        stringstream ss(line);
        string vertex;
        ss >> vertex;
        
        // Lê o primeiro vértice de cada linha e adiciona
        int vertex_number = extract_vertex_number(vertex); 
        g.addVertex(false, vertex_number);
    }
    
    // Para ler de novo o arquivo 
    file.clear();  
    file.seekg(0, ios::beg);  
    

    // Adicionando as arestas
    while (getline(file, line)) {
        stringstream ss(line);
        string vertex;
        ss >> vertex;

        int vertex_number = extract_vertex_number(vertex);

        // Lê a lista dos adjacentes na linha corrente
        string adjacent;
        while (ss >> adjacent) {
            // Id dos adjacentes
            int adjacent_number = extract_vertex_number(adjacent);

            // Adiciona aresta entre corrente e o adjacente
            float distance = rand() % 100 + 1;
            float trafficRate = rand() % 10 + 1;
            g.addEdge(vertex_number, adjacent_number, distance, trafficRate);
        }
    }

    file.close();
    return g;  // Retorna o grafo construído
}

