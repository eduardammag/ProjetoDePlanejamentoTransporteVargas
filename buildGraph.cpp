#include "buildGraph.h"
#include "vertexAndEdge.h"
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <queue>

using namespace std;

void parseJsonFile(const string& filePath, vector<Vertex*>& vertices, vector<Edge*>& edges) {
    ifstream file(filePath);
    if (!file.is_open()) {
        throw runtime_error("Erro ao abrir o arquivo JSON: " + filePath);
    }

    string line;
    bool parsingVertices = false, parsingEdges = false;

    while (getline(file, line)) {
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line.find("\"vertex\":") != string::npos) {
            parsingVertices = true;
            parsingEdges = false;
            continue;
        }
        if (line.find("\"edges\":") != string::npos) {
            parsingVertices = false;
            parsingEdges = true;
            continue;
        }

        if (line.find("]") != string::npos) {
            parsingVertices = false;
            parsingEdges = false;
            continue;
        }

        if (parsingVertices && line.find("{") != string::npos) {
            int id = -1;
            bool isMetro = false;

            while (getline(file, line) && line.find("}") == string::npos) {
                line.erase(0, line.find_first_not_of(" \t"));
                line.erase(line.find_last_not_of(" \t") + 1);

                if (line.find("\"id_vertex\":") != string::npos) {
                    id = stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"isMetroStation\":") != string::npos) {
                    isMetro = line.find("true") != string::npos;
                }
            }
            vertices.push_back(new Vertex(isMetro, id));
        }

        if (parsingEdges && line.find("{") != string::npos) {
            int idEdge = -1, v1 = -1, v2 = -1, distance = 0;
            float trafficRate = 0.0f; int id_zipCode = 0;

            while (getline(file, line) && line.find("}") == string::npos) {
                line.erase(0, line.find_first_not_of(" \t"));
                line.erase(line.find_last_not_of(" \t") + 1);

                if (line.find("\"id_edge\":") != string::npos) {
                    idEdge = stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"v1\":") != string::npos) {
                    v1 = stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"v2\":") != string::npos) {
                    v2 = stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"distance\":") != string::npos) {
                    distance = stoi(line.substr(line.find(":") + 1));
                } else if (line.find("\"traficRate\":") != string::npos) {
                    trafficRate = stof(line.substr(line.find(":") + 1));
                } else if (line.find("\"cep\":") != string::npos) {
                    id_zipCode = stof(line.substr(line.find(":") + 1));
                }
            }

            Vertex* vertex1 = nullptr;
            Vertex* vertex2 = nullptr;

            for (const auto& vertex : vertices) {
                if (vertex->id() == v1) vertex1 = vertex;
                if (vertex->id() == v2) vertex2 = vertex;
            }

            if (vertex1 && vertex2) {
                Edge* edge = new Edge(distance, vertex1, vertex2, trafficRate, idEdge, id_zipCode);
                edges.push_back(edge);
            }
        }
    }

    file.close();
}

void generateAdjacencyMatrix(const vector<Vertex*>& vertices, const vector<Edge*>& edges, vector<vector<Edge*>>& adjacencyMatrix) {
    int n = vertices.size();
    adjacencyMatrix.resize(n, vector<Edge*>(n, nullptr));  //Inicializa com nullptr

    // Preenche a matriz de adjacência com ponteiros para as arestas
    for (const auto& edge : edges) {
        int v1_id = edge->vertex1()->id();
        int v2_id = edge->vertex2()->id();
        
        //Grafo não direcionado, então preenche ambos os lugares
        adjacencyMatrix[v1_id][v2_id] = edge;
        adjacencyMatrix[v2_id][v1_id] = edge;
    }
}

void generateAdjacencyList(const vector<vector<Edge*>>& adjacencyMatrix, vector<vector<tuple<int, Edge*>>>& adjacencyList) {
    int n = adjacencyMatrix.size();
    adjacencyList.resize(n);  //Inicializa a lista de adjacência

    //Preenche a lista de adjacência com tuplas (ID do vértice de destino, ponteiro para a aresta)
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (adjacencyMatrix[i][j] != nullptr) {  //Verifica se existe uma aresta
                adjacencyList[i].push_back(make_tuple(j, adjacencyMatrix[i][j]));
            }
        }
    }
}

// Função para imprimir o grafo (exibe as arestas de cada vértice)
void printGraph(const std::vector<Vertex*>& vertices, const std::vector<std::vector<std::tuple<int, Edge*>>>& adjacencyList) {
    for (int i = 0; i < vertices.size(); ++i) {
        std::cout << "Vértice " << vertices[i]->id() << ": ";  // Imprime o ID do vértice

        // Acessa as arestas do vértice i
        for (const auto& edge : adjacencyList[i]) {
            int neighbor = std::get<0>(edge);  // O índice do vértice vizinho
            Edge* e = std::get<1>(edge);  // O ponteiro para a aresta

            // Imprime a aresta com o ID dos vértices conectados e o peso da aresta
            std::cout << "(" << vertices[neighbor]->id() << ", " << e->distance() << ") ";
        }
        std::cout << std::endl;
    }
}

vector<vector<Edge*>> groupEdgesByCepVector(const vector<Edge*>& edges) {
    //Mapeia os `ceps` para listas de arestas
    unordered_map<int, vector<Edge*>> cepToEdgesMap;

    for (const auto& edge : edges) {
        int cep = edge->id_zipCode();
        cepToEdgesMap[cep].push_back(edge);
    }

    //Converte o mapa em um vetor de listas, preservando os grupos por `cep`
    vector<vector<Edge*>> edgesByCep;
    for (const auto& [cep, edgeList] : cepToEdgesMap) {
        edgesByCep.push_back(edgeList);
    }

    return edgesByCep;
}

void printEdgesGroupedByCepVector(const unordered_map<int, vector<Edge*>>& cepToEdgesMap) {
    for (const auto& [cep, edgeList] : cepToEdgesMap) {
        cout << "CEP: " << cep << " -> Arestas: ";
        for (const auto& edge : edgeList) {
            cout << edge->idEdge() << " "; //Imprime o ID da aresta
        }
        cout << endl;
    }
}

// Acessar o vértice na outra ponta
Vertex* otherVertex(Edge* edge, int currentId) 
{
    return (edge->vertex1()->id() == currentId) ? edge->vertex2(): edge->vertex1();
}

// Percorre por bfs adicionando direção ao grafo
void bfs(Vertex* start,
             const vector<vector<tuple<int, Edge*>>>& adj,
             vector<vector<tuple<int, Edge*>>>& directedAdj,
             vector<int>& degreeOut,
             vector<int>& degreeIn) 
    {
        // Armazenar acessos a vertex e edges
        unordered_set<int> visited;
        unordered_set<int> processedEdges;
        
        // Fila para a BFS
        queue<Vertex*> q;
        q.push(start);
        visited.insert(start->id());
        
        while (!q.empty()) 
        {
            // Primeiro vertex na fila
            Vertex* current = q.front();
            q.pop();
            
            // acessa os adjacentes do vértice atual 
            for (const auto& [vertexId, edge] : adj[current->id()]) 
            {
                // se a aresta já foi visitada
                if (processedEdges.count(edge->idEdge())) continue;

                processedEdges.insert(edge->idEdge());
                Vertex* nextVertex = otherVertex(edge, current->id());

                // se grau entrada 0, adiciona uma aresta para dentro
                if (degreeIn[nextVertex->id()] < 1) 
                {
                    directedAdj[nextVertex->id()].emplace_back(nextVertex->id(), edge);
                    // atualiza os graus
                    degreeOut[current->id()]++;
                    degreeIn[nextVertex->id()]++;
                    
                    // Rua de mão dupla, aleatóriamente escolhida
                    if (rand() % 10 < 2) 
                    {   
                        // Criando a nova aresta com sentido oposto
                        int newId =  edge->idEdge() + 1000;
                        Edge* newEdge = new Edge(edge->distance(), edge->vertex1(), edge->vertex2(), edge->trafficRate(), newId, edge->id_zipCode());
                        directedAdj[current->id()].emplace_back(current->id(), newEdge);
                        degreeOut[current->id()]++;
                        degreeIn[nextVertex->id()]++;
                    }
                } 
                // se grau de saída 0, adiciona uma aresta para fora
                else if (degreeOut[current->id()] < 1) 
                {
                    directedAdj[current->id()].emplace_back(current->id(), edge);
                    degreeOut[current->id()]++;
                    degreeIn[nextVertex->id()]++;
                    
                    // Rua de mão dupla, aleatóriamente escolhida
                    if (rand() % 10 < 2) 
                    { // Cria nova aresta com sentido oposto
                        Edge* newEdge = new Edge(edge->distance(), edge->vertex1(), edge->vertex2(), edge->trafficRate(), edge->idEdge()+1000, edge->id_zipCode());
                        directedAdj[nextVertex->id()].emplace_back(nextVertex->id(), newEdge);
                        degreeOut[nextVertex->id()]++;
                        degreeIn[current->id()]++;
                    }
                    
                } 
                else 
                {
                    // se já tiver grau de entrada e saída a direção é aleatória
                    if (rand() % 2 == 0) 
                    {
                        // Para fora
                        directedAdj[current->id()].emplace_back(current->id(), edge); 
                        degreeOut[current->id()]++;
                        degreeIn[nextVertex->id()]++;
                    } 
                    else 
                    {
                        // Para dentro
                        directedAdj[nextVertex->id()].emplace_back(nextVertex->id(), edge); 
                        degreeOut[nextVertex->id()]++;
                        degreeIn[current->id()]++;
                    }
                }


                // Se o próximo vértice ainda não foi visitado, adiciona na fila
                if (!visited.count(nextVertex->id())) 
                {
                    visited.insert(nextVertex->id());
                    q.push(nextVertex);
                }
            }
        }
    }


vector<vector<tuple<int, Edge*>>> convertToDirected(const vector<vector<tuple<int, Edge*>>>& adj) 
{   

    int n = adj.size();
    // Inicializa estruturas auxiliares
    vector<vector<tuple<int, Edge*>>> directedAdj(n);
    unordered_set<int> visited;
    unordered_set<int> processedEdges;

    // Inicializa os vetores de graus
    vector<int> degreeOut(n, 0);
    vector<int> degreeIn(n, 0);

    // Escolhe um vértice inicial 
    Vertex* start = std::get<1>(adj[0][0])->vertex1();
   // Realiza a BFS
    bfs(start, adj, directedAdj, degreeOut, degreeIn);

    return directedAdj;
}

void printDirectedAdjacencyList(const vector<vector<tuple<int, Edge*>>>& directedAdj) 
{
        cout << "Lista de Adjacência do Grafo Direcionado:" << endl;
        for (int i = 0; i < directedAdj.size(); ++i) 
        {
            cout << "Vértice " << i << ": ";
            for (const auto& [vertexId, edge] : directedAdj[i]) 
            {
                cout << " Edge ID: " << edge->idEdge() << ", Conecta: " << otherVertex(edge,vertexId)->id();
            }
            cout << endl;
        }
}
