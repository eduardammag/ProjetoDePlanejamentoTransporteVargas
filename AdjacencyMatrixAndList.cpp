class Graph {
public:
    vector<Vertex*> vertices;
    vector<Edge*> edges;
    vector<list<tuple<int, Edge*>>> adjacencyList; // Lista de adjacência
    vector<vector<Edge*>> adjacencyMatrix;          // Matriz de adjacência

    // Adiciona um vértice ao grafo
    void addVertex(bool isMetroStation, int id) 
    {
        vertices.push_back(new Vertex(isMetroStation, id));
    }

    // Adiciona uma aresta ao grafo
    void addEdge(int vertex1Id, int vertex2Id, int distance, float trafficRate) 
    {
        if (vertex1Id == vertex2Id)
        {
            cout << "Erro: Laços não são permitidos.\n";
            return;
        }
        Vertex* vertex1 = nullptr;
        Vertex* vertex2 = nullptr;

        for (auto v : vertices) 
        {
            if (v->id() == vertex1Id) vertex1 = v;
            if (v->id() == vertex2Id) vertex2 = v;
        }

        if (vertex1 && vertex2) 
        {
            Edge* edge = new Edge(distance, vertex1, vertex2, trafficRate);
            edges.push_back(edge);
        }
    }

    // Gera a lista de adjacência
    void generateAdjacencyList() 
    {
        adjacencyList.clear();
        adjacencyList.resize(vertices.size());

        for (auto e : edges) 
        {
            int vertex1Id = e->vertex1()->id();
            int vertex2Id = e->vertex2()->id();

            adjacencyList[vertex1Id].emplace_back(vertex2Id, e);
            adjacencyList[vertex2Id].emplace_back(vertex1Id, e); // Grafo não direcionado
        }
    }

    // Gera a matriz de adjacência
    void generateAdjacencyMatrix() 
    {
        adjacencyMatrix.clear();
        adjacencyMatrix.resize(vertices.size(), vector<Edge*>(vertices.size(), nullptr));

        for (auto e : edges) 
        {
            int vertex1Id = e->vertex1()->id();
            int vertex2Id = e->vertex2()->id();

            adjacencyMatrix[vertex1Id][vertex2Id] = e;
            adjacencyMatrix[vertex2Id][vertex1Id] = e; // Grafo não direcionado
        }
    }

    // Imprime a lista de adjacência
    void printAdjacencyList() 
    {
        cout << "Lista de Adjacência:\n";
        for (size_t i = 0; i < adjacencyList.size(); ++i) 
        {
            cout << "Vértice " << i << ":";
            for (auto [neighborId, edge] : adjacencyList[i]) 
            {
                cout << " -> (" << neighborId << ", E("
                << edge->vertex1()->id() << ", "
                << edge->vertex2()->id() << ", Distância: " 
                << edge->distance() << "))";
            }
            cout << "\n";
        }
    }

    // Imprime a matriz de adjacência
    void printAdjacencyMatrix() 
    {
        cout << "Matriz de Adjacência:\n";
        for (size_t i = 0; i < adjacencyMatrix.size(); ++i) 
        {
            for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) 
            {
                if (adjacencyMatrix[i][j]) {
                    cout << "E(" << adjacencyMatrix[i][j]->vertex1()->id() << ", "
                         << adjacencyMatrix[i][j]->vertex2()->id() << ", Distância: "
                         << adjacencyMatrix[i][j]->distance() << ") ";
                } else {
                    cout << "0 ";
                }
            }
            cout << "\n";
        }
    }
};
