#include <iostream>
#include "classes.h"
#include "graph.h"
#include <random>
#include <cmath>
using namespace std;


int main() {
    Vertex v1(true, 1);  
    Vertex v2(false, 2);  

    Edge e(100, &v1, &v2, 1.5);  

    cout << "TESTE DAS CLASSES VERTEX E EDGE" << endl;
    cout << "-----------------" << endl;

    cout << "Vértice 1 ID: " << e.vertex1()->id() << " | É estação de metrô? " 
         << (e.vertex1()->isMetroStation() ? "Sim" : "Não") << endl;

    cout << "Vértice 2 ID: " << e.vertex2()->id() << " | É estação de metrô? " 
         << (e.vertex2()->isMetroStation() ? "Sim" : "Não") << endl;

    cout << "Distância da aresta: " << e.distance() << endl;
    cout << "CEP (4 dígitos): " << e.id_zipCode() << endl;
    cout << "Rua (8 dígitos): " << e.id_street() << endl;
    cout << "Taxa de Tráfego: " << e.trafficRate() << endl;
    cout << "Custo de Escavação: " << e.excavationCost() << endl;

    // teste da construção do grafo e representação em lista e matriz de adjacência
    
    Graph g;

    // Adiciona vértices
    g.addVertex(true, 0);
    g.addVertex(false, 1);
    g.addVertex(true, 2);

    // Adiciona arestas
    g.addEdge(0, 1, 10, 1.2f);
    g.addEdge(1, 2, 15, 0.8f);
    g.addEdge(2, 2, 20, 1.0f);   // Testando Laço (não deve ser adicionado)

    // Gera as representações
    g.generateAdjacencyList();
    g.generateAdjacencyMatrix();

    // Imprime as representações
    cout << "TESTE CLASSE GRAPH" << endl;
    cout << "-----------------" << endl;
    g.printAdjacencyList();
    g.printAdjacencyMatrix();

    return 0;
}

