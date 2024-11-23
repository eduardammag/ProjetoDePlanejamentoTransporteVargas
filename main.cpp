#include <iostream>
#include "vertexAndEdge.h"
#include "graph.h"
#include <random>
#include <cmath>
#include <ctime>

using namespace std;

int main() 
{
    srand(time(0));  //Inicializa a semente aleatória para gerar valores diferentes a cada execução

    cout << "TESTE DAS CLASSES VERTEX E EDGE" << endl;
    cout << "-----------------" << endl;

    //Criar vértices
    Vertex v1(true, 1);  
    Vertex v2(false, 2);  

    Edge e(100, &v1, &v2, 1.5, 10);  

    cout << "Vértice 1 ID: " << e.vertex1()->id() << " | É estação de metrô? " 
         << (e.vertex1()->isMetroStation() ? "Sim" : "Não") << endl;

    cout << "Vértice 2 ID: " << e.vertex2()->id() << " | É estação de metrô? " 
         << (e.vertex2()->isMetroStation() ? "Sim" : "Não") << endl;

    cout << "Distância da aresta: " << e.distance() << endl;
    cout << "CEP (4 dígitos): " << e.id_zipCode() << endl;
    cout << "Rua (8 dígitos): " << e.id_street() << endl;
    cout << "Taxa de Tráfego: " << e.trafficRate() << endl;
    cout << "Custo de Escavação: " << e.excavationCost() << endl;
    cout << "-----------------" << endl;

    //Criar grafo
    Graph g;
    const int numVertices = 100;  //100 vértices
    const int maxEdges = 5;  //Número máximo de arestas por vértice

    //Adiciona 100 vértices
    for (int i = 0; i < numVertices; ++i) 
    {
        g.addVertex(i % 2 == 0, i);  //Alternando estação de metrô e não
    }

    //Adiciona arestas aleatórias entre vértices, com restrição de uma aresta por par de vértices
    for (int i = 0; i < numVertices; ++i) 
    {
        //Cada vértice pode ter até 'maxEdges' arestas
        int edgesToAdd = rand() % maxEdges;  //Número aleatório de arestas para o vértice 'i'

        int counter = 0;
        for (int j = 0; j < edgesToAdd; ++j) 
        {
            int target = rand() % numVertices;  //Escolhe um vértice aleatório para se conectar
            if (target != i) 
            {  //Impede arestas de laço (vértice se conectando a ele mesmo)
                //Verifica se já existe uma aresta entre os dois vértices
                bool edgeExists = false;
                for (auto& e : g.edges()) 
                {  //Usando 'edges' diretamente
                    if ((e->vertex1()->id() == i && e->vertex2()->id() == target) ||
                        (e->vertex1()->id() == target && e->vertex2()->id() == i)) 
                        {
                        edgeExists = true;
                        break;
                        }
                }
                if (!edgeExists) 
                {  //Se não existe, adicione a aresta
                    float distance = rand() % 100 + 1;  //Distância aleatória entre 1 e 100
                    float trafficRate = rand() % 10 + 1;  //Taxa de tráfego aleatória entre 1 e 10
                    g.addEdge(i, target, distance, trafficRate, counter++);
                 }
            }
        }
    }

    //Imprime as representações
    g.printAdjacencyList();
    g.printAdjacencyMatrix();
    cout << "-----------------" << endl;

    return 0;
}
