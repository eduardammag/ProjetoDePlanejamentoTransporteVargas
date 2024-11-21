#include <iostream>
#include "classes.h"
#include <random>
#include <cmath>
using namespace std;


int main() {
    Vertex v1(true, 1);  
    Vertex v2(false, 2);  

    Edge e(100, &v1, &v2, 1.5);  

    cout << "TESTE DAS CLASSES" << endl;
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

    return 0;
}

