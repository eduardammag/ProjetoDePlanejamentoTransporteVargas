#ifndef FASTROUTE_H
#define FASTROUTE_H

#include <vector>
#include <algorithm>
#include "vertexAndEdge.h" // Inclui a definição de Vertex e Edge.
#include "fastRoute.h"

using namespace std;

// Função para encontrar a aresta correspondente a um número de imóvel em uma rua.
Edge* findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges);
pair<vector<Edge*>, int> dijkstraFoot(Vertex* start,  Vertex* destination, const vector<vector<tuple<int, Edge*>>> adjacencyList);


// Declaração da função dijkstraTaxi
tuple<float, float, vector<Edge*>> dijkstraTaxi(
    const vector<vector<tuple<int, Edge*>>>& directedAdj, // Lista de adjacência representando o grafo dirigido
    int start, // Vértice inicial
    int destination // Vértice de destino
);
#endif // FAST_ROUTE_H