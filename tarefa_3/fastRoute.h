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

#endif // FAST_ROUTE_H