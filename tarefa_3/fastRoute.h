#ifndef FASTROUTE_H
#define FASTROUTE_H

#include <vector>
#include <algorithm>
#include "vertexAndEdge.h" // Inclui a definição de Vertex e Edge.

using namespace std;

// Função para encontrar a aresta correspondente a um número de imóvel em uma rua.
Edge* findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges);

#endif // FASTROUTE_H
