
#ifndef FIND_STATION_H
#define FIND_STATION_H

#include "vertexAndEdge.h"
#include "buildGraph.h"
#include <queue>
#include <climits>
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <tuple>


using namespace std;


// Implementação da função cptDijkstraFast
void cptDijkstraFast(Vertex* v0, vector<int>& parent, vector<int>& distance, const    vector<vector<tuple<int, Edge*>>> adjacencyList);


// Implementação da função findOptimalVertexFast
Vertex* findOptimalVertexFast(const vector<vector<Edge*>>& regionVertices, const vector<vector<tuple<int, Edge*>>>& adjacencyList);



#endif // FIND_STATION_H