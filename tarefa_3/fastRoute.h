#ifndef FASTROUTE_H
#define FASTROUTE_H

#include <algorithm>
#include <string>
#include "vertexAndEdge.h"
#include "fastRoute.h"

using namespace std;

// Função para encontrar a aresta correspondente a um número de imóvel em uma rua
pair<Edge*, Vertex*> findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges);

//Função que calcula melhor caminho a pé usando Dijkstra
pair<vector<Edge*>, int> dijkstraFoot(Vertex* start,  Vertex* destination, const vector<vector<tuple<int, Edge*>>> adjacencyList);

//Função que calcula o melhor caminho de táxi usando Dijkstra
tuple<float, float, vector<Edge*>> dijkstraTaxi(const vector<vector<tuple<int, Edge*>>>& directedAdj, 
                                    int start,
                                    int destination);

//Função para verificar se a aresta destino está na mesma região que a aresta inicial
bool isTheSameRegion(const Edge* edge1, const Edge* edge2);

//Função para verificar o melhor caminho entre dois vértices (de táxi ou a pé), considerando o orçamento
tuple<vector<Edge*>, float, int, string> findBestPath(Vertex* start, Vertex* destination, 
                           const vector<vector<tuple<int, Edge*>>>& adjacencyList, 
                           const vector<vector<tuple<int, Edge*>>>& directedAdj, float budget);
                           

tuple<vector<pair<int, Edge*>>, vector<int>, vector<int>> findPathBetweenStation(
    const vector<vector<tuple<int, Edge*>>>& mstadj,
    int region1CEP, int region2CEP );

// Função principal que calcula a rota mais rápida
tuple<vector<pair<Edge*, string>>, int, float> fastestRoute(Vertex* startVertex, Vertex* destinationVertex, 
                                    Edge* startEdge, Edge* destEdge, int hour, float budget, 
                                    const vector<vector<tuple<int, Edge*>>>&adjList, 
                                    const vector<vector<tuple<int, Edge*>>>&dirAdjList, 
                                    const vector<vector<tuple<int, Edge*>>>& mstadj);

#endif