#ifndef FASTROUTE_H
#define FASTROUTE_H

#include <vector>
#include <algorithm>
#include "vertexAndEdge.h"
#include "fastRoute.h"

using namespace std;

// Função para encontrar a aresta correspondente a um número de imóvel em uma rua.
pair<Edge*, Vertex*> findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges);

//Função que calcula melhor caminho a pé usando Dijkstra
pair<vector<Edge*>, int> dijkstraFoot(Vertex* start,  Vertex* destination, const vector<vector<tuple<int, Edge*>>> adjacencyList);

//Função que calcula o melhor caminho de táxi usando Dijkstra
tuple<float, float, vector<Edge*>> dijkstraTaxi(
    const vector<vector<tuple<int, Edge*>>>& directedAdj, // Lista de adjacência representando o grafo dirigido
    int start, // Vértice inicial
    int destination // Vértice de destino
);

//Função para verificar se a aresta destino está na mesma região que a aresta inicial
bool isTheSameRegion(const Edge* edge1, const Edge* edge2);

//Função para verificar o melhor caminho entre dois vértices (de táxi ou a pé), considerando o orçamento
vector<Edge*> findBestPath(pair<Edge*, Vertex*> start, pair<Edge*, Vertex*> destination, 
                           const vector<vector<tuple<int, Edge*>>>& adjacencyList, 
                           const vector<vector<tuple<int, Edge*>>>& directedAdj, float budget);

#endif