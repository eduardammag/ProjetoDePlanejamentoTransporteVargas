#ifndef BUILDGRAPH_H
#define BUILDGRAPH_H

#include <vector>
#include <string>
#include "vertexAndEdge.h"

using namespace std;

//Função para fazer o parse do arquivo JSON e preencher os vetores de vértices e arestas
void parseJsonFile(const string& filePath, vector<Vertex*>& vertices, vector<Edge*>& edges);

//Função para gerar a matriz de adjacência
void generateAdjacencyMatrix(const vector<Vertex*>& vertices, const vector<Edge*>& edges, vector<vector<Edge*>>& adjacencyMatrix);

//Função para converter a matriz de adjacência para lista de adjacência
void generateAdjacencyList(const vector<vector<Edge*>>& adjacencyMatrix, vector<vector<tuple<int, Edge*>>>& adjacencyList);

//Função para gerar um vetor de listas de arestas agrupadas por cep
vector<vector<Edge*>> groupEdgesByCepVector(const vector<Edge*>& edges);

//Função auxiliar para imprimir as listas de arestas para cada cep
void printEdgesGroupedByCepVector(const unordered_map<int, vector<Edge*>>& cepToEdgesMap);

Vertex* otherVertex(Edge* edge, Vertex* current);
         
void bfs(Vertex* start,
             const vector<vector<tuple<int, Edge*>>>& adj,
             vector<vector<tuple<int, Edge*>>>& directedAdj,
             vector<int>& degreeOut,
             vector<int>& degreeIn);

vector<vector<tuple<int, Edge*>>> convertToDirected(const vector<vector<tuple<int, Edge*>>>& adj);
void printDirectedAdjacencyList(const vector<vector<tuple<int, Edge*>>>& directedAdj);


#endif
