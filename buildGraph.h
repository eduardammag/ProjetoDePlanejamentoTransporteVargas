#ifndef BUILDGRAPH_H
#define BUILDGRAPH_H

#include <vector>
#include <string>
#include <tuple>
#include "vertexAndEdge.h"

// Função para fazer o parse do arquivo JSON e preencher os vetores de vértices e arestas
void parseJsonFile(const std::string& filePath, std::vector<Vertex*>& vertices, std::vector<Edge*>& edges);

// Função para gerar a matriz de adjacência
void generateAdjacencyMatrix(const std::vector<Vertex*>& vertices, const std::vector<Edge*>& edges, std::vector<std::vector<Edge*>>& adjacencyMatrix);

// Função para converter a matriz de adjacência para lista de adjacência
void generateAdjacencyList(const std::vector<std::vector<Edge*>>& adjacencyMatrix, std::vector<std::vector<std::tuple<int, Edge*>>>& adjacencyList);

#endif // BUILDGRAPH_H
