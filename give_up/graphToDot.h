#ifndef GRAPHTODOT_H
#define GRAPHTODOT_H

#include <string>
#include "graph.h"

// Função para exportar o grafo para um arquivo DOT
void graphToDot(const Graph& g, const std::string& filename);

#endif
