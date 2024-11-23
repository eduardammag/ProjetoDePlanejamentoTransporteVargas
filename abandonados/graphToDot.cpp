#include <fstream>
#include "graph.h"

// Função para exportar o grafo para um arquivo DOT
void graphToDot(const Graph& g, const std::string& filename) {
    std::ofstream file(filename);
    file << "graph G {\n";
    
    // Itera sobre as arestas e adiciona as conexões no formato DOT
    for (const auto& edge : g.edges) {
        int vertex1Id = edge->vertex1()->id();
        int vertex2Id = edge->vertex2()->id();
        
        // Adiciona a aresta ao arquivo
        file << "  " << vertex1Id << " -- " << vertex2Id << ";\n";
    }

    file << "}\n";
    file.close();
}
