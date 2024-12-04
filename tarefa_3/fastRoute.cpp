#include "fastRoute.h"
#include <iostream> 

using namespace std; 

// Função para encontrar a aresta correspondente a um número de imóvel em uma rua.
Edge* findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges) {
    int current_distance = 0; // Distância acumulada ao longo da rua.

    // Ordena as arestas para garantir a sequência crescente dos IDs dos vértices.
    vector<Edge*> sorted_edges = edges;
    sort(sorted_edges.begin(), sorted_edges.end(), [](Edge* a, Edge* b) {
        return min(a->vertex1()->id(), a->vertex2()->id()) < min(b->vertex1()->id(), b->vertex2()->id());
    });

    for (Edge* edge : sorted_edges) {
        // cout << "Verificando aresta: Rua ID " << edge->id_street() << ", CEP " << edge->id_zipCode() << endl;
    
        if (edge->id_street() == street && edge->id_zipCode() == id_zipCode) {
            int id_start = min(edge->vertex1()->id(), edge->vertex2()->id());
            int id_end = max(edge->vertex1()->id(), edge->vertex2()->id());
    
            int start_number = current_distance + 1;
            int end_number = current_distance + edge->distance();
    
            // cout << "Aresta encontrada: Vértices (" << id_start << ", " << id_end << ")" << endl;
            // cout << "Intervalo de números na aresta: [" << start_number << ", " << end_number << "]" << endl;
    
            if (number_build >= start_number && number_build <= end_number) {
                // cout << "Número do imóvel encontrado nesta aresta!" << endl;
                return edge;
            }
    
            current_distance = end_number; // Atualiza a distância acumulada.
        } else {
            // cout << "Aresta ignorada: não corresponde à rua ou ao CEP." << endl;
        }
    }


    return nullptr; // Retorna nullptr caso nenhuma aresta seja encontrada.
}


// findSameRigion( ){
//     vai o maximo de taxi dentro do orçamento e o resto a  pe de graça
//     ou só a pé
// }

// vector<tuple<str, Edge*>> findFastRoute( int street, int id_zipCode, int number_build, int max_cost, int hour){
//     taxi completo = preço tempo usando dijkstra direcionado transito*velocidade
//     a pé = 0 reais tempo grafo não direcionado velocidade constante
//     se custo< max_custo : break;

//     testa para a estação da mesma região, qual o mais rápido: taxi  ou a pé
//     Se no final ultrapassar o orçamento, tentamos a péaté a primeira
//         testa até a primeira estação no caminho para maximo de metro, a estação da região do destino:
//             Testa taxi e a pé, qual o mais rápido

//         Testa mais uma:
//             Testa taxi e a pé, qual o mais rapido para o destino final dentro do orçamento
//             Se o taxi não está no orçamento e a pé é mais devagar que taxi, e tiver outro metro, testa o metro para a próxima 

//         no maximo de metro, a estação da região do destino
//         se está na mesma região, não vai usar metrô


// }