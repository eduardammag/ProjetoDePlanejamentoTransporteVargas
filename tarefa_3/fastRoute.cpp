#include "fastRoute.h"
#include <vector>

using namespace std;


// O(V+E)
Edge* findEdgeAddress(int street, int id_zipCode, int number_build, const vector<Edge*>& edges) {
    for (Edge* edge : edges) {
        if (edge->id_street() == street && edge->id_zipCode() == id_zipCode) {
            // Supondo que o número do imóvel esteja entre os dois vértices da aresta
            if (edge->vertex1()->id() <= number_build && edge->vertex2()->id() >= number_build) {
                return edge;
            }
        }
    }
    return nullptr; // Retorna nullptr caso nenhuma aresta seja encontrada
}


findSameRigion( ){
    vai o maximo de taxi dentro do orçamento e o resto a  pe de graça
    ou só a pé
}

vector<tuple<str, Edge*>> findFastRoute( int street, int id_zipCode, int number_build, int max_cost, int hour){
    taxi completo = preço tempo usando dijkstra direcionado transito*velocidade
    a pé = 0 reais tempo grafo não direcionado velocidade constante
    se custo< max_custo : break;

    testa para a estação da mesma região, qual o mais rápido: taxi  ou a pé
    Se no final ultrapassar o orçamento, tentamos a péaté a primeira
        testa até a primeira estação no caminho para maximo de metro, a estação da região do destino:
            Testa taxi e a pé, qual o mais rápido

        Testa mais uma:
            Testa taxi e a pé, qual o mais rapido para o destino final dentro do orçamento
            Se o taxi não está no orçamento e a pé é mais devagar que taxi, e tiver outro metro, testa o metro para a próxima 

        no maximo de metro, a estação da região do destino
        se está na mesma região, não vai usar metrô


}