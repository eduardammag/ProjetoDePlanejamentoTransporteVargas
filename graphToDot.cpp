#include <fstream>
#include <string>
#include <vector>

using namespace std;


// Função para visualizar o grafp construído

// Talvez tenha que mudar o tipo da matriz
void graphDot(int** matrix, const string& filename, int numVertex) 
{
    ofstream file(filename);
    file << "graph G {\n";
    
    // Itera sobre os elementos acima da diagonal
    for (int i = 0; i < numVertex; i++)
    {
        for (int j = i + 1; j < numVertex; j++) 
        { 
            if (matrix[i][j] =! nullptr) 
            {
                file << "  " << i << " -- " << j << ";\n";
            }
        }
    }

    file << "}\n";
    file.close();
}

int main() 
{   // Matriz teste
    int numVertex = 5;
    int** matrix = new int*[numVertex];
    for (int i = 0; i < numVertex; i++) {
        matrix[i] = new int[numVertex]{0}; 
    }
    matrix[0][1] = 1; 
    matrix[1][2] = 1; 
    matrix[2][3] = 1; 
    matrix[3][4] = 1; 
    matrix[0][4] = 1; 

    // Exporta o grafo para um arquivo DOT
    graphDot(matrix, "grafo.dot", numVertex);

    // Liberação da memória alocada
    for (int i = 0; i < numVertex; i++) {
        delete[] matrix[i];
    }
    delete[] matrix;

    return 0;
}
