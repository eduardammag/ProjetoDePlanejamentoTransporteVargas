## Planejamento de Meios de Transporte na cidade "Vargas" usando C++

### Integrantes:
- Ana Júlia Amaro Pereira Rocha
- Mariana Fernandes Rocha
- Maria Eduarda Mesquita Magalhães
- Paula Eduarda de Lima

### Objetivo:
Trabalho referente à disciplina de Projeto e Análise de Algoritmos do curso de Ciência de Dados e Inteligência Artificial da FGV-EMAp.
O objetivo é projetar alguns meios de transporte de uma cidade "Vargas" como um grafo em C++ e assim explorar conceitos estudados na disciplina em um problema da vida real.

### Modelagem:
O grafo da cidade foi feito em python, de modo que já obtemos um json com os vértices e seus respectivos atributos (id e booleano indicando se é uma estação de metrô), além das arestas com seus respectivos atributos também (id, distância, vértices 1 e 2 que a integram, código da região e da rua e a taxa de tráfego). O json é lido em c++ formando objetos das classes Edge e Vertex. 
O grafo é quadrado e todas as arestas são ou verticais ou horizontais. Todos os vértices possuem grau maior ou igual a 2. Uma visualização do grafo da cidade pode ser encontrada em graph_generation/imagens/grafo_cidade.png
Foram feitas alguns testes com tamanhos diferentes de grafo, por isso na pasta graph_generation/images pode-se encontrar outras imagens e arquivos referentes aos grafos.

### Tarefa 1:
Foi feito uma função para definir as estações de metrô (sendo uma em cada região) de modo que a escolha de cada estação busca minimizar a distância entre ela e o ponto mais longe da sua respectiva região. 
Além disso, foi projetado um algoritmo capaz de definir os segmentos a serem escavados, de forma que o custo para a cidade seja mínimo, mas todas as estações definidas sejam conectadas.

### Tarefa 3:


### Execução:


