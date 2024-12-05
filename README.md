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

Dentro dessa pasta, há uma outra pasta que contém os arquivos que analisam o tempo de execução das funções criadas nessa tarefa.

### Tarefa 3:
Foi criado uma função para converter um grafo não-direcionado em um grafo direcionado. Isso, pois precisaríamos desse grafo direcionado para gerar as rotas do taxi visto que este não pode andar na contramão. Depois, foi gerada uma função que criava o caminho otimizado da rota do taxi usando o algoritmo Dijkstra para caminhos mínimos. Simultaneamente a isto, foi construída uma função para encontrar o caminho do metrô mais próximo da origem até o mais próximo do destino utilizando a MST gerada na parte 2 da tarefa 1.

Depois disso, mais duas funções foram criadas, uma para gerar a rota mínima para deslocamento não-motorizado, ou seja, a pé e a outra para verificar o caminho de táxi e o caminho a pé dado o orçamento e o tempo. 

Se os meios alternativos não satisfazerem o orçamento então a locomoção deve se dar por meio não motorizado que não tem custo para o usuário. Nesse caso, não devemos considerar taxa trânsito nem direção das arestas e o caminho que minimiza o tempo será também o de menor distância. 
Para esse problema foi implementado o algortimo de Dijkstra que retorna a distancia que deve ser percorrida e o tempo que leva percorrendo esse percurso.

A última função foi feita para verificar se a viagem de táxi pelo menor caminho seria por um preço menor ou igual ao valor disponível, ou seja, o máximo que o consumidor está disposto a pagar pela viagem até o seu destino. Além disso, se o percurso de táxi for mais caro do que o consumidor pode pagar, analisamos se a menor rota que pode ser feita a pé até o destino gastaria menos tempo que a viagem de carro porque isso quer dizer que não está muito distante, então seria a opção com menor gasto de tempo e custo zero.
Caso uma destas opções seja válida como a melhor partindo do ponto em que está até o endereço destino, retorna as arestas que compõem o caminho indicando o meio de transporte em cada uma.

Assim como para a tarefa 1, nessa pasta há uma outra pasta que contém os arquivos que analisam o tempo de execução das funções criadas nessa tarefa.


### Execução: 
Todo o projeto foi repetidas vezes testado no compilador online ''Online GDB''. Este, por sua vez, não recebe a estrutura de pastas, logo, para rodar o código completo, deve-se colocar todos os arquivos em um mesmo diretório.       

