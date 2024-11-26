#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>

using namespace std;

typedef int vertex;

class WeightedGraph {
private:
    int numVertices;
    int numEdges;
    vector<vector<pair<int, int>>> adjList; // Lista de adjacências: {vértice destino, peso}

public:
    WeightedGraph(int numVertices)
        : numVertices(numVertices), numEdges(0) {
        adjList.resize(numVertices);
    }

    void addEdge(vertex v1, vertex v2, int weight) {
        if (v1 < numVertices && v2 < numVertices) {
            adjList[v1].push_back({v2, weight});
            numEdges++;
        }
    }

    void removeEdge(vertex v1, vertex v2) {
        if (v1 < numVertices) {
            auto& neighbors = adjList[v1];
            for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
                if (it->first == v2) {
                    neighbors.erase(it);
                    numEdges--;
                    break;
                }
            }
        }
    }

    int getNumEdges() const {
        return numEdges;
    }

    int getNumVertices() const {
        return numVertices;
    }

    const vector<pair<int, int>>& getNeighbors(int vertex) const {
        return adjList[vertex];
    }

    void print() {
        for (int i = 0; i < numVertices; i++) {
            cout << "Vértice " << i << ": ";
            for (const auto& neighbor : adjList[i]) {
                cout << "(" << neighbor.first << ", peso: " << neighbor.second << ") ";
            }
            cout << endl;
        }
    }
    
    void bfs(vertex start, vector<int>& order, vector<int>& parent) {
        // Inicializa os vetores
        order.assign(numVertices, -1);
        parent.assign(numVertices, -1);
        int counter = 0;
    
        // Fila para a BFS
        queue<vertex> q;
        order[start] = counter++;
        parent[start] = start;
        q.push(start);
    
        while (!q.empty()) {
            vertex v = q.front();
            q.pop();
    
            // Percorre os vizinhos de `v`
            for (const auto& neighbor : adjList[v]) {
                vertex u = neighbor.first; // Pega o vértice destino
                if (order[u] == -1) { // Se não foi visitado
                    order[u] = counter++;
                    parent[u] = v;
                    q.push(u);
                }
            }
        }
    }
};


// =====================================
// QUESTÂO 3 (inicial)
// =====================================
// int cptBellman(WeightedGraph& graph, int start, int end) {
//     int numVertices = graph.getNumVertices();
//     vector<int> distance(numVertices, INT_MAX);

//     // Inicializar a distância de start como 0
//     distance[start] = 0;

//     // Relaxar as arestas (V - 1) vezes
//     for (int i = 0; i < numVertices - 1; i++) {
//         for (int j = 0; j < numVertices; j++) {
//             for (const auto& [v, weight] : graph.getNeighbors(j)) {
//                 if (distance[j] != INT_MAX && distance[j] + weight < distance[v]) {
//                     distance[v] = distance[j] + weight;
//                 }
//             }
//         }
//     }

//     // Verificar ciclos negativos
//     for (int u = 0; u < numVertices; u++) {
//         for (const auto& [v, weight] : graph.getNeighbors(u)) {
//             if (distance[u] != INT_MAX && distance[u] + weight < distance[v]) {
//                 cout << "O grafo contém um ciclo negativo!" << endl;
//                 return INT_MIN;
//             }
//         }
//     }

//     cout << "Vertex: " << end << ". L1: " << start << ". distance: " << distance[end] << endl;
//     // Retornar a distância mínima até o vértice end
//     return distance[end];
// }


// int maxDistanceinL(WeightedGraph& graph, vector<int>& L, int vertex) {
//     int maxDistance = INT_MIN;
    
//     for(int l : L) {
//         int distance = cptBellman(graph, l, vertex);
//         if (distance > maxDistance) {
//             maxDistance = distance;
//         }
//     }
//     cout << "A máxima para " << vertex << " foi " << maxDistance << endl;
//     return maxDistance;
// }


// int cheapestVertex(WeightedGraph& graph, vector<int>& L) {
//     int bestResult = INT_MAX;
//     int bestVertex = -1;
    
//     for (int i = 0; i < graph.getNumVertices(); i++) {
//         int result = maxDistanceinL(graph, L, i);
        
//         if (result < bestResult) {
//             bestResult = result;
//             bestVertex = i;
//         }
//     }
    
//     return bestVertex;
// }

// =====================================
// QUESTÂO 3 (final)
// =====================================
vector<int> bellmanFord(WeightedGraph& graph, int start) {
    int numVertices = graph.getNumVertices();
    vector<int> distances(numVertices, INT_MAX);

    // Inicializa a distância do vértice inicial como 0
    distances[start] = 0;

    // Relaxa todas as arestas (V - 1) vezes
    for (int i = 0; i < numVertices - 1; i++) {
        for (int u = 0; u < numVertices; u++) {
            for (const auto& [v, weight] : graph.getNeighbors(u)) {
                if (distances[u] != INT_MAX && distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                }
            }
        }
    }
    
    for (int u = 0; u < numVertices; u++) {
            for (const auto& [v, weight] : graph.getNeighbors(u)) {
                if (distances[u] != INT_MAX && distances[u] + weight < distances[v]) {
                    throw runtime_error("O grafo tem um ciclo negativo");
                }
            }
        }
    
    return distances;
}

vector<int> dijkstra(const WeightedGraph& graph, int start) {
    int numVertices = graph.getNumVertices();
    vector<int> distances(numVertices, INT_MAX); // Inicializa todas as distâncias como infinito
    distances[start] = 0; // A distância para o próprio vértice inicial é 0

    // Fila de prioridade implementada (distância, vértice)
    priority_queue<pair<int, int>> heap;
    heap.push({0, start});

    while (!heap.empty()) {
        int currentDistance = -heap.top().first; // Negativo para simular menor prioridade
        int currentVertex = heap.top().second;
        heap.pop();

        // Ignora se a distância não for mínima (processado antes)
        if (currentDistance > distances[currentVertex]) continue;

        // Relaxa todas as arestas do vértice atual
        for (const auto& [neighbor, weight] : graph.getNeighbors(currentVertex)) {
            if (distances[currentVertex] + weight < distances[neighbor]) {
                distances[neighbor] = distances[currentVertex] + weight;
                heap.push({-distances[neighbor], neighbor}); // Insere com distância negativa
            }
        }
    }

    return distances; // Retorna as distâncias mínimas de start para todos os vértices
}

vector<vector<int>> johnson(WeightedGraph& graph) {
    int numVertices = graph.getNumVertices();
    WeightedGraph tempGraph(numVertices + 1);

    // Copia as arestas do grafo original
    for (int u = 0; u < numVertices; u++) {
        for (const auto& [v, weight] : graph.getNeighbors(u)) {
            tempGraph.addEdge(u, v, weight);
        }
    }

    // Adiciona vértice temporário conectado a todos os outros (com peso 0)
    int tempVertex = numVertices;
    for (int i = 0; i < numVertices; i++) {
        tempGraph.addEdge(tempVertex, i, 0);
    }

    // Passo 1: Executa Bellman-Ford a partir do vértice temporário
    // para encontrar a função de potencial
    vector<int> h = bellmanFord(tempGraph, numVertices);
    
    WeightedGraph adjustedGraph(numVertices); // Para adicionar novos pesos
    
    for (int u = 0; u < numVertices; u++) {
        for (auto& [v, weight] : graph.getNeighbors(u)) {
            int newWeight = weight + h[u] - h[v]; // Função 
            adjustedGraph.addEdge(u, v, newWeight);
        }
    }

    // Inicializa a matriz de menores caminhos
    vector<vector<int>> allPairsShortestPaths(numVertices, vector<int>(numVertices, INT_MAX));

    // Calcula Dijkstra para cada vértice
    for (int u = 0; u < numVertices; u++) {
        allPairsShortestPaths[u] = dijkstra(adjustedGraph, u);
    }

    return allPairsShortestPaths;
}

int cheapestVertex(WeightedGraph& graph, const vector<int>& L) {
    vector<vector<int>> distances = johnson(graph);

    int bestVertex = -1;
    int bestResult = INT_MAX;

    for (int v = 0; v < graph.getNumVertices(); v++) {
        int maxDistance = 0;

        for (int l : L) {
            maxDistance = max(maxDistance, distances[l][v]);
        }

        if (maxDistance < bestResult) {
            bestResult = maxDistance;
            bestVertex = v;
        }
    }

    return bestVertex;
}


// =====================================
// QUESTÂO 4
// =====================================
void dijkstraWithPath(const WeightedGraph& graph, int start, int end, vector<int>& pathX) {
    int numVertices = graph.getNumVertices();
    vector<int> distances(numVertices, INT_MAX);
    vector<int> parents(numVertices, -1);
    distances[start] = 0;

    priority_queue<pair<int, int>> heap;
    heap.push({0, start});

    while (!heap.empty()) {
        int currentDistance = -heap.top().first;
        int currentVertex = heap.top().second;
        heap.pop();

        if (currentDistance > distances[currentVertex]) continue;

        for (const auto& [neighbor, weight] : graph.getNeighbors(currentVertex)) {
            if (distances[currentVertex] + weight < distances[neighbor]) {
                distances[neighbor] = distances[currentVertex] + weight;
                heap.push({-distances[neighbor], neighbor});
                parents[neighbor] = currentVertex;
            }
        }
    }

    if (distances[end] == INT_MAX) {
        cout << "Não há caminho entre os vértices!" << endl;
        return;
    }

    for (int i = end; i != -1; i = parents[i]) {
        pathX.push_back(i);
    }
    reverse(pathX.begin(), pathX.end());
}

vector<int> bestCPath(WeightedGraph& graph, vector<int>& path, int X, vector<int>& pathX) {
    int numVertices = graph.getNumVertices();
    vector<int> notInC;
    vector<int> inC;
    
    // Separa os vértices entre notInC (não estão no path) e inC (estão no path)
    for (int v = 0; v < numVertices; v++) {
        if (find(path.begin(), path.end(), v) == path.end()) {
            notInC.push_back(v); // Vértices que não estão no path
        } else {
            inC.push_back(v); // Vértices que estão no path
        }
    }
    
    // Vetor que indicará quais vértices são válidos
    vector<bool> validVertexes(numVertices, false);
    
    // Todos os vértices em C são válidos
    for (int c : inC) {
        validVertexes[c] = true;
    }
    
    // Para cada vértice em notInC, verificamos se a distância até algum vértice em inC é menor que X
    for (int v : notInC) {
        vector<int> distances = dijkstra(graph, v); // Executa Dijkstra a partir do vértice v
        
        // Se a distância a algum vértice em inC for menor que X, marca o vértice v como válido
        for (int c : inC) {
            if (distances[c] < X) {
                validVertexes[v] = true;
                break;
            }
        }
    }
    
    // Criação de um grafo temporário contendo apenas as arestas entre vértices válidos
    WeightedGraph nearGraph(numVertices);
    
    // Adiciona arestas entre vértices válidos no grafo temporário
    for (int v = 0; v < numVertices; v++) {
        if (validVertexes[v]) {
            for (auto& [u, weight] : graph.getNeighbors(v)) {
                if (validVertexes[u]) {
                    nearGraph.addEdge(v, u, weight); // Adiciona aresta entre v e u, se ambos forem válidos
                }
            }
        }
    }
    
    // Executa o Dijkstra para encontrar o melhor caminho no grafo temporário `nearGraph`
    int start = path[0];
    int end = path[path.size() - 1];
    
    dijkstraWithPath(nearGraph, start, end, pathX); // Executa o Dijkstra no grafo temporário
    
    return pathX;
}


// =====================================
// QUESTÂO 5
// =====================================
// void mstPrim(WeightedGraph& graph, WeightedGraph& mst) {
//     int numVertices = graph.getNumVertices();
//     vector<int> parent(numVertices, -1);       // Array para armazenar os pais
//     vector<bool> inTree(numVertices, false);  // Marcar vértices já incluídos na MST
//     vector<int> vertexCost(numVertices, INT_MAX); // Custos dos vértices para MST

//     vertexCost[0] = 0; // Começa pelo vértice 0
//     parent[0] = -1;    // Nenhum pai para o vértice inicial

//     for (int k = 0; k < numVertices - 1; k++) {
//         // Encontra o vértice com menor custo não incluído na MST
//         int minCost = INT_MAX;
//         int v1 = -1;
//         for (int i = 0; i < numVertices; i++) {
//             if (!inTree[i] && vertexCost[i] < minCost) {
//                 minCost = vertexCost[i];
//                 v1 = i;
//             }
//         }

//         if (v1 == -1) break; // Verifica se é desconexo

//         // Adiciona o vértice encontrado à MST
//         inTree[v1] = true;

//         // Atualiza os custos dos vértices adjacentes
//         for (const auto& [v, weight] : graph.getNeighbors(v1)) {
//             if (!inTree[v] && weight < vertexCost[v]) {
//                 parent[v] = v1;
//                 vertexCost[v] = weight;
//             }
//         }
//     }

//     // Adiciona as arestas da MST ao grafo resultante
//     for (int i = 1; i < numVertices; i++) {
//         if (parent[i] != -1) {
//             mst.addEdge(parent[i], i, vertexCost[i]); // Adiciona a aresta ao MST
//             mst.addEdge(i, parent[i], vertexCost[i]); // Pros 2 lados
//         }
//     }
// }

void mstPrim(WeightedGraph& graph, WeightedGraph& mst) {
    int numVertices = graph.getNumVertices();
    vector<int> parent(numVertices, -1);     
    vector<bool> inTree(numVertices, false);  
    vector<int> vertexCost(numVertices, INT_MAX);

    // Inicializa a heap e o custo do vértice inicial
    priority_queue<pair<int, int>> heap;
    vertexCost[0] = 0; // Custo do vértice inicial é 0
    heap.push({0, 0});  // (custo, vértice)

    while (!heap.empty()) {
        int v1 = heap.top().second;
        int currentCost = -heap.top().first;
        heap.pop();

        // Verifica se o vértice já está na MST
        if (inTree[v1]) continue;

        // Adiciona o vértice à MST
        inTree[v1] = true;

        // Atualiza os custos dos vizinhos
        for (const auto& [v2, weight] : graph.getNeighbors(v1)) {
            if (!inTree[v2] && weight < vertexCost[v2]) {
                parent[v2] = v1;
                vertexCost[v2] = weight;
                heap.push({-weight, v2}); // (negativo para min-heap)
            }
        }
    }

    // Adiciona as arestas ao grafo da MST
    for (int i = 1; i < numVertices; i++) {
        if (parent[i] != -1) { // Se o vértice tem um pai válido
            mst.addEdge(parent[i], i, vertexCost[i]); // Adiciona a aresta
            mst.addEdge(i, parent[i], vertexCost[i]); // Adiciona na direção inversa
        }
    }
}

pair<int, int> findCriticalEdge(WeightedGraph& graph, WeightedGraph& mst) {
    int originalCost = 0;
    int numVertices = graph.getNumVertices();

    // Calcula o custo original da MST
    for (int v = 0; v < numVertices; v++) {
        for (const auto& [u, weight] : mst.getNeighbors(v)) {
            if (v < u) { // Evitar contar duas vezes
                originalCost += weight;
            }
        }
    }

    int maxIncrease = 0;
    pair<int, int> criticalEdge;
    vector<bool> inComponent;

    for (int v = 0; v < numVertices; v++) {
        for (const auto& [u, weight] : mst.getNeighbors(v)) {
            if (v < u) { // Evitar duplicação de arestas
                int minIncrease = INT_MAX;
    
                // Reseta o vetor de componentes e inicializa o BFS
                inComponent.assign(numVertices, false);
                inComponent[v] = true;
    
                queue<int> q;
                q.push(v);
    
                // BFS para marcar os vértices do componente
                while (!q.empty()) {
                    int v1 = q.front();
                    q.pop();
    
                    for (const auto& [v2, _] : mst.getNeighbors(v1)) {
                        if (!inComponent[v2] && !(v1 == v && v2 == u)) { // Ignorar aresta removida
                            q.push(v2);
                            inComponent[v2] = true;
                        }
                    }
                }
    
                // Busca a menor aresta que conecta os dois componentes
                for (int v1 = 0; v1 < numVertices; v1++) {
                    for (const auto& [v2, weight2] : graph.getNeighbors(v1)) {
                        if (inComponent[v1] && !inComponent[v2] && weight < minIncrease && !(v1 == v && v2 == u)) {
                            minIncrease = weight2 - weight;
                        }
                    }
                }
    
                cout << "minIncrease de " << v << " e " << u << " é " << minIncrease << endl;
    
                // Atualiza a aresta crítica se necessário
                if (minIncrease > maxIncrease) {
                    maxIncrease = minIncrease;
                    criticalEdge = {v, u};
                }
            }
        }
    }

    return criticalEdge;
}




int main() {
    // Criar um grafo ponderado com 5 vértices
    WeightedGraph graph(5);

    // Adicionar arestas com pesos
    graph.addEdge(0, 1, -2);
    graph.addEdge(0, 2, 3);
    graph.addEdge(1, 2, 4);
    graph.addEdge(1, 3, 5);
    graph.addEdge(2, 4, 10);
    graph.addEdge(3, 4, 2);
    graph.addEdge(4, 0, 4);
    graph.addEdge(4, 3, 1);

    // Imprimir o grafo
    cout << "Lista de Adjacência do Grafo Ponderado:" << endl;
    graph.print();
    
    // =====================================
    // QUESTÂO 3
    // =====================================
    // Lista de vértices que o caminho deve visitar em ordem
    vector<int> L = {0, 3, 4};

    // Testando a função shortestLPath
    cout << "\nTestando cheapestVertex com L = {0, 3, 4}:" << endl;
    int result = cheapestVertex(graph, L);
    cout << "O vértice com a menor maior distância é: " << result << "\n" << endl;

    
    // =====================================
    // QUESTÂO 4
    // =====================================
    WeightedGraph graph2(8);

    // Adicionar arestas com pesos
    graph2.addEdge(0, 1, 3);
    graph2.addEdge(1, 2, 3);
    graph2.addEdge(1, 4, 1);
    graph2.addEdge(1, 5, 4);
    graph2.addEdge(2, 3, 20);
    graph2.addEdge(2, 7, 4);
    graph2.addEdge(4, 3, 6);
    graph2.addEdge(5, 6, 3);
    graph2.addEdge(6, 7, 0);
    graph2.addEdge(7, 3, 4);
    
    vector<int> pathOriginal = {0, 1, 2, 3};
    vector<int> newPath;
    int X1 = 5;
    
    vector<int> results = bestCPath(graph2, pathOriginal, X1, newPath);

    // Exibir o resultado
    cout << "Novo caminho com X = 5: ";
    for (int vertex : results) {
        cout << vertex << " ";
    }
    cout << endl;
    
    vector<int> newPath2;
    X1 = 7;
    
    results = bestCPath(graph2, pathOriginal, X1, newPath2);

    // Exibir o resultado
    cout << "Novo caminho com X = 7: ";
    for (int vertex : results) {
        cout << vertex << " ";
    }
    cout << endl;
    
    // =====================================
    // QUESTÂO 5
    // =====================================
    WeightedGraph graph3(6);

    // Adicionar arestas com pesos (Agora para ambos lados)
    graph3.addEdge(0, 1, 6);
    graph3.addEdge(1, 0, 6);
    graph3.addEdge(0, 5, 0);
    graph3.addEdge(5, 0, 0);
    graph3.addEdge(1, 2, 4);
    graph3.addEdge(2, 1, 4);
    graph3.addEdge(1, 3, 8);
    graph3.addEdge(3, 1, 8);
    graph3.addEdge(1, 5, 3);
    graph3.addEdge(5, 1, 3);
    graph3.addEdge(2, 4, 5);
    graph3.addEdge(4, 2, 5);
    graph3.addEdge(3, 4, -2);
    graph3.addEdge(4, 3, -2);
    graph3.addEdge(3, 5, 2);
    graph3.addEdge(5, 3, 2);
    graph3.addEdge(4, 5, 5);
    graph3.addEdge(5, 4, 5);
    
    WeightedGraph mst(6);
    
    mstPrim(graph3, mst);
    
    cout << "\n";
    mst.print();
    cout << "\n";
    
    pair<int, int> criticalEdge = findCriticalEdge(graph3, mst);

    cout << "Par crítico: " << criticalEdge.first << " e " << criticalEdge.second << endl;
    
    return 0;
}

