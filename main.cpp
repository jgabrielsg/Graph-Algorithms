#include <iostream>
#include <vector>
#include <queue>
#include <algorithm> 
#include <climits>

typedef int vertex;

using namespace std;

class GraphAdjList {
private:
    int numVertices;
    int numEdges;
    vector<vector<int>> adjList;

public:
    GraphAdjList(int numVertices)
        : numVertices(numVertices), numEdges(0) {
        adjList.resize(numVertices);
    }

    void addEdge(vertex v1, vertex v2) {
        if (v1 < numVertices && v2 < numVertices) {
            adjList[v1].push_back(v2);
            numEdges++;
        }
    }

    void removeEdge(vertex v1, vertex v2) {
        if (v1 < numVertices && v2 < numVertices) {
            auto& adj = adjList[v1];
            adj.erase(std::remove(adj.begin(), adj.end(), v2), adj.end()); // Corrigido std::remove
            numEdges--;
        }
    }

    int getNumEdges() const {
        return numEdges;
    }

    int getNumVertices() const {
        return numVertices;
    }
    
    const vector<int>& getNeighbors(int vertex) const {
        return adjList[vertex];
    }

    void print() {
        for (int i = 0; i < numVertices; i++) {
            cout << "Vértice " << i << ": ";
            for (int j : adjList[i]) {
                cout << j << " ";
            }
            cout << endl;
        }
    }

    // Função DFS com pré-ordem e pós-ordem
    void dfs(vector<int>& preOrder, vector<int>& postOrder, vector<int>& parents) {
        preOrder.assign(numVertices, -1);
        postOrder.assign(numVertices, -1);
        parents.assign(numVertices, -1);
        int preCounter = 0, postCounter = 0;
        
        for (vertex v = 0; v < numVertices; ++v) {
            if (preOrder[v] == -1) {
                parents[v] = v;
                dfsRecursive(v, preOrder, preCounter, postOrder, postCounter, parents);
            }
        }
    }

    void dfsRecursive(vertex v, vector<int>& preOrder, int& preCounter, vector<int>& postOrder, int& postCounter, vector<int>& parents) {
        preOrder[v] = preCounter++;
        for (vertex neighbor : adjList[v]) {
            if (preOrder[neighbor] == -1) {
                parents[neighbor] = v;
                dfsRecursive(neighbor, preOrder, preCounter, postOrder, postCounter, parents);
            }
        }
        postOrder[v] = postCounter++;
    }

    // Função para verificar ciclos usando DFS
    bool hasCycle() {
        vector<int> preOrder(numVertices, -1);
        vector<int> postOrder(numVertices, -1);
        vector<int> parents(numVertices, -1); // Criar parents como uma variável separada
        dfs(preOrder, postOrder, parents);

        for (vertex v = 0; v < numVertices; ++v) {
            for (vertex neighbor : adjList[v]) {
                if (preOrder[v] > preOrder[neighbor] && postOrder[v] < postOrder[neighbor]) {
                    return true; // Ciclo detectado
                }
            }
        }
        return false;
    }

    // Função de ordenação topológica para verificar se o grafo possui uma ordem válida
    bool hasTopologicalOrder(vector<int>& order) {
        vector<int> inDegree(numVertices, 0);
        for (vertex v = 0; v < numVertices; ++v) {
            for (vertex neighbor : adjList[v]) {
                inDegree[neighbor]++;
            }
        }

        queue<vertex> q;
        for (vertex v = 0; v < numVertices; ++v) {
            if (inDegree[v] == 0) q.push(v);
        }

        int counter = 0;
        while (!q.empty()) {
            vertex v = q.front();
            q.pop();
            order[v] = counter++;
            for (vertex neighbor : adjList[v]) {
                if (--inDegree[neighbor] == 0) q.push(neighbor);
            }
        }

        return counter == numVertices;
    }
    
    // BFS
    void bfs(vertex start, vector<int>& order, vector<int>& parent) {
        order.assign(numVertices, -1);
        parent.assign(numVertices, -1);
        int counter = 0;
        
        queue<vertex> q;
        order[start] = counter++;
        parent[start] = start;
        q.push(start);
        
        while (!q.empty()) {
            vertex v = q.front();
            q.pop();
            for (vertex neighbor : adjList[v]) {
                if (order[neighbor] == -1) {
                    order[neighbor] = counter++;
                    parent[neighbor] = v;
                    q.push(neighbor);
                }
            }
        }
    }
    
};

// =====================================
// QUESTÂO 1
// =====================================
void getRadicalRecursive(GraphAdjList& graph, int root, GraphAdjList& radicalTree, vector<bool>& visited) {
    for (int neighbor : graph.getNeighbors(root)) {  // Para cada vizinho do vértice atual
        if (!visited[neighbor]) {              // Se o vizinho ainda não foi visitado
            visited[neighbor] = true;          // Marca como visitado
            
            radicalTree.addEdge(root, neighbor); // Adiciona aresta na árvore radicada
            
            // Chamada recursiva para explorar o vizinho
            getRadicalRecursive(graph, neighbor, radicalTree, visited);
        }
    }
}

void getRadicalTree(GraphAdjList& graph, int root, GraphAdjList& radicalTree) {
    int size = graph.getNumVertices();                    
    vector<bool> visited(size, false);     // Vetor de visitados para evitar ciclos
    
    visited[root] = true;                   // Marca o vértice raiz como visitado
    getRadicalRecursive(graph, root, radicalTree, visited);
}

// =====================================
// QUESTÂO 1. a)
// =====================================
void getList1(GraphAdjList& graph, GraphAdjList& radicalTree, vector<bool>& L1, vector<bool>& L2) {
    int numVertices = radicalTree.getNumVertices();
    L2.assign(numVertices, false);  // Aqui nós inicializamos L2 como falso para todos os vértices
    
    vector<int> preOrder;
    vector<int> postOrder;              
    vector<int> parents(numVertices, -1); 
    
    // Aqui nós fazemos uma DFS na árvore radicada para determinar as ordens pré e pós
    radicalTree.dfs(preOrder, postOrder, parents);
    
    vector<int> finalOrder(numVertices); // Aqui guardamos a ordem de finalização dos vértices
    
    for (int v = 0; v < numVertices; v++) {
        finalOrder[postOrder[v]] = v;  // Aqui construímos a ordem dos vértices a partir da pós-ordem
    }
    
    // Aqui adicionamos L1 em L2
    for (int i = postOrder.size() - 1; i >= 0; i--) {
        if (L1[i]) {
            L2[i] = true;  // Se o vértice está em L1, colocamos ele em L2
        }
    }
    
    vector<int> oddCount(numVertices, 0);  // Aqui guardamos os descendentes ímpares para cada vértice

    // Aqui nós processamos os vértices na ordem de pós-ordem (de baixo para cima na árvore)
    for (int i = 0; i < postOrder.size(); i++) {
        int vertex = finalOrder[i]; // Pegamos o vértice atual

        // Aqui iteramos sobre os filhos diretos do vértice
        for (int child : radicalTree.getNeighbors(vertex)) {
            if (parents[child] == vertex && L2[child]) { // Verificamos se é um filho direto e está em L2
                oddCount[vertex]++;  // Aqui somamos o filho ao contador de ímpares do vértice
                oddCount[vertex] += oddCount[child];  // E somamos também os descendentes ímpares do filho
            }
        }

        // Se o número de descendentes ímpares for ímpar, colocamos o vértice em L2
        if (oddCount[vertex] % 2 == 1) {
            L2[vertex] = true;
        }
    }
    // Aqui imprimimos o vetor com os contadores de descendentes ímpares para cada vértice
    cout << "\nOddcount:    ";
    for (int v : oddCount) cout << v << " ";
    cout << endl;
}


// =====================================
// QUESTÂO 1. b)
// =====================================
void getList2(GraphAdjList& graph, GraphAdjList& radicalTree, vector<bool>& L1, vector<bool>& L2) {
    int numVertices = radicalTree.getNumVertices();
    L2.assign(numVertices, false);  // Aqui nós inicializamos L2 como falso para todos os vértices
    
    vector<int> preOrder;
    vector<int> postOrder;              
    vector<int> parents(numVertices, -1); 
    
    // Aqui nós fazemos uma DFS na árvore radicada para determinar as ordens pré e pós
    radicalTree.dfs(preOrder, postOrder, parents);
    
    vector<int> finalOrder(numVertices); // Aqui guardamos a ordem de finalização dos vértices
    
    for (int v = 0; v < numVertices; v++) {
        finalOrder[postOrder[v]] = v;  // Aqui construímos a ordem dos vértices a partir da pós-ordem
    }
    
    // Aqui adicionamos L1 em L2
    for (int i = postOrder.size() - 1; i >= 0; i--) {
        if (L1[i]) {
            L2[i] = true;  // Se o vértice está em L1, colocamos ele em L2
        }
    }
    
    vector<int> oddCount(numVertices, 0);  // Aqui guardamos os ancestrais ímpares para cada vértice

    // Aqui processamos os vértices na ordem de pós-ordem
    for (int i = postOrder.size() - 1; i >= 0; i--) {
        int vertex = finalOrder[i]; // Pegamos o vértice atual

        // Aqui iteramos sobre os filhos diretos do vértice
        for (int child : radicalTree.getNeighbors(vertex)) {
            if (parents[child] == vertex && L2[vertex]) { // Verificamos se é filho direto e está em L2
                oddCount[child]++;  // Somamos o ancestral ao contador do filho
                oddCount[child] += oddCount[vertex];  // Somamos também os ancestrais do vértice
                cout << "Achamos um ancestral para " << child << ", que é: " << vertex << endl;
            }
            if (oddCount[child] % 2 == 1) { // Se o número de ancestrais for ímpar, colocamos o vértice em L2
                L2[child] = true;
            }
        }
    }
    // Aqui imprimimos o vetor com os contadores de ancestrais ímpares para cada vértice
    cout << "\nOddcount:    ";
    for (int v : oddCount) cout << v << " ";
    cout << endl;
}


// =====================================
// QUESTÂO 1. c)
// =====================================
void getList3(GraphAdjList& graph, GraphAdjList& radicalTree, vector<bool>& L1, vector<bool>& L2) {
    int numVertices = radicalTree.getNumVertices();
    L2.assign(numVertices, false);  // Inicializa L2 como falso para todos os vértices
    
    vector<int> preOrder;
    vector<int> postOrder;              
    vector<int> parents(numVertices, -1); 
    
    // Realiza uma DFS na árvore radicada para determinar as ordens pré e pós
    radicalTree.dfs(preOrder, postOrder, parents);
    
    vector<int> finalOrder(numVertices); // Guarda a ordem de finalização dos vértices
    
    for (int v = 0; v < numVertices; v++) {
        finalOrder[postOrder[v]] = v;  // Constrói a ordem dos vértices a partir da pós-ordem
    }
    
    vector<int> descendentsInL2(numVertices, 0);  // Guarda os descendentes acumulados em L2
    vector<int> olderCousins(numVertices, 0);  // Guarda os primos mais velhos
    
    // Adiciona L1 em L2
    for (int i = postOrder.size() - 1; i >= 0; i--) {
        if (L1[i]) {
            L2[i] = true;
            descendentsInL2[i]++;
        }
    }

    // Processa os descendentes acumulados
    for (int i = preOrder.size() - 1; i >= 0; i--) {
        int vertex = preOrder[i];
        
        // Itera sobre os filhos diretos do vértice
        for (int child : radicalTree.getNeighbors(vertex)) {
            descendentsInL2[vertex] += descendentsInL2[child];  // Acumula os descendentes do filho no pai
        }
    }
    
    cout << "\nDescendentsInL2: ";
    for (int v : descendentsInL2) cout << v << " ";
    cout << endl;
    
    for (int i = postOrder.size() - 1; i >= 0; i--) {
        int vertex = finalOrder[i]; // Pega o vértice atual

        int olderCousinsDescendantsCount = 0;

        // Atualiza os primos mais velhos
        for (int child : radicalTree.getNeighbors(vertex)) {
            olderCousinsDescendantsCount += descendentsInL2[child];
            olderCousins[child] += olderCousinsDescendantsCount - descendentsInL2[child];  // Subtrai o acumulado do próprio filho
            olderCousins[child] += olderCousins[vertex];  // Soma o que já foi acumulado nos primos anteriores
            if (olderCousins[child] % 2 == 1) {
                L2[child] = true;  // Coloca o filho em L2 se o número de primos for ímpar
            }
        }
    }
    
    // Imprime os primos mais velhos
    cout << "OlderCousins:   ";
    for (int v : olderCousins) cout << v << " ";
    cout << endl;
}



// =====================================
// QUESTÂO 2
// =====================================
void shortest2Path(GraphAdjList& graph, int l1, int l2, vector<int>& path) {
    int numVertices = graph.getNumVertices();
    vector<int> distance(numVertices, INT_MAX); // Vetor de distâncias, inicializado com infinito
    vector<int> parent(numVertices, -1);        // Vetor para rastrear os pais dos vértices
    queue<int> q;                               // Fila para BFS
    
    // Inicializa BFS
    distance[l1] = 0;
    q.push(l1);
    
    // Executa BFS
    while (!q.empty()) {
        int current = q.front();
        q.pop();
        
        // Explora os vizinhos
        for (int neighbor : graph.getNeighbors(current)) {
            if (distance[neighbor] == INT_MAX) { // Se ainda não foi visitado
                distance[neighbor] = distance[current] + 1;
                parent[neighbor] = current;    // Rastreia o pai
                q.push(neighbor);
                
                // Se encontramos o destino, podemos parar
                if (neighbor == l2) {
                    break;
                }
            }
        }
    }

    vector<int> twoPath;
    for (int i = l2; i != -1; i = parent[i]) {
        twoPath.push_back(i);
    }
    reverse(twoPath.begin(), twoPath.end()); // Invertemos para obter o caminho correto
    
    // Adicionamos o caminho reconstruído ao caminho final
    if (!path.empty()) {
        path.pop_back(); // Removemos o último elemento para evitar duplicação
    }
    path.insert(path.end(), twoPath.begin(), twoPath.end()); // Adiciona, no fim, o caminho do primeiro ao último
    
    cout << "Menor caminho entre " << l1 << " e " << l2 << " tem distância: " << distance[l2] << endl;

    cout << "Caminho: ";
    for (int v : twoPath) {
        cout << v << " ";
    }
    cout << endl;
}


void shortestLPath(GraphAdjList& graph, vector<int>& L, vector<int>& path) {
    // Calculamos o menor caminho entre vértices consecutivos em L
    for (int i = 0; i < L.size() - 1; i++) {
        shortest2Path(graph, L[i], L[i + 1], path);
    }

    cout << "Caminho final que passa por todos os vértices de L na ordem: ";
    for (int v : path) {
        cout << v << " ";
    }
    cout << endl;
}



int main() {
    GraphAdjList graph(5);
    graph.addEdge(0, 1);
    graph.addEdge(0, 2);
    graph.addEdge(1, 2);
    graph.addEdge(1, 3);
    graph.addEdge(2, 4);
    graph.addEdge(3, 4);
    
    // Radical
    GraphAdjList radicalTree(5);
    getRadicalTree(graph, 0, radicalTree);
    radicalTree.print();
    
    // =====================================
    // QUESTÂO 1
    // =====================================
    // L1 e l2
    vector<bool> L1(graph.getNumVertices(), false);
    L1[3] = true; L1[4] = true; // Vértices escolhidos
    vector<bool> L2;

    cout << "Testando getList1:\n";
    getList1(graph, radicalTree, L1, L2);
    for (int i = 0; i < L2.size(); i++) {
        cout << "L2[" << i << "] = " << L2[i] << endl;
    }

    // Testamos `getList2`
    cout << "\nTestando getList2:\n";
    L2.assign(5, false);
    L1.assign(5, false);
    L1[1] = true;
    getList2(graph, radicalTree, L1, L2);
    for (int i = 0; i < L2.size(); i++) {
        cout << "L2[" << i << "] = " << L2[i] << endl;
    }

    // Testamos `getList3`
    cout << "\nTestando getList3:\n";
    L2.assign(5, false);
    L1.assign(5, false);
    L1[2] = true;
    getList3(graph, radicalTree, L1, L2);
    for (int i = 0; i < L2.size(); i++) {
        cout << "L2[" << i << "] = " << L2[i] << endl;
    }
    
    
    // =====================================
    // QUESTÂO 2
    // =====================================
    GraphAdjList graph2(5);
    graph2.addEdge(0, 1);
    graph2.addEdge(0, 2);
    graph2.addEdge(1, 2);
    graph2.addEdge(1, 3);
    graph2.addEdge(2, 4);
    graph2.addEdge(3, 4);
    graph2.addEdge(4, 0);
    
    // Lista de vértices que o caminho deve visitar em ordem
    vector<int> L = {0, 3, 4, 2, 3};

    // Caminho final
    vector<int> path;

    // Testando a função shortestLPath
    cout << "Testando shortestLPath com L = {0, 3, 4, 2, 1}:" << endl;
    shortestLPath(graph2, L, path);
    
    
    return 0;
}
