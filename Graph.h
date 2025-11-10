#pragma once
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector> // Added for vector<vector<int>>
#include <queue> // Added for BFS algorithm
#include <stack> // Added for DFS algorithm
#include <cstddef> // Added for size_t

using namespace std;

template <typename T>
class Graph{
private:
    unordered_map<T, unordered_set<T>> adjList;

public:
    // --- CONSTRUCTOR ---
    Graph() = default;

    // --- INSERTION METHODS ---
    // Add vertex. This method adds a vertex to the graph, with no connections.
    // List O(1).
    bool addVertex(T vertex){
        if(!adjList.contains(vertex)){ // Avoids duplicate elements
            adjList[vertex];
            return true;
        }

        return false;
    }

    // Add edge. This method creates a connection between two existing vertices.
    //List O(1).
    bool addEdge(T vertex1, T vertex2) {
        if (adjList.contains(vertex1) && adjList.contains(vertex2)) {
            adjList.at(vertex1).insert(vertex2); // Adds vertex 2 to the set of connections of vertex 1
            adjList.at(vertex2).insert(vertex1); // Adds vertex 1 to the set of connections of vertex 2
            return true;
        }
        return false;
    }

    // --- DELETION METHODS ---
    // Remove edge. Removes an existing edge between two vertices
    // List O(1)
    bool removeEdge(T vertex1, T vertex2) {
        if (adjList.contains(vertex1) && adjList.contains(vertex2)) {
            adjList.at(vertex1).erase(vertex2);
            adjList.at(vertex2).erase(vertex1);
            return true;
        }
        return false;
    }

    // Remove vertex. Removes an existing vertex and its associated edges from the graph
    // List O(n)
    bool removeVertex(T vertex){
        if (!adjList.contains(vertex)) return false;

        // Iterate through the set of connections of the vertex we want to remove
        for(auto otherVertex: adjList[vertex]){
            // Removes the connection in the other vertex's set
            adjList[otherVertex].erase(vertex);
        }

        adjList.erase(vertex); // Removes it from list
        return true;
    }

    // --- OPERATOR OVERLOAD ---
    template <typename U>
    friend ostream& operator<<(ostream& os, const Graph<U>& graph);

    /**
     * Método que carga las aristas de un grafo y las almacena en una matriz de adyacencia y en una lista de adyacencia (objeto Graph).
     * Parámetros:
     *  - n: cantidad de vértices
     *  - m: cantidad de aristas
     *  - matrizAdj: referencia a matriz de adyacencia (vector<vector<int>>)
     *  - listaAdj: referencia a objeto Graph<int> (lista de adyacencia)
     * Precondición:
     *  n y m > 0.
     *  Se lee m veces dos enteros u, v (arista entre u y v, 0-indexados)
     */
    void loadGraph(int n, int m, vector<vector<int>>& matrizAdj, Graph<int>& listaAdj) {
        // Inicializa los vértices del grafo a partir del 0 - n-1.
        for (int i = 0; i < n; i++){
            listaAdj.addVertex(i);
        }    
        
        // Inicializa la matriz de adyacencia:
        matrizAdj.assign(n, vector<int>(n, 0));

        // Solicita al usuario relacionar los vértices que se generearon en el grafo.
        cout << "Ingrese " << m << " aristas, cada una como dos enteros (u v) separados por espacio (0-indexados):\n";
        for (int i = 0; i < m; ++i) {
            int u, v;
            cin >> u >> v;
            // Agregar a la matriz de adyacencia
            matrizAdj[u][v] = 1;
            matrizAdj[v][u] = 1; // grafo no dirigido
            // Agregar a la lista de adyacencia
            listaAdj.addEdge(u, v);
        }
    }

    /**
     * Método sobrecargado que carga las aristas de un grafo y las almacena en una matriz de adyacencia genérica
     * y en una lista de adyacencia (objeto Graph).
     * Parámetros:
     *  - n: cantidad de vértices
     *  - m: cantidad de aristas
     *  - matrizAdj: referencia a matriz de adyacencia genérica (unordered_map<T, unordered_map<T, bool>>)
     *  - listaAdj: referencia a objeto Graph<T> (lista de adyacencia)
     * Precondición:
     *  n y m > 0.
     *  Se lee m veces dos enteros u, v (arista entre u y v, 0-indexados)
     */
    template<typename U>
    void loadGraph(int n, int m, unordered_map<U, unordered_map<U, bool>>& matrizAdj, Graph<U>& listaAdj) {
        // Inicializa los vértices del grafo a partir del 0 - n-1.
        for (int i = 0; i < n; i++){
            U vertice = static_cast<U>(i);  // Convertir int a tipo U
            listaAdj.addVertex(vertice);
            // Inicializar la fila en la matriz para este vértice
            matrizAdj[vertice] = unordered_map<U, bool>();
        }
        
        // Solicita al usuario relacionar los vértices que se generaron en el grafo.
        cout << "Ingrese " << m << " aristas, cada una como dos enteros (u v) separados por espacio (0-indexados):\n";
        for (int i = 0; i < m; ++i) {
            int u, v;
            cin >> u >> v;
            U verticeU = static_cast<U>(u);
            U verticeV = static_cast<U>(v);
            
            // Agregar a la matriz de adyacencia genérica
            matrizAdj[verticeU][verticeV] = true;
            matrizAdj[verticeV][verticeU] = true; // grafo no dirigido
            
            // Agregar a la lista de adyacencia
            listaAdj.addEdge(verticeU, verticeV);
        }
    }


    void BFS(vector<vector<int>>& matrizAdj, int nodoInicial) {
        int n = matrizAdj.size();
        vector<bool> visitados(n, false);
        queue<int> cola;
        
        if (nodoInicial < 0 || nodoInicial >= n) {
            cout << "Error: El nodo inicial " << nodoInicial << " no es válido." << endl;
            return;
        }
        
        visitados[nodoInicial] = true;
        cola.push(nodoInicial);
        cout << "Recorrido BFS desde el nodo " << nodoInicial << ": ";
        
        while (!cola.empty()) {
            int nodoActual = cola.front();
            cola.pop();
            cout << nodoActual;
            
            for (int vecino = 0; vecino < n; vecino++) {
                if (matrizAdj[nodoActual][vecino] == 1 && !visitados[vecino]) {
                    visitados[vecino] = true;
                    cola.push(vecino);
                }
            }
            
            if (!cola.empty()) {
                cout << " ";
            }
        }
        cout << endl;
    }


    void DFS(const vector<vector<int>>& matrizAdj, int nodoInicial) {
        int n = matrizAdj.size();
        vector<bool> visitados(n, false);
        stack<int> pila;
        
        if (nodoInicial < 0 || nodoInicial >= n) {
            cout << "Error: El nodo inicial " << nodoInicial << " no es válido." << endl;
            return;
        }
        
        cout << "Recorrido DFS desde el nodo " << nodoInicial << ": ";
        pila.push(nodoInicial);
        bool esPrimerNodo = true;
        
        while (!pila.empty()) {
            int nodoActual = pila.top();
            pila.pop();
            
            if (!visitados[nodoActual]) {
                visitados[nodoActual] = true;
                
                if (!esPrimerNodo) {
                    cout << " ";
                }
                esPrimerNodo = false;
                cout << nodoActual;
                
                for (int vecino = n - 1; vecino >= 0; vecino--) {
                    if (matrizAdj[nodoActual][vecino] == 1 && !visitados[vecino]) {
                        pila.push(vecino);
                    }
                }
            }
        }
        cout << endl;
    }



    void bfs(const T& startNode) {
        if (!adjList.contains(startNode)) {
            cout << "Error: Start node not found in graph." << endl;
            return;
        }

        // 1. Initialize a Queue for nodes to visit and a Set for visited nodes
        queue<T> Q;
        unordered_set<T> visited;

        // 2. Start at the node passed in
        Q.push(startNode);
        visited.insert(startNode);

        cout << "Recorrido bfs desde el nodo " << startNode << ": ";

        while (!Q.empty()) {
            // 3. Dequeue the current node (FIFO)
            T current = Q.front();
            Q.pop();

            // 4. Print current node
            cout << current << " ";

            // 5. Look at all neighbors of the current node
            for (const T& neighbor : adjList.at(current)) {
                // 6. If the neighbor hasn't been visited, mark it and enqueue it
                if (!visited.contains(neighbor)) {
                    visited.insert(neighbor);
                    Q.push(neighbor);
                }
            }
        }
        cout << endl;
    }


    void dfs(const T& startNode) const {
        if (!adjList.contains(startNode)) {
            cout << "Error: Start node not found in graph." << endl;
            return;
        }

        // 1. Initialize a Stack for nodes to visit and a Set for visited nodes
        stack<T> S;
        unordered_set<T> visited;

        // 2. Start at the node passed as parameter
        S.push(startNode);
        visited.insert(startNode);

        cout << "Recorrido dfs desde el nodo " << startNode << ": ";

        while (!S.empty()) {
            // 3. Pop the current node (LIFO)
            T current = S.top();
            S.pop();

            // 4. Process the current node (in this case, print it)
            cout << current << " ";

            // 5. Look at all neighbors of the current node
            for (const T& neighbor : adjList.at(current)) {
                // 6. If the neighbor hasn't been visited, mark it and push it
                if (!visited.contains(neighbor)) {
                    visited.insert(neighbor);
                    S.push(neighbor);
                }
            }
        }
        cout << endl;
    }
};

// --- OPERATOR OVERLOAD IMPLEMENTATION ---
template <typename T>
ostream& operator<<(ostream& os, const Graph<T>& graph) {
    // Iterate over the adjacency list (key-value pairs: node -> set of neighbors)
    auto kvPair = graph.adjList.begin();

    while (kvPair != graph.adjList.end()) {
        // Output the current node (key)
        os << "* " << kvPair->first << " : [";

        // Iterate over the set of neighbors (edges)
        auto edge = kvPair->second.begin();
        bool first = true;

        while (edge != kvPair->second.end()) {
            if (!first) {
                os << ", ";
            }
            // Output the neighbor (edge)
            os << *edge;
            ++edge;
            first = false;
        }

        os << "]" << endl;
        ++kvPair;
    }

    return os;
}
