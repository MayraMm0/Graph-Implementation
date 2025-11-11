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
    // Adjacency list representation
    unordered_map<T, unordered_set<T>> adjList;

    //Auxiliary structures for Matrix indexing
    // Maps T type to a fixed int index (id)
    unordered_map<T, int> vertexToId;
    // Maps int index back to T type
    vector<T> idToVertex;

public:
    // --- CONSTRUCTOR ---
    Graph() = default;

    // --- INSERTION METHODS ---
    // Add vertex. This method adds a vertex to the graph, with no connections.
    // It also assigns it an id for matrix use
    bool addVertex(T vertex){
        if(!adjList.contains(vertex)){ // Avoids duplicate elements
            adjList[vertex];

            int newId = idToVertex.size();
            vertexToId[vertex] = newId;
            idToVertex.push_back(vertex);
            return true;
        }

        return false;
    }

    // Add edge. This method creates a connection between two existing vertices.
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
    bool removeVertex(T vertex) {
        if (!adjList.contains(vertex)) return false;

        // Iterate through the set of connections of the vertex we want to remove
        for(auto otherVertex: adjList[vertex]){
            // Removes the connection in the other vertex's set
            adjList[otherVertex].erase(vertex);
        }

        adjList.erase(vertex); // Removes it from list only
        return true;
    }

    // --- TRAVERSAL METHODS ---
    // Breadth First Search (Level by Level)

    // Adjacency List bfs O(V+E)
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

        cout << "BFS list traversal starting from " << startNode << ": ";

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

    // Adjacency Matrix bfs O(V^2)
    void bfs_matrix(const vector<vector<int>>& matrizAdj, int startId) {
        int n = matrizAdj.size();
        vector<bool> visitados(n, false);
        queue<int> cola;

        if (startId < 0 || startId >= n) {
            cout << "Error: El nodo inicial " << startId << " no es válido." << endl;
            return;
        }

        visitados[startId] = true;
        cola.push(startId);
        cout << "Recorrido BFS desde el nodo " << idToVertex.at(startId) << ": ";

        while (!cola.empty()) {
            int currentId = cola.front();
            cola.pop();
            cout << idToVertex.at(currentId) << " ";

            for (int vecino = 0; vecino < n; vecino++) {
                if (matrizAdj[currentId][vecino] == 1 && !visitados[vecino]) {
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

    // Depth-First Search (Complete paths)

    // Adjacency List dfs O(V+E)
    void dfs(const T& startNode) {
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

        cout << "DFS list traversal starting from " << startNode << ": ";

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

    // Adjacency Matrix dfs O(V^2)
    void dfs_matrix(const vector<vector<int>>& matrizAdj, int startId) {
        int n = matrizAdj.size();
        vector<bool> visitados(n, false);
        stack<int> pila;

        if (startId < 0 || startId >= n) {
            cout << "Error: El nodo inicial " << startId << " no es válido." << endl;
            return;
        }

        cout << "Recorrido DFS desde el nodo " << startId << ": ";
        pila.push(startId);
        bool esPrimerNodo = true;

        while (!pila.empty()) {
            int currentId = pila.top();
            pila.pop();

            if (!visitados[currentId]) {
                visitados[currentId] = true;

                if (!esPrimerNodo) {
                    cout << " ";
                }
                esPrimerNodo = false;
                cout << idToVertex.at(currentId) << " ";

                for (int vecino = n - 1; vecino >= 0; vecino--) {
                    if (matrizAdj[currentId][vecino] == 1 && !visitados[vecino]) {
                        pila.push(vecino);
                    }
                }
            }
        }
        cout << endl;
    }

    // --- LOAD METHOD ---
    // Loads the graph edges into the Adjacency Matrix (external ref)
    // and the Adjacency List (internal member).
    // Input: n (number of vertices), m (number of edges), matrizAdj (matrix by ref)
    void loadGraph(int n, int m, vector<vector<int>>& matrizAdj) {
        // Clear previous data
        adjList.clear();
        idToVertex.clear();
        vertexToId.clear();

        // 1. Initialize external Adj Matrix
        matrizAdj.assign(n, vector<int>(n, 0));

        for (int i = 0; i < n; ++i) {
            addVertex(static_cast<T>(i)); // This also updates the internal ID maps
        }

        cout << "Ingrese " << m << " aristas, cada una como dos enteros (u v) separados por espacio (0-indexados):\n";
        for (int i = 0; i < m; ++i) {
            int u_id, v_id;
            cin >> u_id >> v_id;

            if (u_id < 0 || u_id >= n || v_id < 0 || v_id >= n) {
                cout << "Error: arista inválida (" << u_id << " " << v_id << ") ignorada." << endl;
                continue;
            }

            // Get T values corresponding to the read IDs
            T u_vertex = idToVertex.at(u_id);
            T v_vertex = idToVertex.at(v_id);

            // Update Adj Matrix (external)
            matrizAdj[u_id][v_id] = 1;
            matrizAdj[v_id][u_id] = 1;

            // Update adj list (internal)
            addEdge(u_vertex, v_vertex);
        }
    }


    // --- OPERATOR OVERLOAD ---
    template <typename U>
    friend ostream& operator<<(ostream& os, const Graph<U>& graph);
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
