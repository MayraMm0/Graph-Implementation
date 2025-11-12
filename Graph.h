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
    // Adjacency list representation (Directed)
    unordered_map<T, unordered_set<T>> adjList;

    // --- HELPER METHOD FOR COUNTING IN-DEGREE ---
    // Calculates the in-degree (number of incoming edges) for every node.
    // Time Complexity: O(V + E)
    unordered_map<T, int> calculateInDegree() const {
        unordered_map<T, int> inDegree;

        // 1. Initialize in-degree for all vertices to zero.
        for (const auto& pair : adjList) {
            inDegree[pair.first] = 0;
        }

        // 2. Iterate through every edge (u -> v) to count incoming edges to v.
        for (const auto& pair : adjList) {
            // 'pair.first' is the source (u)
            // 'pair.second' is the set of neighbors (destinations v)
            for (const T& neighbor : pair.second) {
                // Increment degree
                ++inDegree[neighbor];
            }
        }
        return inDegree;
    }


public:
    // --- CONSTRUCTOR ---
    Graph() = default;

    // --- INSERTION METHODS ---
    // Add vertex. This method adds a vertex to the graph, with no connections.
    bool addVertex(T vertex){
        if(!adjList.contains(vertex)){ // Avoids duplicate elements
            adjList[vertex];
            return true;
        }
        return false;
    }

    // Add edge. This method creates a connection between two existing vertices.
    // THE GRAPH IS DIRECTED "vertex1 -> vertex2"
    bool addEdge(T vertex1, T vertex2) {
        if (adjList.contains(vertex1) && adjList.contains(vertex2)) {
            adjList.at(vertex1).insert(vertex2); // Adds vertex 2 to the set of connections of vertex 1
            return true;
        }
        return false;
    }

    // --- DELETION METHODS ---
    // Remove edge. Removes an existing edge from vertex1 -> vertex2
    // List O(1)
    bool removeEdge(T vertex1, T vertex2) {
        if (adjList.contains(vertex1) && adjList.contains(vertex2)) {
            adjList.at(vertex1).erase(vertex2);
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
            cout << "Error: Nodo no encontrado." << endl;
            return;
        }

        // 1. Initialize a Queue for nodes to visit and a Set for visited nodes
        queue<T> Q;
        unordered_set<T> visited;

        // 2. Start at the node passed in
        Q.push(startNode);
        visited.insert(startNode);

        cout << "BFS traversal empezando de " << startNode << ": ";

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

    // Depth-First Search (Complete paths)
    // Adjacency List dfs O(V+E)
    void dfs(const T& startNode) {
        if (!adjList.contains(startNode)) {
            cout << "Error: Nodo no encontrado." << endl;
            return;
        }

        // 1. Initialize a Stack for nodes to visit and a Set for visited nodes
        stack<T> S;
        unordered_set<T> visited;

        // 2. Start at the node passed as parameter
        S.push(startNode);
        visited.insert(startNode);

        cout << "DFS traversal empezando de " << startNode << ": ";

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

    // --- TOPOLOGICAL SORT (KAHN'S ALGORITHM) ---
    // O(V + E)
    vector<T> topologicalSort() const {
        // 1. Calculate in-degree for all vertices (using helper method)
        unordered_map<T, int> inDegree = calculateInDegree();

        // 2. Initialize Queue and result vector
        queue<T> Q;
        vector<T> topologicalOrder;

        // 3. Final all vertices with an in-degree of 0 and add them to queue
        for (const auto& pair : inDegree){
            if (pair.second == 0){
                Q.push(pair.first);
            }
        }

        // 4. Process the queue until empty
        while (!Q.empty()){
            // Dequeue current
            T current = Q.front();
            Q.pop();

            // Add current to topological order list
            topologicalOrder.push_back(current);

            // 5. Iterate through all neighbors of current
            for (const T& neighbor : adjList[current]){
                // Decrement the in-degree of neighbor
                --inDegree[neighbor];

                // 6. If neighbors in-degree is now 0, enqueue
                if (inDegree[neighbor] == 0){
                    Q.push(neighbor);
                }
            }
        }

        // 7. Check if graph is not DAG
        // If num of nodes in topological order is less than total
        // num of nodes in graph -> there is a cycle (not DAG)
        if (topologicalOrder.size() != adjList.size()){
            cerr << "Error: El grafo contiene un ciclo (No es DAG). " << endl;
            return{};
        }

        return topologicalOrder;
    }

    // --- LOAD METHOD ---
    // Loads the graph edges into the Adjacency List (internal member).
    // Input: n (number of vertices) and m (number of edges).
    void loadGraph(int n, int m) {
        // Clear previous data
        adjList.clear();

        cout << "Ingresa " << n << " vertices, separados por espacios:\n";
        // 1. Read N vertices and add them to the graph.
        for (int i = 0; i < n; ++i) {
            T vertex;
            if (!(cin >> vertex)) {
                cerr << "Error leyendo input para vertices." << endl;
                return;
            }
            addVertex(vertex);
        }

        cout << "Ingresa " << m << " aristas, como dos vertices (u v) de tipo T:\n";
        // 2. Read M edges and add them to the graph.
        for (int i = 0; i < m; ++i) {
            T u_vertex, v_vertex;
            if (!(cin >> u_vertex >> v_vertex)) {
                cerr << "Error leyendo input para aristas." << endl;
                return;
            }

            // The addEdge method handles checking if vertices exist.
            if (!addEdge(u_vertex, v_vertex)) {
                // Inform the user if the vertices weren't part of the initial N.
                cout << "Advertencia: Arista (" << u_vertex << ", " << v_vertex << ") ignorada" << endl;
            }
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
