#pragma once
#include <fstream>
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
            for (const T& neighbor : adjList.at(current)){
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

    // From file .txt
    bool loadGraph(istream& in){
        int n, m;   //  Número de vétices y arcos.

        //  Valída de que el archivo cuente con el formato correcto.
        if (!(in >> n >> m)){
            cout << "Formato invalido";
            return false;
        }

        //  Limpia cualquier grafo previo.
        adjList.clear();

        //  Crea el número de vértices solcitados a partir de 0 a n-1.
        for(int i = 0; i < n; i++){
            addVertex(static_cast<T>(i));
        }

        //  Lee los 'm' arcos solicitados y realiza la unión.
        for(int i = 0; i < m; i++){
            int u, v;
            in >> u >> v;   //  Lee el par de vérticea.
            addEdge(u,v);   //  Realiza el arco con ambos vértices.
        }

        return true;
    }



    // From input: n (number of vertices) and m (number of edges).
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

    // --- METODO PARA VERIFICAR SI ES ÁRBOL --- O(V + E)
    // Criterio 1: Debe tener una única raíz (Un nodo con inDegree de 0)
    // Criterio 2: Debe tener V-1 aristas
    // Criterio 2: Todos los demás nodos pueden ser accedidos desde la raíz
    // Critetio 3: Debe ser DAG (sin ciclos)
    bool isTree() const{
        //  1. Calculate inDegree of every node
        unordered_map<T,int> inDegree = calculateInDegree();

        //  2. Search for non dependent nodes (degree : 0)
        T root;
        int rootCount = 0;
        for(const auto& pair : inDegree){
            if(pair.second == 0){
                root = pair.first;
                rootCount++;
            }
        }

        //  3. Check there is a unique root
        if(rootCount != 1){
            return false;
        }

        // 4. Check connectivity with BFS from root
        unordered_set<T> visited;
        queue<T> Q;

        Q.push(root);
        visited.insert(root);

        while(!Q.empty()){
            T current = Q.front();
            Q.pop();

            // Recorre los vecinos.
            for(const T& neighboor : adjList.at(current)){
                if(!visited.contains(neighboor)){
                    visited.insert(neighboor);
                    Q.push(neighboor);
                }
            }
        }

        //  4. Check all nodes are accesible from root
        if(topologicalSort().empty()){
            return false;
        }

        return true;
    }


    // --- BIPARTITE CHECK (TWO-COLORING ALGORITHM) ---
    // Un grafo es bipartito si no contiene ciclos primos
    // Se maneja asignando "colores" o sets
    // O(V + E)
    bool isBipartite() const {
        // 1. Initialize a map to store the color/set ID for each vertex.
        // 0: Uncolored/Unvisited, 1: Set U, 2: Set V
        unordered_map<T, int> color;
        for (const auto& pair : adjList) {
            color[pair.first] = 0; // Initialize all vertices as uncolored
        }

        // 2. Use BFS approach, but we must iterate through all vertices to handle potentially disconnected components.
        for (const auto& startPair : adjList) {
            T startNode = startPair.first;

            // If the component has not yet been colored, start a new BFS/coloring process.
            if (color[startNode] == 0) {
                queue<T> Q;
                Q.push(startNode);
                color[startNode] = 1; // Start coloring the current component with Set U (color 1)

                while (!Q.empty()) {
                    T u = Q.front();
                    Q.pop();

                    // 3. Check all neighbors (v) of the current node (u).
                    for (const T& v : adjList.at(u)) {
                        // If neighbor (v) is uncolored, color it with the opposite color and enqueue.
                        if (color[v] == 0) {
                            // Si u es de color 1, asigna a v 2. Si u NO es de color 1, asigna 1
                            color[v] = (color[u] == 1) ? 2 : 1;
                            Q.push(v);
                        }
                        // If neighbor (v) is already colored and has the SAME color as u,
                        // an odd cycle exists
                        else if (color[v] == color[u]) {
                            return false; // Not Bipartite
                        }
                    }
                }
            }
        }

        // 4. If the coloring process completes without conflicts, the graph is bipartite.
        return true;
    }

    // --- LOAD FROM FILE ---
    bool loadFromFile(const string& filename){
        ifstream file(filename);    //  Se abre el archivo.

        //  Valida si el archivo se pudo abrir.
        if(!file.is_open()){
            cerr << "No se puedo abrir el archivo.";
            return false;
        }

        //  Manda a llamar a 'loadGraph' para crear el grafo con la información del archivo.
        return loadGraph(file);
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
