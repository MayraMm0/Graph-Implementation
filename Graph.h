#pragma once
#include <iostream>
#include <unordered_map>
#include <unordered_set>

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
