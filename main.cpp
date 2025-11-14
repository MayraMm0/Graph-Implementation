#include <iostream>
#include <string>
#include "Graph.h"
#include <vector>
using namespace std;

// --- HELPER METHOD print oder ---
template <typename T>
void printTopologicalOrder(const vector<T>& order) {
    if (order.empty()) {
        cout << "Ordenamiento topológico: No es posible (Ciclo detectado)." << endl;
    } else {
        cout << "Ordenamiento topológico (Kahn's):" << endl;
        for (const T& node : order) {
            cout << node << " -> ";
        }
        cout << "END" << endl;
    }
}

int main() {
    int n, m;
    cout << "Introduce el numero de vetices (n): ";
    cin >> n;
    cout << "Introduce el numero de aristas (m): ";
    cin >> m;

    Graph<int> listaAdj;              // Lista de adyacencia (interna, grafo)

    // Cargar el grafo
    listaAdj.loadGraph(n, m);

    // Mostrar la lista de adyacencia (usando el operador sobrecargado)
    cout << "\nLista de adyacencia (Graph):" << endl;
    cout << listaAdj << endl;

    // Realizar recorrido BFS
    int nodoInicial;
    cout << "Ingrese el índice inicial para hacer los recorridos (0-indexado): ";
    cin >> nodoInicial;

    cout << "Recorridos con Lista O(V+E)" << endl;
    listaAdj.dfs(nodoInicial);
    listaAdj.bfs(nodoInicial);

    cout << "Ordenamiento Topologico: " << endl;
    // Ejecutar Kahn's Algorithm
    vector<int> resultOrder = listaAdj.topologicalSort();
    printTopologicalOrder(resultOrder);

}
