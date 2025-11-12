#include <iostream>
#include <string>
#include "Graph.h"
#include <vector>
using namespace std;

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
    //vector<int> topo = listaAdj.topologicalSort();
    for (const auto& i: listaAdj.topologicalSort()){

    }

    return 0;
}
