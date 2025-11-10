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

    vector<vector<int>> matrizAdj;      // Matriz de adyacencia
    Graph<int> listaAdj;                // Grafo (lista de adyacencia)

    // Cargar el grafo (esto solicita al usuario los m pares de nodos)
    // Puedes llamar al método como es ahora no siendo estático:
    listaAdj.loadGraph(n, m, matrizAdj);

    // Mostrar la matriz de adyacencia
    cout << "Matriz de adyacencia:" << endl;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            cout << matrizAdj[i][j] << " ";
        }
        cout << endl;
    }

    // Mostrar la lista de adyacencia (usando el operador sobrecargado)
    cout << "\nLista de adyacencia (Graph):" << endl;
    cout << listaAdj << endl;

    // Realizar recorrido BFS
    int nodoInicial;
    cout << "Ingrese el nodo inicial para hacer los recorridos (0-indexado): ";
    cin >> nodoInicial;
    
    cout << "\n";
    cout << "Matriz de adyacencia:" << endl;
    listaAdj.BFS(matrizAdj, nodoInicial);
    listaAdj.DFS(matrizAdj, nodoInicial);
    cout << "Lista de adyacencia:" << endl;
    listaAdj.dfs(nodoInicial);
    listaAdj.bfs(nodoInicial);


    return 0;
}
