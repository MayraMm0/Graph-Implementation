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

    vector<vector<int>> matrizAdj;    // Matriz de adyacencia (externa)
    Graph<int> listaAdj;              // Lista de adyacencia (interna, grafo)

    // Cargar el grafo (esto solicita al usuario los m pares de nodos)
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
    cout << "Ingrese el índice inicial para hacer los recorridos (0-indexado): ";
    cin >> nodoInicial;
    
    cout << "\n";
    cout << "Recorridos con Matríz O(V^2)" << endl;
    listaAdj.bfs_matrix(matrizAdj, nodoInicial);
    listaAdj.dfs_matrix(matrizAdj, nodoInicial);
    cout << "Recorridos con Lista O(V+E)" << endl;
    listaAdj.dfs(nodoInicial);
    listaAdj.bfs(nodoInicial);


    return 0;
}
