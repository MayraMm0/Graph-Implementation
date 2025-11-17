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
        for (const T& node : order) {
            cout << node << " ";
        }
        cout << endl;
    }
}

int main() {
    Graph<int> grafo1;
    Graph<int> grafo2;
    Graph<int> grafo3;
    Graph<int> grafo4;
    // Load graphs from files first, then collect them into the vector
    grafo1.loadFromFile("Grafo1.txt");
    grafo2.loadFromFile("Grafo2.txt");
    grafo3.loadFromFile("Grafo3.txt");
    grafo4.loadFromFile("Grafo4.txt");

    vector<Graph<int>> grafos = {grafo1, grafo2, grafo3, grafo4};

    cout << "\n=== GRAFOS CARGADOS ===" << endl;

    // iterate over all graphs in the vector
    for (size_t i = 0; i < grafos.size(); ++i) {
        cout << "\n=== GRAFO "<< i+1 << " ===" << endl;

        cout << "\n- Lista de adjacencia -" << endl;
        cout << grafos[i] << endl;

        cout << "\n- Orden Topológico -" << endl;
        vector<int> orden = grafos[i].topologicalSort();
        printTopologicalOrder(orden);
        cout << endl;

        cout << "- Verificaciones -" << endl;
        if(grafos[i].isTree() == true){
            cout << "Es un árbol." << endl;
        }else{
            cout << "No es un árbol." << endl;
        }
        if(grafos[i].isBipartite() == true){
            cout << "Es bipartito." << endl;
        }else{
            cout << "No es bipartito." << endl;
        }

        cout << "\n";
    }

    return 0;

}
