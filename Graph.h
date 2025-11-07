#pragma once
#include "Node.h"
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using namespace std;


class Graph{
private:
    unordered_map<string, unordered_set<string>> adjList;

public:
    void printGraph() {
        unordered_map<string, unordered_set<string>>::iterator kvPair = adjList.begin();
        while (kvPair != adjList.end()) {
            cout << kvPair->first << ": [";
            unordered_set<string>::iterator edge = kvPair->second.begin();
            bool first = true;
            while (edge != kvPair->second.end()) {
                if (!first) {
                    cout << ", ";
                }
                cout << *edge;
                edge++;
                first = false;
            }
            cout << "]" << endl;
            kvPair++;
        }
    }
};

