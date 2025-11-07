#pragma once
#include <iostream>

template <typename T>
struct Node{
    T value;
    Node* left;
    Node* right;
    int height;
    Node<T>* parentNode;

    Node(T value){
        this-> value = value;
        left = nullptr;
        right = nullptr;
        height = 0;
        parentNode = nullptr;
    }

    Node<T>* get_parent() {
        return parentNode;
    }

    int get_height() {
        return height;
    }

};

template<typename Q>
std::ostream& operator<<(std::ostream& os, const Node<Q>& n){
    // Inicia con el valor del nodo y el paréntesis
    os << n.value << " (" ;

    // 1. Imprimir ParentNode:
    os << "P:"; // Etiqueta opcional para claridad
    if (!n.parentNode)
        os << "nullptr";
    else
        os << n.parentNode->value;

    os << ", L:"; // Separador y etiqueta para Left

    // 2. Imprimir Left Child:
    if (n.left == nullptr)
        os << "nullptr";
    else
        os << n.left->value;

    os << ", R:"; // Separador y etiqueta para Right

    // 3. Imprimir Right Child:
    if (n.right == nullptr)
        os << "nullptr";
    else
        os << n.right->value;

    // 4. Imprimir Altura y cerrar paréntesis
    os <<", H:" << n.height << ")";

    return os;
}

template<typename R>
std::ostream& operator<<(std::ostream& os, const Node<R>* n_ptr) {
    if (n_ptr == nullptr) return os << "nullptr";

    // 2. Si no es nulo, desreferencia el puntero (*) y llama a la primera sobrecarga
    return os << *n_ptr;
}