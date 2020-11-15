#ifndef TREE_H
#define TREE_H

//#include "rw/rw.hpp"
#include <rw/math.hpp>
#include <rw/loaders/image/PGMLoader.hpp>
#include <rw/sensor/Image.hpp>

#include <iostream>
#include <vector>
#include <math.h>

using namespace std;
using namespace rw::math;


struct Node
{
    Node(): parent(NULL){}
    Node(Q q): _q(q){}

    Q _q;
    Node* parent;
};

class Tree
{
public:
    Tree();
    Tree(Node* _root);
    ~Tree();
    Node* find_nearest_neighbor(Node n);
    void add_node(Node* n);
    void add_edge(Node* parent, Node* child);
    void print_route(Node* leaf);

    void visualize(string filename, int dof1, int dof2);
    vector<Node*> get_nodes();
    Node* get_root();
   // void obliterateTree();



private:



    vector<Node*> nodes;
    Node* root;


};

#endif // TREE_H
