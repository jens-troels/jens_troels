#include "tree.h"


Tree::Tree()
{

}

Tree::Tree(Node* _root) {
    root=_root;
    nodes.push_back(_root);
    _root->parent=NULL;
}

Tree::~Tree()
{
    //obliterateTree();
}

Node* Tree::find_nearest_neighbor(Node n)
{
    double min_dist = std::numeric_limits<double>::infinity();
    int index=std::numeric_limits<int>::infinity();
    for (int i=0; i< nodes.size(); i++){
        double euclid_dist= sqrt(pow(n._q[0]-nodes.at(i)->_q[0],2)+
                                 pow(n._q[1]-nodes.at(i)->_q[1],2)+
                                 pow(n._q[2]-nodes.at(i)->_q[2],2)+
                                 pow(n._q[3]-nodes.at(i)->_q[3],2)+
                                 pow(n._q[4]-nodes.at(i)->_q[4],2)+
                                 pow(n._q[5]-nodes.at(i)->_q[5],2));
        if(min_dist>euclid_dist){
            min_dist=euclid_dist;
            index=i;
        }
    }
    return nodes.at(index);
}

void Tree::add_node(Node* n)
{
    nodes.push_back(n);
}

void Tree::add_edge(Node* parent, Node* child)
{
    child->parent=parent;
}

void Tree::print_route(Node *leaf) {
    if(leaf->parent != NULL)
    {
        print_route(leaf->parent);
    }
    std::cout << leaf->_q << std::endl;
}

void Tree::visualize(string filename, int dof1, int dof2)
{
    rw::sensor::Image::Ptr canvas = new rw::sensor::Image;
    canvas->resize(720,720);

    //Paint entire canvas white:
    for(int i = 0; i < canvas->getWidth(); i++)
        for(int j = 0; j < canvas->getHeight(); j++)
            canvas->setPixel8U(i,j,255);

    for(int i = 0; i < nodes.size(); i++)
    {
        double q1 = nodes[i]->_q[dof1];
        double q2 = nodes[i]->_q[dof2];

        //Coordinate system is (-360,-360) at bottom left -> (360,360) top right. This assumes constraints of [-360;360] for every joint
        int x = -q1 * 360 / (2 * M_PI) + 360;
        int y = -q2 * 360 / (2 * M_PI) + 360;
        canvas->setPixel8U(x,y,0);
    }

    if( canvas->saveAsPGM(filename) )
        cout << "saved tree visualization to file: " << filename << endl;
    else
        cout << "failed to save file" << endl;

}

vector<Node*> Tree::get_nodes()
{
    return nodes;
}

Node* Tree::get_root()
{
    return root;
}

//void Tree::obliterateTree()
//{
//    for(int i = 0; i < nodes.size(); i++)
//    {
//        delete nodes.at(i);
//    }
//}





