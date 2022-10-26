#include "node.hh"

Node::Node(int x, int y, double theta, double time, int g_value, int h_value, Node *parent)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->time = time;
    this->g_value = g_value;
    this->h_value = h_value;
    this->parent = parent;
}

Node::~Node()
{
}