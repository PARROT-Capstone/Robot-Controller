#ifndef NODE_HH
#define NODE_HH

#include <stdlib.h>
#include <string>
#include <algorithm>
#include <constants.hh>

class Node
{
public:
    int x;
    int y;
    double theta;
    double time;
    double g_value;
    double h_value;
    Node *parent;

    Node(int x, int y, double theta, double time, double g_value, double h_value, Node *parent);
    ~Node();
};

/**
 * @brief Sorts the nodes by f-value for use in the priority queue
 *
 */
struct OpenListNodeComparator
{
    bool operator()(const Node *n1, const Node *n2) const
    {
        // NOTE Weight is 100t, but can be changed if we decided to do weighted A*
        return G_VALUE_WEIGHT * n1->g_value + H_VALUE_WEIGHT * n1->h_value > G_VALUE_WEIGHT * n2->g_value + H_VALUE_WEIGHT * n2->h_value;
    }
};

/**
 * @brief used as the comparator for nodes in the set
 *
 */
struct ClosedListNodeComparator
{
    bool operator()(const Node *n1, const Node *n2) const
    {
        // reuturn true if both nodes have the same x, y, theta, and time
        return n1->x == n2->x && n1->y == n2->y && n1->theta == n2->theta && n1->time == n2->time;
    }
};

/**
 * @brief Used to store all of the nodes in the closed list set and allow for O(1) lookup
 *
 */
struct NodeHasher
{
    size_t operator()(const Node *n) const
    {
        // concat x, y, theta, and time into a string and hash it
        std::string s = std::to_string(n->x) + std::to_string(n->y) + std::to_string(n->theta) + std::to_string(n->time);
        return std::hash<std::string>{}(s);
    }
};

#endif /* NODE_HH */