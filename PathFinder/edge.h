#ifndef EDGE_H
#define EDGE_H

#include "node.h"


class Edge
{
private:
    Node* m_start;
    Node* m_finish;
public:
    Edge();
    Edge(Node* nodeStart, Node* nodeFinish);
    Edge(Edge* edge);
};

#endif // EDGE_H
