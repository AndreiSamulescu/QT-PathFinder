#ifndef NODE_H
#define NODE_H

class Node
{
private:
    double m_x;
    double m_y;
public:
    Node();
    Node(double x, double y);
    Node(const Node& otherNode);
};

#endif // NODE_H
