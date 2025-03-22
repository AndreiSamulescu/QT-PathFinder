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

    double getX();
    double getY();

    void setX(double x);
    void setY(double y);
};

#endif // NODE_H
