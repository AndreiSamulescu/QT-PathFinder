#include "node.h"

Node::Node() {}
Node::Node(double x, double y) : m_x(x), m_y(y){}
Node::Node(const Node& otherNode) : m_x(otherNode.m_x), m_y(otherNode.m_y){}

double Node::getX() {
    return this->m_x;
}

double Node::getY() {
    return this->m_y;
}

void Node::setX(double x) {
    this->m_x = x;
}

void Node::setY(double y) {
    this->m_y = y;
}
