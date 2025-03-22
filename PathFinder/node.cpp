#include "node.h"

Node::Node() {}
Node::Node(double x, double y) : m_x(x), m_y(y){}
Node::Node(const Node& otherNode) : m_x(otherNode.m_x), m_y(otherNode.m_y){}
