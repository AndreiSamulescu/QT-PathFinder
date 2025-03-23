#include "edge.h"

Edge::Edge() {}
Edge::Edge(Node* start, Node* finish): m_start(start), m_finish(finish){}
Edge::Edge(Edge* edge): m_start(edge->m_start), m_finish(edge->m_finish) {}


