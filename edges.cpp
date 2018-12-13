#include "edges.hpp"

Edge::Edge()
 : _v1{0}, _v2{0}, directed{false} {}

// accessing by pointers so we can have attributes etc
Edge::Edge(Vertex* f, Vertex* t, bool _directed)
    : _v1{f}, _v2{t}, directed{_directed}{}

 Edge::Edge(const Edge& other)
     : _v1{&(other.v1())}, _v2{&(other.v2())}, directed{other.isDirected()}, attributes{other.attributes}
 {}

Edge& Edge::operator=(const Edge& other)
{
    _v1 = &(other.v1());
    _v2 = &(other.v2());
    directed = other.isDirected();
    attributes = other.attributes;
    return *this;
}
