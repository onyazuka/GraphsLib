#include "vertices.hpp"

// hash function for Vertex
namespace std
{
    template <>
    struct hash<Vertex>
    {
        size_t operator()(const Vertex& v) const
        {
            return hash<Vertex::Number>()(v.number());
        }
    };
}

Vertex::Vertex()
    : _number {0} {}

Vertex::Vertex(Number num)
    : _number{num}
{
    assert(_number >= 0);   // number can not be negative
}

Vertex::Vertex(const Vertex& other)
    : _number{other.number()}, attributes{other.attributes} {}

Vertex& Vertex::operator=(const Vertex& other)
{
    _number = other.number();
    attributes = other.attributes;
    return *this;
}

