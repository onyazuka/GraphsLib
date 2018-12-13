#pragma once
#include "vertices.hpp"

typedef unsigned long long ull;
typedef ull Cantor;

/*
    Function for uniting two numbers.
*/
template<typename T>
ull cantorFunction(T v1, T v2)
{
    return (v1 + v2) * (v1 + v2 + 1) / 2 + v2;
}

class Edge
{
public:
    typedef int AttributeType;
    typedef std::string AttributeName;
    typedef size_t AttributeId;
    enum Limits {MinusInf = std::numeric_limits<AttributeType>::min(),
                      Inf = std::numeric_limits<AttributeType>::max()};
    Edge();
    Edge(Vertex* f, Vertex* t, bool _directed = false);
    // needed to forbid this to exclude implicit conversion
    Edge(Vertex::Number f, Vertex::Number t, bool _directed = false) = delete;
    Edge(const Edge& other);
    Edge& operator=(const Edge& other);
    inline Vertex& v1() const {return *_v1;}
    inline Vertex& v2() const {return *_v2;}
    inline bool isDirected() const {return directed;}
    inline AttributeType getAttribute(AttributeId attrId) const { return attributes.at(attrId); }
    inline void setAttribute(AttributeId attrId, AttributeType attrVal) { attributes[attrId] = attrVal; }
    inline void clearAttributes() {attributes.clear();}
    inline void addNewAttributes(size_t n = 1) {attributes.resize(attributes.size() + n);}
private:
    typedef std::vector<AttributeType> Attributes;
    Vertex* _v1;
    Vertex* _v2;
    bool directed;
    Attributes attributes;
};

// hash function for Edge
namespace std
{
    template <>
    struct hash<Edge>
    {
        size_t operator()(const Edge& e) const
        {
            ull cantor = cantorFunction(e.v1().number(), e.v2().number());
            return hash<ull>()(cantor);
        }
    };
}
