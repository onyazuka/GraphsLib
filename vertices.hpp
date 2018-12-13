#pragma once
#include <cstdint>
#include <string>
#include <limits>
#include <vector>
#include <unordered_map>
#include "graphexception.hpp"

/*
    Vertex of a graph.
    Contains number(0 - ..., non negative).
*/
class Vertex
{
public:
    typedef int AttributeType;
    typedef std::string AttributeName;
    typedef size_t AttributeId;
    enum Limits {MinusInf = std::numeric_limits<AttributeType>::min(),
                      Inf = std::numeric_limits<AttributeType>::max()};
    // Number can be MAXIMUM 4 bytes unsigned!!!
    typedef int Number;
    Vertex();
    Vertex(Number num);
    Vertex(const Vertex& other);
    Vertex& operator=(const Vertex& other);
    inline Number number() const { return _number; }
    inline AttributeType getAttribute(AttributeId attrId) const { return attributes.at(attrId); }
    inline void setAttribute(AttributeId attrId, AttributeType attrVal) { attributes[attrId] = attrVal; }
    inline void clearAttributes() {attributes.clear();}
    inline void addNewAttributes(size_t n = 1) {attributes.resize(attributes.size() + n);}
private:
    typedef std::vector<AttributeType> Attributes;
    Number _number;
    Attributes attributes;
};

