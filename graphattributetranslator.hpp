#pragma once
#include "edges.hpp"

// Deletion of attributes is not needed, is it?

/*
    Helper class.
    Registers attributes by its names.
    Than we can get numeric ID to access this attribute in graph.
*/
class GraphAttributeTranslator
{
public:
    typedef std::unordered_map<Vertex::AttributeName, Vertex::AttributeId> VertexAttributeNameTranslator;
    typedef std::unordered_map<Edge::AttributeName, Edge::AttributeId> EdgeAttributeNameTranslator;
    Vertex::AttributeId addVertexAttribute(const Vertex::AttributeName& attrName);
    Edge::AttributeId addEdgeAttribute(const Edge::AttributeName& attrName);
    Vertex::AttributeId getVertexAttributeIdByName(const Vertex::AttributeName& attrName) const;
    Edge::AttributeId getEdgeAttributeIdByName(const Edge::AttributeName& attrName) const;
    bool isVertexAttributeIdRegistered(const Vertex::AttributeName& attrName) const;
    bool isEdgeAttributeIdRegistered(const Edge::AttributeName& attrName) const;
    size_t getVertexAttributesCount() const{return vertexAttrNameTranslator.size();}
    size_t getEdgeAttributesCount() const{return edgeAttrNameTranslator.size();}
private:
    VertexAttributeNameTranslator vertexAttrNameTranslator;
    EdgeAttributeNameTranslator edgeAttrNameTranslator;
};
