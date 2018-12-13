#include "graphattributetranslator.hpp"

/*
    id is simple integer, that increments with each new attribute
*/
Vertex::AttributeId GraphAttributeTranslator::addVertexAttribute(const Vertex::AttributeName &attrName)
{
    Vertex::AttributeId newId = vertexAttrNameTranslator.size();
    vertexAttrNameTranslator[attrName] = newId;
    return newId;
}

/*
    id is simple integer, that increments with each new attribute
*/
Edge::AttributeId GraphAttributeTranslator::addEdgeAttribute(const Edge::AttributeName &attrName)
{
    Edge::AttributeId newId = edgeAttrNameTranslator.size();
    edgeAttrNameTranslator[attrName] = newId;
    return newId;
}

Vertex::AttributeId GraphAttributeTranslator::getVertexAttributeIdByName(const Vertex::AttributeName& attrName) const
{
    return vertexAttrNameTranslator.at(attrName);
}

Edge::AttributeId GraphAttributeTranslator::getEdgeAttributeIdByName(const Edge::AttributeName& attrName) const
{
    return edgeAttrNameTranslator.at(attrName);
}

bool GraphAttributeTranslator::isVertexAttributeIdRegistered(const Vertex::AttributeName& attrName) const
{
    auto findIter = vertexAttrNameTranslator.find(attrName);
    if(findIter == vertexAttrNameTranslator.end())
    {
        return false;
    }
    return true;
}

bool GraphAttributeTranslator::isEdgeAttributeIdRegistered(const Edge::AttributeName& attrName) const
{
    auto findIter = edgeAttrNameTranslator.find(attrName);
    if(findIter == edgeAttrNameTranslator.end())
    {
        return false;
    }
    return true;
}
