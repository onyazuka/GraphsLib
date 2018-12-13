#pragma once
#include <unordered_map>
#include "interfaces.hpp"
#include "graphattributetranslator.hpp"

class Graph
{
public:
    typedef std::vector<Vertex> Vertices;
    typedef std::vector<Vertex::Number> VertexNumbers;
    typedef std::unordered_map<Cantor, Edge> Edges;

    enum class Interfaces {AdjacencyLists, AdjacencyMatrix};
    Graph(Vertex::Number verticesCount, Interfaces interface = Interfaces::AdjacencyLists, bool _directed = false);
    Graph(const Graph& other);
    Graph& operator=(const Graph& other);
    ~Graph();
    Vertex& getVertexByNumber(Vertex::Number number);
    bool areConnected(Vertex::Number v1, Vertex::Number v2);
    virtual Edge& getEdgeByVertices(Vertex::Number, Vertex::Number) = 0;
    void addVertex(Vertex::Number count = 1);
    void addEdgeBetween(Vertex::Number, Vertex::Number);
    void deleteEdgeBetween(Vertex::Number, Vertex::Number);
    void deleteEdge(Edge edge);
    Vertex::Number vertexCount() const {return vertices.size();}
    ull edgesCount() const {return edges.size();}
    bool isDirected() const {return directed;}
    void transpose();
    VertexNumbers getAdjacentVerticesFor(Vertex::Number v);
    template<typename Container>
    void getAllEdges(Container& cont);
    inline Interfaces getInterfaceType() const {return interfaceType;}
    void clearAttributes();
    Vertex::AttributeId addVertexAttribute(const Vertex::AttributeName& attrName);
    Edge::AttributeId addEdgeAttribute(const Edge::AttributeName& attrName);
    Vertex::AttributeId getVertexAttributeIdByName(const Vertex::AttributeName& attrName) const;
    Edge::AttributeId getEdgeAttributeIdByName(const Edge::AttributeName& attrName) const;
    bool isVertexAttributeIdRegistered(const Vertex::AttributeName& attrName) const;
    bool isEdgeAttributeIdRegistered(const Edge::AttributeName& attrName) const;
    Vertex::AttributeId registerVertexAttributeIfNotRegistered(const Vertex::AttributeName& attrName);
    Edge::AttributeId registerEdgeAttributeIfNotRegistered(const Edge::AttributeName& attrName);

    virtual Vertex::Number degree(Vertex::Number v) = 0;

protected:
    GraphInterface* getInterface() {return _interface;}
    bool isCorrectVertex(Vertex::Number v);
    bool isCorrectEdgeBetween(Vertex::Number v1, Vertex::Number v2);
    Vertices vertices;
    Edges edges;

private:   
    void initializeInterface();
    void updateEdgesAfterVertexDeletion(Vertex::Number v);
    GraphInterface* _interface;
    bool directed;
    Interfaces interfaceType;   // it can be changed in copy assignment
    GraphAttributeTranslator attrTranslator;
};

/*
    Fills passed container with edges from this graph.
*/
template<typename Container>
void Graph::getAllEdges(Container& cont)
{
    for(auto& edgePair: edges)
    {
        cont.push_back(&(edgePair.second));
    }  
}

//--------------------------------------------------------------

class UndirectedGraph : public Graph
{
public:
    UndirectedGraph(Vertex::Number verticesCount, Interfaces interface = Interfaces::AdjacencyLists);
    UndirectedGraph(const UndirectedGraph& other);
    UndirectedGraph& operator=(const UndirectedGraph& other);
    Edge& getEdgeByVertices(Vertex::Number fromVertex, Vertex::Number toVertex);
    Vertex::Number degree(Vertex::Number v);
    void getDegreesList(VertexNumbers& degrees);
};

class DirectedGraph : public Graph
{
public:
    DirectedGraph(Vertex::Number verticesCount, Interfaces interface = Interfaces::AdjacencyLists);
    DirectedGraph(const DirectedGraph& other);
    DirectedGraph& operator=(const DirectedGraph& other);
    Edge& getEdgeByVertices(Vertex::Number fromVertex, Vertex::Number toVertex);
    Vertex::Number degree(Vertex::Number v);
    Vertex::Number inDegree(Vertex::Number v);
    Vertex::Number outDegree(Vertex::Number v);
    void getDegreesLists(VertexNumbers& inDegs, VertexNumbers& outDegs, VertexNumbers& degrees);
};
