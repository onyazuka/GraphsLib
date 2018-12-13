#pragma once
#include <algorithm>
#include <unordered_set>
#include <set>
#include <vector>
#include <stack>
#include "vertices.hpp"
/*
    We can represent a graph as:
        1. Adjacency lists;
        2. Adjacency matrix.
    Some times one way is better than another.
    This is an abstract class for ways of graph representations.
*/
class GraphInterface
{
public:
    typedef std::vector<Vertex::Number> VertexNumbers;
    typedef std::vector<VertexNumbers> AdjacencyMatrix;
    virtual ~GraphInterface() {}
    virtual bool hasEdge(Vertex::Number, Vertex::Number) const = 0;
    virtual void addVertex(Vertex::Number = 1) = 0;
    virtual void addEdge(Vertex::Number, Vertex::Number, bool /*directed*/) = 0;
    virtual void deleteEdge(Vertex::Number, Vertex::Number, bool /*directed*/) = 0;
    virtual void transpose() = 0;
    virtual Vertex::Number inDegree(Vertex::Number) const = 0;
    virtual Vertex::Number outDegree(Vertex::Number) const = 0;
    virtual Vertex::Number degree(Vertex::Number) const = 0;
    virtual void getDegreesList(VertexNumbers& inDegContainer, VertexNumbers& outDegContainer, VertexNumbers& degContainer) const = 0;
    virtual VertexNumbers getAdjacentVerticesFor(Vertex::Number v) const = 0;
};

class AdjacencyListsInterface : public GraphInterface
{
    typedef std::vector<std::unordered_set<Vertex::Number>>  AdjacencyLists;
public:
    AdjacencyListsInterface(){}
    AdjacencyListsInterface(const AdjacencyListsInterface& other);
    AdjacencyListsInterface& operator=(const AdjacencyListsInterface& other);
    bool hasEdge(Vertex::Number v1, Vertex::Number v2) const;
    void addVertex(Vertex::Number count = 1);
    void addEdge(Vertex::Number v1, Vertex::Number v2, bool directed);
    void deleteEdge(Vertex::Number v1, Vertex::Number v2, bool directed);
    void transpose();
    Vertex::Number inDegree(Vertex::Number) const;
    Vertex::Number outDegree(Vertex::Number) const;
    Vertex::Number degree(Vertex::Number) const;
    void getDegreesList(VertexNumbers& inDegContainer, VertexNumbers& outDegContainer, VertexNumbers& degContainer) const;
    VertexNumbers getAdjacentVerticesFor(Vertex::Number v) const;
private:
    AdjacencyLists adjLists;
};

class AdjacencyMatrixInterface : public GraphInterface
{
    typedef bool ElementPresented;
    enum {NotPresented, Presented};
    typedef std::vector<std::vector<ElementPresented>> AdjacencyMatrix;
public:
    AdjacencyMatrixInterface(){}
    AdjacencyMatrixInterface(const AdjacencyMatrixInterface& other);
    AdjacencyMatrixInterface& operator=(const AdjacencyMatrixInterface& other);
    bool hasEdge(Vertex::Number v1, Vertex::Number v2) const ;
    void addVertex(Vertex::Number count = 1);
    void addEdge(Vertex::Number v1, Vertex::Number v2, bool directed);
    void deleteEdge(Vertex::Number v1, Vertex::Number v2, bool directed);
    void transpose();
    Vertex::Number inDegree(Vertex::Number) const;
    Vertex::Number outDegree(Vertex::Number) const;
    Vertex::Number degree(Vertex::Number) const;
    void getDegreesList(VertexNumbers& inDegContainer, VertexNumbers& outDegContainer, VertexNumbers& degContainer) const;
    VertexNumbers getAdjacentVerticesFor(Vertex::Number v) const;
private:
    AdjacencyMatrix adjMatrix;
};
