#include "graphs.hpp"

//-----------------------------Graph

Graph::Graph(Vertex::Number verticesCount, Interfaces interface, bool _directed)
    : directed{_directed}, interfaceType{interface}
{
    initializeInterface();
    // adding vertices
    addVertex(verticesCount);
}

Graph::Graph(const Graph& other)
    : vertices{other.vertices}, edges{other.edges}, directed{other.directed}, interfaceType{other.interfaceType}, attrTranslator{other.attrTranslator}
{
    initializeInterface();
    // adding vertices and edges into interface
    for(auto vertex: vertices)
    {
        _interface->addVertex();
    }
    for(auto edge: edges)
    {
        getInterface()->addEdge(edge.second.v1().number(), edge.second.v2().number(), isDirected());
    }
}

Graph& Graph::operator=(const Graph& other)
{
    vertices = other.vertices;
    edges = other.edges;
    directed = other.directed;
    interfaceType = other.interfaceType;
    attrTranslator = other.attrTranslator;
    initializeInterface();
    // adding vertices and edges into interface
    for(auto vertex: vertices)
    {
        _interface->addVertex();
    }
    for(auto edge: edges)
    {
        getInterface()->addEdge(edge.second.v1().number(), edge.second.v2().number(), isDirected());
    }
    return *this;
}

Graph::~Graph()
{
    delete _interface;
}

Vertex& Graph::getVertexByNumber(Vertex::Number number)
{
    return vertices.at(number);
}

bool Graph::areConnected(Vertex::Number v1, Vertex::Number v2)
{
    if(!isCorrectEdgeBetween(v1, v2))
    {
        throw std::out_of_range("Graph::areConnected() - incorrect edge");
    }
    return getInterface()->hasEdge(v1, v2);
}

// number of new vertex will be vertices count + 'count'
void Graph::addVertex(Vertex::Number count)
{
    for(int i = 0; i < count; ++i)
    {
        Vertex newVertex(vertices.size());
        vertices.push_back(newVertex);
        newVertex.addNewAttributes(attrTranslator.getVertexAttributesCount());
    }
    getInterface()->addVertex(count);
}

// preconditions: 0 < from < vertexCount(); 0 < to < vertexCount();
void Graph::addEdgeBetween(Vertex::Number v1, Vertex::Number v2)
{
    if(!isCorrectEdgeBetween(v1, v2))
    {
        throw std::out_of_range("Graph::addEdgeBetween() - incorrect edge");
    }
    // we do not want to create a new edge if we already has it
    if(areConnected(v1, v2))
    {
        return;
    }
    getInterface()->addEdge(v1, v2, isDirected());
    Edge newEdge(&getVertexByNumber(v1), &getVertexByNumber(v2), isDirected());
    newEdge.addNewAttributes(attrTranslator.getEdgeAttributesCount());
    edges[cantorFunction(v1, v2)] = newEdge;

}

void Graph::deleteEdgeBetween(Vertex::Number v1, Vertex::Number v2)
{
    if(!isCorrectEdgeBetween(v1, v2))
    {
        throw std::out_of_range("Graph::deleteEdgeBetween() - incorrect edge");
    }
    getInterface()->deleteEdge(v1, v2, isDirected());
    edges.erase(cantorFunction(v1, v2));
}

void Graph::deleteEdge(Edge edge)
{
    deleteEdgeBetween(edge.v1().number(), edge.v2().number());
}

void Graph::transpose()
{
    // transposing edges
    Edges newEdges;
    for(auto item: edges)
    {
        Vertex::Number v1 = item.second.v1().number();
        Vertex::Number v2 = item.second.v2().number();
        newEdges[cantorFunction(v2, v1)] = Edge(&getVertexByNumber(v2), &getVertexByNumber(v1), isDirected());
    }
    edges = newEdges;
    // transposing adjacency struct
    getInterface()->transpose();
}

/*
    It IS designed to have stable order in resulting vector -
        adjacent vertices returned in sorted order.
*/
std::vector<Vertex::Number> Graph::getAdjacentVerticesFor(Vertex::Number v)
{
    if(!isCorrectVertex(v))
    {
        throw std::out_of_range("Graph::getAdjacentEdgesFor() - incorrect vertex");
    }
    return getInterface()->getAdjacentVerticesFor(v);
}

void Graph::clearAttributes()
{
    for (auto& vertex: vertices)
    {
        vertex.clearAttributes();
    }
    for (auto& edge: edges)
    {
        edge.second.clearAttributes();
    }
}

Vertex::AttributeId Graph::addVertexAttribute(const Vertex::AttributeName& attrName)
{
    Vertex::AttributeId newId = attrTranslator.addVertexAttribute(attrName);
    for(int v = 0; v < vertexCount(); ++v)
    {
        getVertexByNumber(v).addNewAttributes();
    }
    return newId;
}

Edge::AttributeId Graph::addEdgeAttribute(const Edge::AttributeName& attrName)
{
    Edge::AttributeId newId = attrTranslator.addEdgeAttribute(attrName);
    for(auto& edgePair : edges)
    {
        edgePair.second.addNewAttributes();
    }
    return newId;
}

Vertex::AttributeId Graph::getVertexAttributeIdByName(const Vertex::AttributeName& attrName) const
{
    return attrTranslator.getVertexAttributeIdByName(attrName);
}

Edge::AttributeId Graph::getEdgeAttributeIdByName(const Edge::AttributeName& attrName) const
{
    return attrTranslator.getEdgeAttributeIdByName(attrName);
}

bool Graph::isVertexAttributeIdRegistered(const Vertex::AttributeName& attrName) const
{
    return attrTranslator.isVertexAttributeIdRegistered(attrName);
}

bool Graph::isEdgeAttributeIdRegistered(const Edge::AttributeName& attrName) const
{
    return attrTranslator.isEdgeAttributeIdRegistered(attrName);
}

Vertex::AttributeId Graph::registerVertexAttributeIfNotRegistered(const Vertex::AttributeName& attrName)
{
    if(!isVertexAttributeIdRegistered(attrName))
    {
        return addVertexAttribute(attrName);
    }
    return getVertexAttributeIdByName(attrName);
}

Edge::AttributeId Graph::registerEdgeAttributeIfNotRegistered(const Edge::AttributeName& attrName)
{
    if(!isEdgeAttributeIdRegistered(attrName))
    {
        return addEdgeAttribute(attrName);
    }
    return getEdgeAttributeIdByName(attrName);
}

// conditions: 0 <= v < vertexCount()
bool Graph::isCorrectVertex(Vertex::Number v)
{
    return (0 <= v) && (v < vertexCount());
}

// conditions: 0 <= v1 < vertexCount(); 0 <= v2 < vertexCount();
bool Graph::isCorrectEdgeBetween(Vertex::Number v1, Vertex::Number v2)
{
    return (isCorrectVertex(v1) && isCorrectVertex(v2));
}

void Graph::initializeInterface()
{
    switch(interfaceType)
    {
    case Interfaces::AdjacencyLists:
        _interface = new AdjacencyListsInterface{};
        break;
    case Interfaces::AdjacencyMatrix:
        _interface = new AdjacencyMatrixInterface{};
        break;
    default:
        throw GraphException{"Graph constructor: unsupported interface"};
    }
}

//-----------------------------/Graph

//-----------------------------Undirected Graph

UndirectedGraph::UndirectedGraph(Vertex::Number verticesCount, Graph::Interfaces interface)
    :Graph{verticesCount, interface, false} {}

UndirectedGraph::UndirectedGraph(const UndirectedGraph &other)
    : Graph{other}{}

UndirectedGraph& UndirectedGraph::operator=(const UndirectedGraph& other)
{
    Graph::operator =(other);
    return *this;
}

// gets or edge[from][to] or edge[to][from]
Edge& UndirectedGraph::getEdgeByVertices(Vertex::Number fromVertex, Vertex::Number toVertex)
{
    if(!isCorrectEdgeBetween(fromVertex, toVertex))
    {
        throw std::out_of_range("UndirectedGraph::getEdgeByVertices() - incorrect edge");
    }
    // searching edge
    auto foundEdge = edges.find(cantorFunction(fromVertex, toVertex));
    if(foundEdge != edges.end())
    {
        return foundEdge->second;
    }
    // searching reverse edge
    auto foundReverseEdge = edges.find(cantorFunction(toVertex, fromVertex));
    if(foundReverseEdge != edges.end())
    {
        return foundReverseEdge->second;
    }
    // not found
    throw std::out_of_range("UndirectedGraph::getEdgeByVertices(): incorrect edge");
}

Vertex::Number UndirectedGraph::degree(Vertex::Number v)
{
    if(!isCorrectVertex(v))
    {
        throw std::out_of_range("Graph::degree(v): incorrect vertex number v");
    }
    // because we have not outdeg or indeg for undirected graphs
    // here degree is number of adjacent vertices
    return getInterface()->inDegree(v);
}

void UndirectedGraph::getDegreesList(VertexNumbers& degrees)
{
    // fake lists
    VertexNumbers indegs;
    VertexNumbers outdegs;
    //---
    getInterface()->getDegreesList(indegs, outdegs, degrees);
    for (auto& item: degrees)
    {
        item /= 2;
    }
}

//-----------------------------/Undirected Graph

//-----------------------------Directed Graph

DirectedGraph::DirectedGraph(Vertex::Number verticesCount, Graph::Interfaces interface)
    : Graph{verticesCount, interface, true} {}

DirectedGraph::DirectedGraph(const DirectedGraph &other)
    : Graph{other}{}

DirectedGraph& DirectedGraph::operator=(const DirectedGraph& other)
{
    Graph::operator=(other);
    return *this;
}

Edge& DirectedGraph::getEdgeByVertices(Vertex::Number fromVertex, Vertex::Number toVertex)
{
    if(!isCorrectEdgeBetween(fromVertex, toVertex))
    {
        throw std::out_of_range("DirectedGraph::getEdgeByVertices() - incorrect edge");
    }
    auto foundEdge = edges.find(cantorFunction(fromVertex, toVertex));
    if(foundEdge != edges.end())
    {
        return foundEdge->second;
    }
    // not found
    throw std::out_of_range("DirectedGraph::getEdgeByVertices(): incorrect edge");
}

Vertex::Number DirectedGraph::inDegree(Vertex::Number v)
{
    if(!isCorrectVertex(v))
    {
        throw std::out_of_range("Graph::inDegree(v): incorrect vertex number v");
    }
    return getInterface()->inDegree(v);
}

Vertex::Number DirectedGraph::outDegree(Vertex::Number v)
{
    if(!isCorrectVertex(v))
    {
        throw std::out_of_range("Graph::outDegree(v): incorrect vertex number v");
    }
    return getInterface()->outDegree(v);
}

Vertex::Number DirectedGraph::degree(Vertex::Number v)
{
    if(!isCorrectVertex(v))
    {
        throw std::out_of_range("Graph::degree(v): incorrect vertex number v");
    }
    return getInterface()->degree(v);
}

void DirectedGraph::getDegreesLists(VertexNumbers& inDegs, VertexNumbers& outDegs, VertexNumbers& degrees)
{
    getInterface()->getDegreesList(inDegs, outDegs, degrees);
}

//-----------------------------/Directed Graph
