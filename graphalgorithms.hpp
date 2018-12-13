#pragma once
#include <list>
#include <algorithm>
#include <functional>
#include <set>
#include <map>
#include "graphs.hpp"
#include "DataStructures/disjointset.hpp"
#include "graphalgorithmstopologicalsort.h"
#include "grapharticulationpointssearcher.hpp"
#include "DataStructures/heap.hpp"
#include "utils.hpp"

void BFS(Graph& graph, Vertex::Number s);

class DFS
{
public:
    enum EdgeTypes {NoType = -1, Tree, Back, Direct, Cross};
    DFS(Graph& graph);
protected:
    void DFS_Visit(Graph& graph, Vertex::Number vNum);

    const Vertex::AttributeId ColorAttributeId;
    const Vertex::AttributeId ParentAttributeId;
    const Vertex::AttributeId DistanceAttributeId;
    const Vertex::AttributeId FinishAttributeId;
    const Edge::AttributeId TypeAttributeId;
    enum Colors{White, Gray, Black};
    enum {NoParent=-1};
    int time;       // vertex visit time
};

template<typename Container>
class TopologicalSort;

bool hasCycle(Graph& graph);

class UndirectedComponentsSplitter
{
public:
    typedef std::vector<Vertex::Number> Component;
    typedef std::vector<Component> Components;
    UndirectedComponentsSplitter(UndirectedGraph& graph, Components& components);
private:
    const Vertex::AttributeId ColorAttributeId;

    void visit(UndirectedGraph& graph, Vertex::Number vNum, Components& components);
    enum Colors{White, Gray};
};

class StronglyConnectedComponents
{
public:
    typedef std::vector<Graph::VertexNumbers> Components;
    StronglyConnectedComponents(DirectedGraph& graph, Components& components);
protected:
    void StrongDFS(DirectedGraph& graph);
    void visit(DirectedGraph& graph, Vertex::Number vNum);
    size_t time;
    const Vertex::AttributeId ColorAttributeId;
    const Vertex::AttributeId DistanceAttributeId;
    const Vertex::AttributeId FinishAttributeId;
    enum Colors{White, Gray};
};

class EulerianBuilder
{
    enum {Unvisited, Visited};
public:
    typedef std::vector<Vertex> Vertices;
    typedef std::vector<Vertex::Number> Eulerian;
    EulerianBuilder(UndirectedGraph graphCopy);
    EulerianBuilder(DirectedGraph graphCopy);
    bool hasEulerianCircuit() const {return eulerianCircuitFound;}
    bool hasEulerianPath() const {return eulerianPathFound;}
    Eulerian getEulerianCircuit() const {return eulerian;}
    Eulerian getEulerianPath() const {return eulerian;}
private:
    Vertex::Number checkEulerianConditions(UndirectedGraph& graph);
    Vertex::Number checkEulerianConditions(DirectedGraph& graph);
    void buildEulerian(Graph& graph, Vertex::Number startVertexNumber);
    Eulerian eulerian;
    std::stack<Vertex::Number> walkingStack;
    bool eulerianCircuitFound;
    bool eulerianPathFound;
};

UndirectedGraph minimalSpanningTreeKruskal(UndirectedGraph& graph);
UndirectedGraph minimalSpanningTreePrim(UndirectedGraph& graph, Vertex::Number v = 0);

class ShortestPathSearcher
{
public:
    enum {NoParent = -1, WeightInf = std::numeric_limits<Vertex::AttributeType>::max()};
    typedef std::vector<std::vector<Vertex::AttributeType>> Matrix;
    enum class BellmanFordReturnCode {HasNegativeCycle, Ok};

    ShortestPathSearcher(DirectedGraph* graph);

    BellmanFordReturnCode bellmanFord(Vertex::Number source);
    void dagShortestPaths(Vertex::Number source);
    void dijkstra(Vertex::Number source);
    void floydWarshall(Matrix& weights, Matrix& parents);
private:
    const Vertex::AttributeId ParentAttributeId;
    const Vertex::AttributeId DistanceAttributeId;
    const Edge::AttributeId WeightAttributeId;
    DirectedGraph* graph;

    void doAdjacencyWeightMatrix(Matrix& weightMatrix);
    void fillParentsMatrixFromWeightsMatrix(Matrix& weightsMatrix, Matrix& parentsMatrix);
    void initializeSingleSource(Vertex::Number source);
    void relax(Vertex::Number v1, Vertex::Number v2);
};

class FordFulkerson
{
public:
    FordFulkerson(DirectedGraph& graph, Vertex::Number source, Vertex::Number to);
private:
    typedef std::vector<Edge*> pEdges;
    bool isCorrectNetwork(DirectedGraph& graph, Vertex::Number source, Vertex::Number to);
    size_t findPathCost(DirectedGraph& graph, Vertex::Number source, Vertex::Number to);
    void updateFlows(DirectedGraph& graph, Vertex::Number source, Vertex::Number to, size_t pathCost);
    DirectedGraph makeResidualGraph(DirectedGraph& graph);
    const Vertex::AttributeId ColorAttributeId;
    const Vertex::AttributeId ParentAttributeId;
    const Edge::AttributeId FlowAttributeId;
    const Edge::AttributeId CurrentAttributeId;
    enum Colors{White, Gray};
    enum {NoParent=-1};
    Edge::AttributeId ResidualCurrentAttributeId;
    Edge::AttributeId ResidualColorAttributeId;
    Edge::AttributeId ResidualParentAttributeId;
    Edge::AttributeId ResidualDistanceAttributeId;
};


