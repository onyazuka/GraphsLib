#pragma once
#include "graphs.hpp"
#include <list>
#include <algorithm>

/*
    Writes vertex attributes:
        color
        parent
        distance
        finish
        low
    Writes edge attributes:
        type
    WARNING! Passed graph MUST be connected graph!!!
    Complexity: O(|E| + |V|)
*/
template<typename VertexNumsContainer, typename EdgesContainer>
class ArticulationPointsSearcher
{
public:
    enum EdgeTypes {NoEdge = -1, Tree, Back, Direct, Cross};
    ArticulationPointsSearcher(UndirectedGraph& graph, VertexNumsContainer& vncont, EdgesContainer& edgesCont);
protected:
    void visit(UndirectedGraph& graph, Vertex::Number vNum);
    void findArticulationPoints(UndirectedGraph& graph, VertexNumsContainer& vncont, EdgesContainer& edgesCont);
    const Vertex::AttributeId ColorAttributeId;
    const Vertex::AttributeId ParentAttributeId;
    const Vertex::AttributeId DistanceAttributeId;
    const Vertex::AttributeId FinishAttributeId;
    const Vertex::AttributeId LowAttributeId;
    const Edge::AttributeId TypeAttributeId;
    enum Colors{White, Gray, Black};
    enum {NoParent=-1};
    int time;       // vertex visit time
};

template<typename VertexNumsContainer, typename EdgesContainer>
ArticulationPointsSearcher<VertexNumsContainer, EdgesContainer>::ArticulationPointsSearcher(UndirectedGraph& graph, VertexNumsContainer& vncont, EdgesContainer& edgesCont)
    : ColorAttributeId{graph.registerVertexAttributeIfNotRegistered("color")},
      ParentAttributeId{graph.registerVertexAttributeIfNotRegistered("parent")},
      DistanceAttributeId{graph.registerVertexAttributeIfNotRegistered("distance")},
      FinishAttributeId{graph.registerVertexAttributeIfNotRegistered("finish")},
      LowAttributeId{graph.registerVertexAttributeIfNotRegistered("low")},
      TypeAttributeId{graph.registerEdgeAttributeIfNotRegistered("type")}
{
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        graph.getVertexByNumber(i).setAttribute(ColorAttributeId, White);
        graph.getVertexByNumber(i).setAttribute(ParentAttributeId, NoParent);
    }
    std::vector<Edge*> edges;
    graph.getAllEdges(edges);
    for(Edge* edge: edges)
    {
        edge->setAttribute(TypeAttributeId, NoEdge);
    }
    time = 0;
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        if(graph.getVertexByNumber(i).getAttribute(ColorAttributeId) == White)
        {
            visit(graph, i);
        }
    }
    findArticulationPoints(graph, vncont, edgesCont);
}

/*
    Just a dfs, hai
    Here we are computing attribute "low" - lowest "distance" attribute value of one of ancestors of vertex, that is reachable from one of descendants on this vertex
    Have v.low = min
    {
        v.distance,                            (case 1)
        p.distance, if(v,p) is back edge       (case 2)
        to.low, where (v,to) is tree edge      (case 3)
    }
*/
template<typename VertexNumsContainer, typename EdgesContainer>
void ArticulationPointsSearcher<VertexNumsContainer, EdgesContainer>::visit(UndirectedGraph& graph, Vertex::Number vNum)
{
     std::stack<Vertex::Number> unvisitedVertexNums;
     unvisitedVertexNums.push(vNum);
     unvisitedVertexNums.push(vNum);
     while(!unvisitedVertexNums.empty())
     {
         Vertex::Number curVNum = unvisitedVertexNums.top();
         unvisitedVertexNums.pop();
         Vertex& curV = graph.getVertexByNumber(curVNum);
         if(curV.getAttribute(ColorAttributeId) == Black)
         {
             continue;
         }
         ++time;
         if(curV.getAttribute(ColorAttributeId) == White)
         {
             curV.setAttribute(ColorAttributeId, Gray);
             curV.setAttribute(DistanceAttributeId, time);
             // case 1
             curV.setAttribute(LowAttributeId, curV.getAttribute(DistanceAttributeId));
         }
         else if (curV.getAttribute(ColorAttributeId) == Gray)
         {
             curV.setAttribute(ColorAttributeId, Black);
             curV.setAttribute(FinishAttributeId, time);
             Vertex::AttributeType parentNumber = curV.getAttribute(ParentAttributeId);
             if(parentNumber != NoParent)
             {
                 Vertex& parentV = graph.getVertexByNumber(curV.getAttribute(ParentAttributeId));
                 // case 3
                 Vertex::AttributeType newLow = std::min(curV.getAttribute(LowAttributeId), parentV.getAttribute(LowAttributeId));
                 parentV.setAttribute(LowAttributeId, newLow);
             }
             continue;
         }
         Graph::VertexNumbers adjacentVertices = graph.getAdjacentVerticesFor(curVNum);
         for(auto adjNum : adjacentVertices)
         {
             Vertex& adjV = graph.getVertexByNumber(adjNum);
             Edge& edgeBetween = graph.getEdgeByVertices(curVNum, adjNum);
             if(adjV.getAttribute(ColorAttributeId) == White)
             {
                 adjV.setAttribute(ParentAttributeId, curVNum);
                 edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Tree);
                 unvisitedVertexNums.push(adjNum);
                 unvisitedVertexNums.push(adjNum);
             }
             else if(adjV.getAttribute(ColorAttributeId) == Gray)
             {
                 if(adjV.number() != curV.getAttribute(ParentAttributeId))
                 {
                     // case 2
                     curV.setAttribute(LowAttributeId, adjV.getAttribute(DistanceAttributeId));
                     edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Back);
                 }
             }
             else
             {
                 if(curV.getAttribute(DistanceAttributeId) < adjV.getAttribute(DistanceAttributeId))
                 {
                     edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Direct);
                 }
                 else
                 {
                     edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Cross);
                 }
             }
         }
     }
}

template<typename VertexNumsContainer, typename EdgesContainer>
void ArticulationPointsSearcher<VertexNumsContainer, EdgesContainer>::findArticulationPoints(UndirectedGraph& graph, VertexNumsContainer& vncont, EdgesContainer& edgesCont)
{
    // start vertex was 0 vertex of graph
    enum {StartVertexNumber = 0};
    size_t startVertexChildrenCount = 0;
    std::vector<bool> alreadyMarkedVertices;
    alreadyMarkedVertices.resize(graph.vertexCount(), false);
    for(Vertex::Number v = 0; v < graph.vertexCount(); ++v)
    {
        Vertex curV = graph.getVertexByNumber(v);
        if(curV.getAttribute(ParentAttributeId) == StartVertexNumber)
        {
            ++startVertexChildrenCount;
            continue;
        }
        if(curV.number() == StartVertexNumber)
        {
            continue;
        }
        Vertex parentV = graph.getVertexByNumber(curV.getAttribute(ParentAttributeId));
        // case 1: v.low >= v.parent.distance
        if(alreadyMarkedVertices[parentV.number()] == false &&
                curV.getAttribute(LowAttributeId) >= parentV.getAttribute(DistanceAttributeId))
        {
            vncont.push_back(parentV.number());
            alreadyMarkedVertices[parentV.number()] = true;
        }
        if(curV.getAttribute(DistanceAttributeId) == curV.getAttribute(LowAttributeId))
        {
            edgesCont.push_back(graph.getEdgeByVertices(curV.number(), parentV.number()));
        }
    }
    // case 2: vertex is start vertex and has more than 1 child
    if(startVertexChildrenCount > 1)
    {
        vncont.push_back(StartVertexNumber);
    }
}
