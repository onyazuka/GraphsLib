#pragma once
#include "graphs.hpp"
#include <list>
#include <algorithm>

/*
    The same as DFS, but also fills passed container with vertices sorted in topological order.
    Topological sort has sense only for acyclic directed graphs.
    WARNING 1: if passed graph is cyclic, algorithm has undefined behaviour!!!
    WARNING 2: it is lame and ugly, but I have not found better way to do this :(
    Writes vertex attributes:
        color
        parent
        distance
        finish
    Writes edge attributes:
        type
*/
//--------------------------------Topological Sort

template<typename Container>
class TopologicalSort
{
public:
    TopologicalSort();
    enum EdgeTypes {Tree, Back, Direct, Cross};
    TopologicalSort(DirectedGraph& graph, Container/*<Vertex>*/& cont);
protected:
    void TopologicalSortVisit(Graph& graph, Vertex::Number vNum, std::list<Vertex>& sortedCont);
    enum Colors{White, Gray, Black};
    enum {NoParent=-1};
    const Vertex::AttributeId ColorAttributeId;
    const Vertex::AttributeId ParentAttributeId;
    const Vertex::AttributeId DistanceAttributeId;
    const Vertex::AttributeId FinishAttributeId;
    const Edge::AttributeId TypeAttributeId;
    int time;       // vertex visit time
};


template<typename Container>
TopologicalSort<Container>::TopologicalSort(DirectedGraph& graph, Container/*<Vertex>*/& cont)
    : ColorAttributeId{graph.registerVertexAttributeIfNotRegistered("color")},
      ParentAttributeId{graph.registerVertexAttributeIfNotRegistered("parent")},
      DistanceAttributeId{graph.registerVertexAttributeIfNotRegistered("distance")},
      FinishAttributeId{graph.registerVertexAttributeIfNotRegistered("finish")},
      TypeAttributeId{graph.registerEdgeAttributeIfNotRegistered("type")}

{   
    std::list<Vertex> sortedCont;
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        graph.getVertexByNumber(i).setAttribute(ColorAttributeId, White);
        graph.getVertexByNumber(i).setAttribute(ParentAttributeId, NoParent);
    }
    time = 0;
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        if(graph.getVertexByNumber(i).getAttribute(ColorAttributeId) == White)
        {
            TopologicalSortVisit(graph, i, sortedCont);
        }
    }
    std::move(sortedCont.begin(), sortedCont.end(), std::back_inserter(cont));
}

template<typename Container>
void TopologicalSort<Container>::TopologicalSortVisit(Graph& graph, Vertex::Number vNum, std::list<Vertex>& sortedCont)
{
    std::stack<Vertex::Number> unvisitedVertexNums;
    unvisitedVertexNums.push(vNum);
    unvisitedVertexNums.push(vNum);
    while(!unvisitedVertexNums.empty())
    {
        Vertex::Number curVNum = unvisitedVertexNums.top();
        unvisitedVertexNums.pop();
        Vertex& curV = graph.getVertexByNumber(curVNum);
        // vertex already processed
        if(curV.getAttribute(ColorAttributeId) == Black)
        {
            continue;
        }
        ++time;
        // visiting first time
        if(curV.getAttribute(ColorAttributeId) == White)
        {
            curV.setAttribute(ColorAttributeId, Gray);
            curV.setAttribute(DistanceAttributeId, time);
        }
        // returning back to this vertex
        else if (curV.getAttribute(ColorAttributeId) == Gray)
        {
            curV.setAttribute(ColorAttributeId, Black);
            curV.setAttribute(FinishAttributeId, time);
            sortedCont.push_front(curV);
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
                // next vertex is not visited predecessor and in tree - tree edge
                edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Tree);
                unvisitedVertexNums.push(adjNum);
                unvisitedVertexNums.push(adjNum);
            }
            // next vertex is ancestor, but visited - edge has back type
            else if(adjV.getAttribute(ColorAttributeId) == Gray)
            {
                edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Back);
            }
            else
            {
                // next vertex is visited predecessor - direct edge(because it is not in tree)
                if(curV.getAttribute(DistanceAttributeId) < adjV.getAttribute(DistanceAttributeId))
                {
                    edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Direct);
                }
                // next vertex is predecessor from other component - cross edge
                else
                {
                    edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Cross);
                }
            }
        }
    }
}

//--------------------------------/Topological Sort
