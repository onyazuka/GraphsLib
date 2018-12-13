# GraphsLib
Small object-oriented library for graphs and some useful algorithms.

<h3>Supported algorithms:</h3>
<ul>
  <li>BFS</li>
  <li>DFS</li>
  <li>Topological Sort</li>
  <li>Articulation points</li>
  <li>Bridges</li>
  <li>Cycle detection</li>
  <li>Undirected graph components split</li>
  <li>Directed graph strongly connected components split</li>
  <li>Euler path finding</li>
  <li>Euler circle finding</li>
  <li>Minimal spanning tree: Kruskal algorithm</li>
  <li>Minimal spanning tree: Prim algorithm</li>
  <li>Shortest path: shortest path in acyclic directed graphs</li>
  <li>Shortest path: Dijkstra</li>
  <li>Shortest path: Bellman-Ford</li>
  <li>Shortest pathes: Floyd-Warshall</li>
  <li>Maximum flow problem: Ford-Fulkerson</li>
</ul>

<h3>Work example</h3>
'''c++
    #include "graphs.hpp"
    UndirectedGraph ug(9, UndirectedGraph::Interfaces::AdjacencyMatrix);
    ug.addEdgeBetween(0,1);
    ug.addEdgeBetween(0,7);
    ug.addEdgeBetween(1,7);
    ug.addEdgeBetween(1,2);
    ug.addEdgeBetween(2,3);
    ug.addEdgeBetween(2,8);
    ug.addEdgeBetween(2,5);
    ug.addEdgeBetween(3,4);
    ug.addEdgeBetween(3,5);
    ug.addEdgeBetween(4,5);
    ug.addEdgeBetween(5,6);
    ug.addEdgeBetween(6,7);
    ug.addEdgeBetween(6,8);
    ug.addEdgeBetween(7,8);
    const Edge::AttributeId WeightAttrId = ug.addEdgeAttribute("weight");
    ug.getEdgeByVertices(0,1).setAttribute(WeightAttrId, 4);
    ug.getEdgeByVertices(0,7).setAttribute(WeightAttrId, 8);
    ug.getEdgeByVertices(1,7).setAttribute(WeightAttrId, 11);
    ug.getEdgeByVertices(1,2).setAttribute(WeightAttrId, 8);
    ug.getEdgeByVertices(2,3).setAttribute(WeightAttrId, 7);
    ug.getEdgeByVertices(2,8).setAttribute(WeightAttrId, 2);
    ug.getEdgeByVertices(2,5).setAttribute(WeightAttrId, 4);
    ug.getEdgeByVertices(3,4).setAttribute(WeightAttrId, 9);
    ug.getEdgeByVertices(3,5).setAttribute(WeightAttrId, 14);
    ug.getEdgeByVertices(4,5).setAttribute(WeightAttrId, 10);
    ug.getEdgeByVertices(5,6).setAttribute(WeightAttrId, 2);
    ug.getEdgeByVertices(6,7).setAttribute(WeightAttrId, 1);
    ug.getEdgeByVertices(6,8).setAttribute(WeightAttrId, 6);
    ug.getEdgeByVertices(7,8).setAttribute(WeightAttrId, 7);
    UndirectedGraph spanningTree = minimalSpanningTreeKruskal(ug);
'''
