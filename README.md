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
  <li>Eulerian path</li>
  <li>Eulerian circuit</li>
  <li>Minimal spanning tree: Kruskal algorithm</li>
  <li>Minimal spanning tree: Prim algorithm</li>
  <li>Shortest path: shortest path in acyclic directed graphs</li>
  <li>Shortest path: Dijkstra</li>
  <li>Shortest path: Bellman-Ford</li>
  <li>Shortest pathes: Floyd-Warshall</li>
  <li>Maximum flow problem: Ford-Fulkerson</li>
</ul>

<h3>Complexities</h3>
θ(V+E) - BFS, DFS, Topological Sort, Articulation points, Bridges, Cycle detection, Components, Eulerian, 
  directed acyclic graph shortest path<br/>
θ(ElgV) - Kruskal, Prim, Dijkstra<br/>
θ(EV) - Bellman-Ford<br/>
θ(V^3) - Floyd-Warshall<br/>
θ(E^2 V) - Ford-Fulkerson<br/>

<h3>Features</h3>
<ul>
  <li>Graph inner structure: adjacency list or adjacency matrix</li>
  <li>Undirected and directed graphs</li>
  <li>Copying of graphs, vertices, edges</li>
  <li>Adding of edges</li>
  <li>Adding of vertices</li>
  <li>Getting the edge by its vertices</li>
  <li>Vertex degrees</li>
  <li>Graph transposing</li>
  <li>Attributes: adding, deleting, getting, setting</li>
</ul>
