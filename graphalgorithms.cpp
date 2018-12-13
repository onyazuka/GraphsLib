#include "graphalgorithms.hpp"

/*
    Breadth First Search
    Finds out distance for each vertex of the graph from vertex startV.
    Writes vertex attributes:
        Vertex::Attribute::Parent - parent in algorithm running for this vertex;
        DistanceAttrId1
        Color
    attributes[DistanceAttrId1] == -1 means that this vertex is unreachable from startV.
    attributes[Vertex::Attribute::Parent] == -1 means that this vertex has no parent
    Complexity: O(|V|+|E|) (both with adjacency matrix and lists)
        (max |V| white vertices and max |E| as total of adjacent vertices)
*/
void BFS(Graph& graph, Vertex::Number startVNum)
{
    enum {NotReachable = -1};
    enum {NoParent = -1};
    enum Colors {White, Gray};
    const Vertex::AttributeId ColorAttrId = graph.registerVertexAttributeIfNotRegistered("color");
    const Vertex::AttributeId DistanceAttrId = graph.registerVertexAttributeIfNotRegistered("distance");
    const Vertex::AttributeId ParentAttrId = graph.registerVertexAttributeIfNotRegistered("parent");
    for(Vertex::Number curV = 0; curV < graph.vertexCount(); ++curV)
    {
        graph.getVertexByNumber(curV).setAttribute(ColorAttrId, White);
        graph.getVertexByNumber(curV).setAttribute(DistanceAttrId, NotReachable);
        graph.getVertexByNumber(curV).setAttribute(ParentAttrId, NoParent);
    }
    graph.getVertexByNumber(startVNum).setAttribute(ColorAttrId, Gray);
    graph.getVertexByNumber(startVNum).setAttribute(DistanceAttrId, 0);
    graph.getVertexByNumber(startVNum).setAttribute(ParentAttrId, NoParent);

    std::deque<Vertex::Number> unprocessedVertices;
    unprocessedVertices.push_back(startVNum);
    while(!unprocessedVertices.empty())
    {
        Vertex::Number curVNum = unprocessedVertices.front();
        Vertex curV = graph.getVertexByNumber(curVNum);
        unprocessedVertices.pop_front();
        std::vector<Vertex::Number> adjVertices = graph.getAdjacentVerticesFor(curVNum);
        for(auto curAdjNum: adjVertices)
        {
            Vertex& curAdjV = graph.getVertexByNumber(curAdjNum);
            if(curAdjV.getAttribute(ColorAttrId) == White)
            {
                curAdjV.setAttribute(ColorAttrId, Gray);
                curAdjV.setAttribute(DistanceAttrId,
                                    curV.getAttribute(DistanceAttrId) + 1);
                curAdjV.setAttribute(ParentAttrId, curVNum);
                unprocessedVertices.push_back(curAdjNum);
            }
        }
    }
}

//------------------------------------DFS

/*
    Depth First Search
    Writes vertex attributes:
        parent
        color
        distance - first access time
        finish - last access time
    Writes edge attributes:
        type
        Edges can be:
            tree
            back
            direct
            cross
    Complexity: O(|V|+|E|)
    WARNING! Order of DFS traversal is not determined!
*/
DFS::DFS(Graph& graph)
    : ColorAttributeId{graph.registerVertexAttributeIfNotRegistered("color")},
      ParentAttributeId{graph.registerVertexAttributeIfNotRegistered("parent")},
      DistanceAttributeId{graph.registerVertexAttributeIfNotRegistered("distance")},
      FinishAttributeId{graph.registerVertexAttributeIfNotRegistered("finish")},
      TypeAttributeId{graph.registerEdgeAttributeIfNotRegistered("type")}
{
    // initialize vertices
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        graph.getVertexByNumber(i).setAttribute(ColorAttributeId, White);
        graph.getVertexByNumber(i).setAttribute(ParentAttributeId, NoParent);
    }
    time = 0;
    // need to initialize edge types to correctly track it in undirected graphs
    std::vector<Edge*> edges;
    graph.getAllEdges(edges);
    for(Edge* edge: edges)
    {
        edge->setAttribute(TypeAttributeId, NoType);
    }
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        if(graph.getVertexByNumber(i).getAttribute(ColorAttributeId) == White)
        {
            DFS_Visit(graph, i);
        }
    }
}

/*
    Using iterative DFS so it will no crash with big graphs(recursion limit).
*/
void DFS::DFS_Visit(Graph& graph, Vertex::Number vNum)
{
     std::stack<Vertex::Number> unvisitedVertexNums;
     // pushing 2 copies of vertex number because one is processed for counting distance,
     // and other for counting finish
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
                 // here we need to distinguish undirected edges from directed
                 // if we came from adjV it can't be back edge in undirected graphs
                 if(adjV.number() != curV.getAttribute(ParentAttributeId))
                 {
                    edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Back);
                 }
                 // but if graph IS directed, it is separate back edge
                 else
                 {
                     if(edgeBetween.isDirected())
                     {
                         edgeBetween.setAttribute(TypeAttributeId, EdgeTypes::Back);
                     }
                 }
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

//------------------------------------/DFS

/*
    O(|V| + |E|)
    Reads edge attributes:
        type
    We can do for O(|V|), but this will need changed version of DFS:
        which returns just after finding back edge. That guarantees us that we will visit each vertex exactly one time.
*/
bool hasCycle(Graph& graph)
{
    DFS{graph};
    const Edge::AttributeId TypeAttrId = graph.getEdgeAttributeIdByName("type");
    std::vector<const Edge*> graphEdges;
    graph.getAllEdges(graphEdges);
    for(auto edge: graphEdges)
    {
        if(edge->getAttribute(TypeAttrId) == DFS::EdgeTypes::Back)
        {
            return true;
        }
    }
    return false;
}

//-------------------------------Undirected Graph Components

UndirectedComponentsSplitter::UndirectedComponentsSplitter(UndirectedGraph& graph, Components& components)
    : ColorAttributeId{graph.registerVertexAttributeIfNotRegistered("color")}
{
    // initialize vertices
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        graph.getVertexByNumber(i).setAttribute(ColorAttributeId, White);
    }
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        if(graph.getVertexByNumber(i).getAttribute(ColorAttributeId) == White)
        {
            components.push_back(Component{});
            components.back().push_back(i);
            visit(graph, i, components);
        }
    }
}

void UndirectedComponentsSplitter::visit(UndirectedGraph& graph, Vertex::Number vNum, Components& components)
{
    std::stack<Vertex::Number> unvisitedVertexNums;
    // not using double push because we don't need 'finish' attribute
    unvisitedVertexNums.push(vNum);
    while(!unvisitedVertexNums.empty())
    {
        Vertex::Number curVNum = unvisitedVertexNums.top();
        unvisitedVertexNums.pop();
        Vertex& curV = graph.getVertexByNumber(curVNum);
        if(curV.getAttribute(ColorAttributeId) == Gray)
        {
            continue;
        }
        curV.setAttribute(ColorAttributeId, Gray);
        std::vector<Vertex::Number> adjacentVertices = graph.getAdjacentVerticesFor(curVNum);
        for(Vertex::Number adjNum : adjacentVertices)
        {
            Vertex& adjV = graph.getVertexByNumber(adjNum);
            // not visited
            if(adjV.getAttribute(ColorAttributeId) == White)
            {
                components.back().push_back(adjNum);
                unvisitedVertexNums.push(adjNum);
            }
        }
    }
}

//-------------------------------/Undirected Graph Components

//--------------------------------Strongly Connected Components

/*
    Splits directed graph by strongly connected components.
    Returns vector of vertex numbers of separated components.
    Writes vertex attributes:
        color
        parent
        distance
        finish
*/
StronglyConnectedComponents::StronglyConnectedComponents(DirectedGraph& graph, Components& components)
    : ColorAttributeId{graph.registerVertexAttributeIfNotRegistered("color")},
      DistanceAttributeId{graph.registerVertexAttributeIfNotRegistered("distance")},
      FinishAttributeId{graph.registerVertexAttributeIfNotRegistered("finish")}
{
    typedef std::vector<Vertex> Vertices;
    enum {InvalidValue = -1};
    // not needed to process later - return empty vector
    if(graph.vertexCount() <= 1)
    {
        return;
    }
    // 1. calling DFS
    DFS{graph};
    // 2. finding transponate graph
    DirectedGraph transposedGraph = graph;
    transposedGraph.transpose();

    // 3. calling DFS on transponate graph(with vertices sorted by decreasing Vertex::Attribute::Finish attribute
    StrongDFS(transposedGraph);

    // 4. finding vertices and edges of separate components
    // 4.1 sorting vertices by "distance" attribute order
    Vertices vertices;
    for(int i = 0; i < transposedGraph.vertexCount(); ++i)
    {
        vertices.push_back(transposedGraph.getVertexByNumber(i));
    }
    countingSortTransformed(vertices.begin(), vertices.end(), [this](const Vertex& v){ return v.getAttribute(DistanceAttributeId); });
    // new component condition: finish(C') > finish(C)
    int lastVertexFinish = InvalidValue;
    for(auto& vertex: vertices)
    {
        if(vertex.getAttribute(FinishAttributeId) > lastVertexFinish)
        {
            components.push_back(std::vector<Vertex::Number>());
        }
        components.back().push_back(vertex.number());
        lastVertexFinish = vertex.getAttribute(FinishAttributeId);
    }
}

void StronglyConnectedComponents::StrongDFS(DirectedGraph& graph)
{
    // initializing vertices
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        graph.getVertexByNumber(i).setAttribute(ColorAttributeId, White);
    }
    time = 0;
    // sorting by decreasing finish attribute
    std::vector<Vertex> graphVertices;
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        graphVertices.push_back(graph.getVertexByNumber(i));
    }
    countingSortTransformed(graphVertices.begin(), graphVertices.end(),
                            [this](const Vertex& v) { return v.getAttribute(FinishAttributeId);
    });
    std::reverse(graphVertices.begin(), graphVertices.end());
    for(auto v : graphVertices)
    {
        if(graph.getVertexByNumber(v.number()).getAttribute(ColorAttributeId) == White)
        {
            visit(graph, v.number());
        }
    }
};

void StronglyConnectedComponents::visit(DirectedGraph& graph, Vertex::Number vNum)
{   
    std::stack<Vertex::Number> unvisitedVertexNums;
    unvisitedVertexNums.push(vNum);
    unvisitedVertexNums.push(vNum);
    while(!unvisitedVertexNums.empty())
    {
        Vertex::Number curVNum = unvisitedVertexNums.top();
        unvisitedVertexNums.pop();
        Vertex& curV = graph.getVertexByNumber(curVNum);
        if(curV.getAttribute(ColorAttributeId) > Gray)
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
            curV.setAttribute(FinishAttributeId, time);
            continue;
        }
        curV.setAttribute(DistanceAttributeId, time);
        curV.setAttribute(ColorAttributeId, Gray);
        Graph::VertexNumbers adjacentVertices = graph.getAdjacentVerticesFor(curVNum);
        for(Vertex::Number adjNum : adjacentVertices)
        {
            Vertex& adjV = graph.getVertexByNumber(adjNum);
            // not visited
            if(adjV.getAttribute(ColorAttributeId) == White)
            {
                unvisitedVertexNums.push(adjNum);
                unvisitedVertexNums.push(adjNum);
            }
        }
    }
}

//--------------------------------/Strongly Connected Components

//-------------------------------------Eulerian

/*
    Simple modification of DFS.
    Later we can get result by calling getEulerianCircuit or getEulerianPath method.
*/
EulerianBuilder::EulerianBuilder(UndirectedGraph graphCopy)
    : eulerianCircuitFound{false}, eulerianPathFound{false}
{
    Vertex::Number startVertexNumber = checkEulerianConditions(graphCopy);
    buildEulerian(graphCopy, startVertexNumber);
}

EulerianBuilder::EulerianBuilder(DirectedGraph graphCopy)
    : eulerianCircuitFound{false}, eulerianPathFound{false}
{
    Vertex::Number startVertexNumber = checkEulerianConditions(graphCopy);
    buildEulerian(graphCopy, startVertexNumber);
}

/*
    Undirected graph must have all vertices with even degrees for Eulerian circuit
    or 2 vertices with odd degrees for Eulerian path.
    Returns number of one of odd vertices(so we can start finding path from it).
*/
Vertex::Number EulerianBuilder::checkEulerianConditions(UndirectedGraph& graph)
{
    Vertex::Number verticesWithOddDegree = 0;
    Vertex::Number oddVertexNumber = 0;
    UndirectedGraph::VertexNumbers degrees;
    graph.getDegreesList(degrees);
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        if(degrees[i] % 2 == 1)
        {
            oddVertexNumber = i;
            ++verticesWithOddDegree;
        }
        if(verticesWithOddDegree > 2)
        {
            return oddVertexNumber; // false: have not nor path neither circuit
        }
    }
    if(verticesWithOddDegree == 2)
    {
        eulerianPathFound = true;
    }
    else if(verticesWithOddDegree == 0)
    {
        eulerianCircuitFound = true;
    }
    return oddVertexNumber;
}

/*
    Directed graph must have all vertices with outdeg = indeg for Eulerian circuit
    or 1 vertex with outdeg + 1 = indeg, and 1 with outdeg = indeg + 1.
    Returns number of the vertes which has outdeg + 1 = indeg(so we can start finding path from it).
*/
Vertex::Number EulerianBuilder::checkEulerianConditions(DirectedGraph& graph)
{
    Vertex::Number verticesWithDiffrentDegree = 0;
    Vertex::Number verticesWithOutdegPlus1EqualIndeg = 0;
    Vertex::Number verticesWithIndegPlus1EqualOutdeg = 0;
    Vertex::Number vertexWithGreaterOutdeg = 0;
    DirectedGraph::VertexNumbers indegs;
    DirectedGraph::VertexNumbers outdegs;
    DirectedGraph::VertexNumbers degrees;
    graph.getDegreesLists(indegs, outdegs, degrees);
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        Vertex::Number outdeg = outdegs[i];
        Vertex::Number indeg = indegs[i];
        if(outdeg != indeg)
        {
            ++verticesWithDiffrentDegree;
            if((indeg + 1) == outdeg)
            {
                verticesWithIndegPlus1EqualOutdeg += 1;
                vertexWithGreaterOutdeg = i;
            }
            else if((outdeg + 1) == indeg)
            {
                verticesWithOutdegPlus1EqualIndeg += 1;
            }
        }
        if(verticesWithOutdegPlus1EqualIndeg > 1 || verticesWithIndegPlus1EqualOutdeg > 1)  // have not nor circuit neither path
        {
            return vertexWithGreaterOutdeg;
        }
    }
    if(verticesWithDiffrentDegree == 0)
    {
        eulerianCircuitFound = true;
    }
    else if (verticesWithOutdegPlus1EqualIndeg == (verticesWithIndegPlus1EqualOutdeg == 1))
    {
        eulerianPathFound = true;
    }
    return vertexWithGreaterOutdeg;
}

void EulerianBuilder::buildEulerian(Graph& graph, Vertex::Number startVertexNumber)
{
    if(!eulerianCircuitFound && !eulerianPathFound)
    {
        return;
    }
    walkingStack.push(startVertexNumber);
    while(!walkingStack.empty())
    {
        Vertex::Number curVNum = walkingStack.top();
        std::vector<Vertex::Number> adjacentVertices = graph.getAdjacentVerticesFor(curVNum);
        if(adjacentVertices.size() != 0)
        {
            walkingStack.push(adjacentVertices[0]);
            graph.deleteEdgeBetween(curVNum, adjacentVertices[0]);
        }
        else
        {
            walkingStack.pop();
            eulerian.push_back(curVNum);
        }
    }
    std::reverse(eulerian.begin(), eulerian.end());
}

//-------------------------------------/Eulerian

//-------------------------------------Spanning Trees
/*
    Warning! Edges in input graph MUST have WeightAttrId attribute!!
    greedy algorithm that takes guarantees building of minimal spanning tree.
    Each time we take edge with minimal weight,
    and each time we uniting different components, so we will never have cycle.
    Reads edge attributes:
        weight
    Complexity: O(|E|lg|V|) ?
*/
UndirectedGraph minimalSpanningTreeKruskal(UndirectedGraph& graph)
{
    typedef PDisjointSet<Vertex::Number> VertexSet;
    typedef std::vector<VertexSet> VertexSets;
    typedef std::vector<const Edge*> pEdges;
    const Edge::AttributeId WeightAttrId = graph.getEdgeAttributeIdByName("weight");
    VertexSets vertexSets;
    UndirectedGraph spanningTree(graph.vertexCount(), graph.getInterfaceType());
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        vertexSets.push_back(makeSet(i));
    }
    pEdges graphEdges;
    graph.getAllEdges(graphEdges);
    countingSortTransformed(graphEdges.begin(), graphEdges.end(),
                            [WeightAttrId](const Edge* edge){ return edge->getAttribute(WeightAttrId); });
    for(auto edge: graphEdges)
    {
        VertexSet vs1 = vertexSets[edge->v1().number()];
        VertexSet vs2 = vertexSets[edge->v2().number()];
        if(findSet(vs1) != findSet(vs2))
        {
            spanningTree.addEdgeBetween(edge->v1().number(), edge->v2().number());
            Union(vs1, vs2);
        }
    }
    return spanningTree;
}

/*
    Like Kruskal, but builds spanning tree by vertices.
    Starts building from vertex number v.
    Reads edge attributes:
        weight
    Writes vertex attributes:
        key - used for building spanned tree
        parent
   Complexity: O(|E|lg|V|) ?
    (because sum of adj vertices for all vertices = e, and operations on set are lg|V|)
    WARNING: can we use vEB tree or fibonacci heaps?
    WARNING: DON'T TOUCH THIS HUGGING SHEET SUKA BLYAT!
*/
UndirectedGraph minimalSpanningTreePrim(UndirectedGraph& graph, Vertex::Number v)
{
    enum {InvalidNumber = -1};
    enum {NoParent = -1};
    const Edge::AttributeId WeightAttrId = graph.getEdgeAttributeIdByName("weight");
    const Edge::AttributeId KeyAttrId = graph.registerVertexAttributeIfNotRegistered("key");
    const Edge::AttributeId ParentAttrId = graph.registerVertexAttributeIfNotRegistered("parent");
    UndirectedGraph spanningTree(graph.vertexCount(), graph.getInterfaceType());
    const Vertex::AttributeType Inf = std::numeric_limits<Vertex::AttributeType>::max();
    // initializing vertices
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        graph.getVertexByNumber(i).setAttribute(KeyAttrId, Inf);
        graph.getVertexByNumber(i).setAttribute(ParentAttrId, NoParent);
    }
    graph.getVertexByNumber(v).setAttribute(KeyAttrId, 0);

    typedef std::pair<Vertex::AttributeType, Vertex::Number> WeightNumberPair;
    typedef std::set<WeightNumberPair> VertexHeap;
    VertexHeap vHeap;

    // filling heap
    for(int i = 0; i < graph.vertexCount(); ++i)
    {
        vHeap.insert(WeightNumberPair(graph.getVertexByNumber(i).getAttribute(KeyAttrId), i));
    }

    while(vHeap.size() != 0)
    {
        WeightNumberPair minWeightPair = *vHeap.begin();
        vHeap.erase(vHeap.begin());
        Vertex minWeightVertex = graph.getVertexByNumber(minWeightPair.second);
        // adding edge between cheapest vertex and its parent
        if(minWeightVertex.getAttribute(ParentAttrId) != NoParent)
        {
            spanningTree.addEdgeBetween(minWeightVertex.number(),minWeightVertex.getAttribute(ParentAttrId));
        }
        Graph::VertexNumbers adjVerticesNums = graph.getAdjacentVerticesFor(minWeightVertex.number());
        for(auto adjVNum : adjVerticesNums)
        {
            // searching this vertex in heap
            Vertex adjVFromHeap = graph.getVertexByNumber(adjVNum);
            // decreasing key
            WeightNumberPair adjPair(graph.getVertexByNumber(adjVNum).
                                     getAttribute(KeyAttrId), adjVNum);
            if(vHeap.find(adjPair) != vHeap.end() &&
                    graph.getEdgeByVertices(minWeightVertex.number(), adjVFromHeap.number()).getAttribute(WeightAttrId) <
                    adjVFromHeap.getAttribute(KeyAttrId))
            {
                Vertex::AttributeType newKey = graph.getEdgeByVertices(minWeightVertex.number(),
                                                                       adjVFromHeap.number()).getAttribute(WeightAttrId);
                graph.getVertexByNumber(adjVNum).setAttribute(ParentAttrId, minWeightVertex.number());
                adjVFromHeap.setAttribute(KeyAttrId, newKey);
                vHeap.erase(adjPair);
                vHeap.insert(WeightNumberPair(newKey, adjVNum));
                graph.getVertexByNumber(adjVNum).setAttribute(KeyAttrId, newKey);
            }
        }
    }
    return spanningTree;
}

//-------------------------------------/Spanning Trees

//-------------------------------------Shortest Paths

ShortestPathSearcher::ShortestPathSearcher(DirectedGraph* directedGraph)
    : ParentAttributeId{directedGraph->registerVertexAttributeIfNotRegistered("parent")},
      DistanceAttributeId{directedGraph->registerVertexAttributeIfNotRegistered("distance")},
      WeightAttributeId{directedGraph->registerEdgeAttributeIfNotRegistered("weight")},
      graph{directedGraph}
{}

/*
    Finds shortest path from source to other vertices.
    If graph has negative cycles, returns corresponding return code.
    This algorithm can process negative weights(unlike from Dijkstra).
    Reads vertex attributes:
        distance
    Reads edge attributes:
        weight
    Writes vertex attributes:
        distance
        parent
    Complexity: O(|V|*|E|)
*/
typename ShortestPathSearcher::BellmanFordReturnCode
    ShortestPathSearcher::bellmanFord(Vertex::Number source)
{
    typedef std::vector<const Edge*> pEdges;
    initializeSingleSource(source);
    pEdges edges;
    graph->getAllEdges(edges);
    // 1. Relaxing edges
    // < size - 1 is not a error, this is sufficient
    for(int i = 0; i < graph->vertexCount() - 1; ++i)
    {
        for(auto edge : edges)
        {
            relax(edge->v1().number(), edge->v2().number());
        }
    }
    // 2. searching for negative cycle
    for(auto edge: edges)
    {
        if(edge->v2().getAttribute(DistanceAttributeId) > edge->v1().getAttribute(DistanceAttributeId) +
                edge->getAttribute(WeightAttributeId))
        {
            return BellmanFordReturnCode::HasNegativeCycle;
        }
    }
    return BellmanFordReturnCode::Ok;
}

/*
    WARNING! Can be used only with acyclic directed graphs!
    Returns Vertex::Inf if vertex is unreachable from source.
    Uses properties of topological sort.
    Does paths finding for:
    Reads vertex attributes:
        distance
    Reads edge attributes:
        weight
    Writes vertex attributes:
        distance
        parent
    Complexity: O(|V| + |E|)
*/
void ShortestPathSearcher::dagShortestPaths(Vertex::Number source)
{
    typedef std::vector<Vertex> Vertices;
    typedef std::vector<Vertex::Number> VertexNums;
    Vertices sortedVertices;
    TopologicalSort<Vertices>{*graph, sortedVertices};
    initializeSingleSource(source);
    // can not process last vertex, because it will have outdeg = 0(last vertex after topological sort)
    for(size_t v = 0; v < sortedVertices.size() - 1; ++v)
    {
        VertexNums adjVertices = graph->getAdjacentVerticesFor(sortedVertices[v].number());
        for(Vertex::Number adjVNum : adjVertices)
        {
            relax(sortedVertices[v].number(), adjVNum);
        }
    }
}

/*
    Reads vertex attributes:
        distance
    Reads edge attributes:
        weight
    Writes vertex attributes:
        distance
        parent
    Complexity: O(|E|lg|V|) (if we take adjecent vertices for O(1))
*/
void ShortestPathSearcher::dijkstra(Vertex::Number source)
{
    typedef std::vector<Vertex::Number> VertexNums;
    typedef std::pair<Vertex::AttributeType, Vertex::Number> DistanceNumberPair;
    typedef std::set<DistanceNumberPair> VertexHeap;
    // initializing source
    initializeSingleSource(source);
    // adding vertices in queue with attribute 'distance' as key
    VertexHeap vHeap;
    for(int i = 0; i < graph->vertexCount(); ++i)
    {
        vHeap.insert(DistanceNumberPair(graph->getVertexByNumber(i).getAttribute(DistanceAttributeId), i));
    }
    // while queue is not empty
    while(vHeap.size() != 0)
    {
        // extracting minimum
        DistanceNumberPair minDNPair = *vHeap.begin();
        vHeap.erase(vHeap.begin());
        Vertex::Number minNumber = minDNPair.second;
        // finding adjacent vertices with minimum
        VertexNums adjacentVertexNums = graph->getAdjacentVerticesFor(minNumber);
        // updating adjacent vertices 'distance' attr: min(v.distance, u.distance + e.weight)
        for(Vertex::Number adjVN : adjacentVertexNums)
        {
            Vertex& adjV = graph->getVertexByNumber(adjVN);
            Vertex::AttributeType curDistance = adjV.getAttribute(DistanceAttributeId);
            Vertex::AttributeType newDistance = graph->getVertexByNumber(minNumber).getAttribute(DistanceAttributeId) +
                    graph->getEdgeByVertices(minNumber, adjVN).getAttribute(WeightAttributeId);
            DistanceNumberPair adjPair(graph->getVertexByNumber(adjVN).
                                       getAttribute(DistanceAttributeId), adjVN);
            if((vHeap.find(adjPair) != vHeap.end()) && (newDistance < curDistance))
            {
                vHeap.erase(adjPair);
                vHeap.insert(DistanceNumberPair(newDistance, adjVN));
                adjV.setAttribute(DistanceAttributeId, newDistance);
            }
        }
    }
}

/*
    Fills passed matrixes weights and parents.
    Complexity: O(|V|^3)
    Reads edge attributes:
        weight
*/
void ShortestPathSearcher::floydWarshall(Matrix& weights, Matrix& parents)
{
    doAdjacencyWeightMatrix(weights);
    fillParentsMatrixFromWeightsMatrix(weights, parents);
    for(size_t k = 0; k < weights.size(); ++k)
    {
        // d(ij,k) = w(ij), k = 0
        // min(d(ij, k-1), d(ik, k-1) + d(kj, k-1)
        for(size_t i = 0; i < weights.size(); ++i)
        {
            // min can not be bigger than inf
            if(weights[i][k] == WeightInf)
            {
                continue;
            }
            for(size_t j = 0; j < weights.size(); ++j)
            {
                // min can not be bigger than inf
                if(weights[k][j] == WeightInf)
                {
                    continue;
                }
                Vertex::AttributeType nextCandidate = weights[i][k] + weights[k][j];
                if(weights[i][j] > nextCandidate)
                {
                    weights[i][j] = nextCandidate;
                    parents[i][j] = parents[k][j];
                }
            }
        }
    }
}

/*
    Creates adjacency matrix with weights as elements.
    Reads edge attributes:
        weight
    Complexity: O(|V|^2)
*/
void ShortestPathSearcher::doAdjacencyWeightMatrix(Matrix& weightMatrix)
{
    for(Vertex::Number v1 = 0; v1 < graph->vertexCount(); ++v1)
    {
        weightMatrix.push_back(std::vector<Vertex::AttributeType>{});
        for(Vertex::Number v2 = 0; v2 < graph->vertexCount(); ++v2)
        {
            if(v1 == v2)
            {
                weightMatrix.back().push_back(0);
            }
            else if(!graph->areConnected(v1, v2))
            {
                weightMatrix.back().push_back(WeightInf);
            }
            else
            {
                weightMatrix.back().push_back(graph->getEdgeByVertices(v1, v2).getAttribute(WeightAttributeId));
            }
        }
    }
}

void ShortestPathSearcher::fillParentsMatrixFromWeightsMatrix(Matrix& weightsMatrix, Matrix& parentsMatrix)
{
    parentsMatrix = weightsMatrix;
    for(size_t row = 0; row < weightsMatrix.size(); ++row)
    {
        for(size_t col = 0; col < weightsMatrix.size(); ++col)
        {
            // case 1: row == col - path from itself to itself - min cost = 0, parent = NIL
            if(row == col)
            {
                parentsMatrix[row][col] = NoParent;
            }
            // case 2: weightsMatrix[row][col] == NIL - parent = NIL
            else if(weightsMatrix[row][col] == WeightInf)
            {
                parentsMatrix[row][col] = NoParent;
            }
            // case 3: parent = row
            else
            {
                parentsMatrix[row][col] = row;
            }
        }
    }
}

/*
    Helper procedure.
    Writes vertex attributes:
        distance
        parent
*/
void ShortestPathSearcher::initializeSingleSource(Vertex::Number source)
{
    for(Vertex::Number v = 0; v < graph->vertexCount(); ++v)
    {
        graph->getVertexByNumber(v).setAttribute(DistanceAttributeId, Vertex::Limits::Inf);
        graph->getVertexByNumber(v).setAttribute(ParentAttributeId, NoParent);
    }
    graph->getVertexByNumber(source).setAttribute(DistanceAttributeId, 0);
}

/*
    'Relaxes' edge between v1 and v2(Cormen, p.686)
    Reads vertex attributes:
        distance
    Reads edge attributes:
        weight
    Writes vertex attributes:
        distance
        parent
*/
void ShortestPathSearcher::relax(Vertex::Number v1, Vertex::Number v2)
{
    // inf + x = inf - so it has not sense to process next
    if(graph->getVertexByNumber(v1).getAttribute(DistanceAttributeId) == Vertex::Limits::Inf)
    {
        return;
    }
    if(graph->getVertexByNumber(v2).getAttribute(DistanceAttributeId) >
            (graph->getVertexByNumber(v1).getAttribute(DistanceAttributeId) +
            graph->getEdgeByVertices(v1, v2).getAttribute(WeightAttributeId)))
    {
        graph->getVertexByNumber(v2).setAttribute(DistanceAttributeId,
                                                 graph->getVertexByNumber(v1).getAttribute(DistanceAttributeId) +
                                                 graph->getEdgeByVertices(v1, v2).getAttribute(WeightAttributeId));
        graph->getVertexByNumber(v2).setAttribute(ParentAttributeId, v1);
    }
}

//-------------------------------------/Shortest Paths

//--------------------------------------Ford-Fulkerson

/*
    Searches biggest flow in network graph.
    If algorithm finds that "graph" is not a network, it throws GraphException.
    Checking network conditions:
        indeg(source) = 0
        outdeg(to) = 0
        if (u,v) in network, (v,u) is NOT in network.
    Writes vertex attributes:
        color
        parent
    Reads edge attributes:
        flow
        current
    Writes edge attributes:
        flow
        current
    Complexity: O(|E|^2|V|) - with BFS it is Edmond-Karp's algorithm ?
*/
FordFulkerson::FordFulkerson(DirectedGraph &graph, Vertex::Number source, Vertex::Number to)
    : ColorAttributeId{graph.registerVertexAttributeIfNotRegistered("color")},
      ParentAttributeId{graph.registerVertexAttributeIfNotRegistered("parent")},
      FlowAttributeId{graph.registerEdgeAttributeIfNotRegistered("flow")},
      CurrentAttributeId{graph.registerEdgeAttributeIfNotRegistered("current")}
{
    if(!isCorrectNetwork(graph, source, to))
    {
        throw GraphException("GraphAlgorithms::FordFulkerson(): passed graph is not a correct network!");
    }
    // initializing flows of each edge with 0
    pEdges edges;
    graph.getAllEdges(edges);
    for(auto edge: edges)
    {
        edge->setAttribute(FlowAttributeId, 0);
    }
    while(true)
    {
        DirectedGraph residualGraph = makeResidualGraph(graph);
        // searching if path from source to 'to' exists in residual graph
        BFS(residualGraph, source);
        if(residualGraph.getVertexByNumber(to).getAttribute(ColorAttributeId) == White)
        {
            break;
        }

        size_t pathCost = findPathCost(residualGraph, source, to);

        // updating graph's vertices parents(as in previous BFS)
        Vertex::Number curVNum = to;
        while(curVNum != source)
        {
            Vertex::AttributeType residualParentNum = residualGraph.getVertexByNumber(curVNum).getAttribute(ResidualParentAttributeId);
            graph.getVertexByNumber(curVNum).setAttribute(ParentAttributeId, residualParentNum);
            curVNum = residualParentNum;
        }

        updateFlows(graph, source, to, pathCost);
    }
}

/*
    Complexity: O(E)
*/
bool FordFulkerson::isCorrectNetwork(DirectedGraph& graph, Vertex::Number source, Vertex::Number to)
{
    // case 1: indeg(source) should be equal to 0
    if(graph.inDegree(source) != 0)
    {
        return false;
    }
    // case 2: outdeg(to) should be equal to 0
    if(graph.outDegree(to) != 0)
    {
        return false;
    }
    // case 3: if (u,v) in network, (v,u) is NOT in network.
    pEdges edges;
    graph.getAllEdges(edges);
    for(auto edge: edges)
    {
        if(graph.areConnected(edge->v2().number(), edge->v1().number()))
        {
            return false;
        }
    }
    return true;
}

/*
    Complexity: O(|E|)
*/
size_t FordFulkerson::findPathCost(DirectedGraph& graph, Vertex::Number source, Vertex::Number to)
{
    const size_t inf = std::numeric_limits<size_t>::max();
    size_t pathCost = inf;
    Vertex curV = graph.getVertexByNumber(to);
    while(curV.number() != source)
    {
        Vertex parent = graph.getVertexByNumber(curV.getAttribute(ResidualParentAttributeId));
        Edge& curEdge = graph.areConnected(parent.number(), curV.number()) ?
                    graph.getEdgeByVertices(parent.number(), curV.number()) :
                    graph.getEdgeByVertices(curV.number(), parent.number());
        pathCost = std::min(pathCost, (size_t)curEdge.getAttribute(ResidualCurrentAttributeId));
        curV = parent;
    }
    return pathCost;
}

/*
    Complexity: O(|E|)
*/
void FordFulkerson::updateFlows(DirectedGraph& graph, Vertex::Number source, Vertex::Number to, size_t pathCost)
{
    Vertex curV = graph.getVertexByNumber(to);
    while(curV.number() != source)
    {
        Vertex parent = graph.getVertexByNumber(curV.getAttribute(ParentAttributeId));
        // reverse edge
        if(graph.areConnected(curV.number(), parent.number()))
        {
            Edge& curEdge = graph.getEdgeByVertices(curV.number(), parent.number());
            Edge::AttributeType curFlow =  curEdge.getAttribute(FlowAttributeId);
            curEdge.setAttribute(FlowAttributeId, curFlow - pathCost);
        }
        // direct edge
        else
        {
            Edge& curEdge = graph.getEdgeByVertices(parent.number(), curV.number());
            Edge::AttributeType curFlow =  curEdge.getAttribute(FlowAttributeId);
            curEdge.setAttribute(FlowAttributeId, curFlow + pathCost);
        }
        curV = parent;
    }
}

/*
    Complexity: O(|E|)
*/
DirectedGraph FordFulkerson::makeResidualGraph(DirectedGraph& graph)
{
    pEdges edges;
    graph.getAllEdges(edges);
    DirectedGraph residualGraph(graph.vertexCount());
    ResidualCurrentAttributeId = residualGraph.registerEdgeAttributeIfNotRegistered("current");
    ResidualColorAttributeId = residualGraph.registerVertexAttributeIfNotRegistered("color");
    ResidualDistanceAttributeId = residualGraph.registerVertexAttributeIfNotRegistered("distance");
    ResidualParentAttributeId = residualGraph.registerVertexAttributeIfNotRegistered("parent");
    // cf(u,v) = c(u,v) - f(u,v), (u,v) E E,
    //           f(u,v), (v,u) E E,
    //           0, else
    for(auto edge: edges)
    {
        Edge::AttributeType residualCurrent = edge->getAttribute(CurrentAttributeId) - edge->getAttribute(FlowAttributeId);
        Edge::AttributeType residualAntiCurrent = edge->getAttribute(FlowAttributeId);
        if(residualCurrent)
        {
            residualGraph.addEdgeBetween(edge->v1().number(), edge->v2().number());
            Edge& newEdge = residualGraph.getEdgeByVertices(edge->v1().number(), edge->v2().number());
            newEdge.setAttribute(ResidualCurrentAttributeId, residualCurrent);
        }
        if(residualAntiCurrent)
        {
            residualGraph.addEdgeBetween(edge->v2().number(), edge->v1().number());
            Edge& newEdge = residualGraph.getEdgeByVertices(edge->v2().number(), edge->v1().number());
            newEdge.setAttribute(ResidualCurrentAttributeId, residualAntiCurrent);
        }
    }
    return residualGraph;
}

//--------------------------------------/Ford-Fulkerson
