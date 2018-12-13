#include <iostream>
#include "graphs.hpp"
#include "graphalgorithms.hpp"
#include "tests.hpp"

using namespace std;

void testPerformance()
{
    srand(time(0));
    std::cout << "Performance tests:\n";
    int n = 10000;
    int weightSpread = 100;
    UndirectedGraph ug(n);
    const Edge::AttributeId EdgeAttrId = ug.addEdgeAttribute("weight");
    for(int i = 0; i < n * 5; ++i)
    {
        Vertex::Number v1 = rand() % n;
        Vertex::Number v2 = rand() % n;
        ug.addEdgeBetween(v1, v2);
        ug.getEdgeByVertices(v1,v2).setAttribute(EdgeAttrId, rand() % weightSpread);
    }

    std::cout << "Euler:\n";
    clock_t before = clock();
    EulerianBuilder eb{ug};
    clock_t after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    std::cout << "BFS:\n";
    before = clock();
    BFS(ug, 0);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    std::cout << "Has cycle(uses DFS):\n";
    before = clock();
    hasCycle(ug);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    std::cout << "Components:\n";
    before = clock();
    UndirectedComponentsSplitter::Components c;
    UndirectedComponentsSplitter(ug, c);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    std::cout << "Kruskal spanning tree:\n";
    before = clock();
    UndirectedGraph spanning = minimalSpanningTreeKruskal(ug);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    std::cout << "Prim spanning tree:\n";
    before = clock();
    UndirectedGraph spanning2 = minimalSpanningTreePrim(ug);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    DirectedGraph g(n);
    const Edge::AttributeId EdgeAttrId2 = g.addEdgeAttribute("weight");
    for(int i = 0; i < n * 5; ++i)
    {
        Vertex::Number v1 = rand() % n;
        Vertex::Number v2 = rand() % n;
        g.addEdgeBetween(v1, v2);
        g.getEdgeByVertices(v1, v2).setAttribute(EdgeAttrId2, rand() % weightSpread);
    }

    std::cout << "Directed graph strongly connected components:\n";
    before = clock();
    StronglyConnectedComponents::Components comp;
    StronglyConnectedComponents(g, comp);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    // SLOW SHIT
    /*std::cout << "Bellman-Ford:\n";
    before = clock();
    ShortestPathSearcher::bellmanFord(g, rand() % n);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;*/

    std::cout << "DAC:\n";
    before = clock();
    ShortestPathSearcher searcher(&g);
    searcher.dagShortestPaths(rand() % n);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    std::cout << "Dijkstra:\n";
    before = clock();
    searcher.dijkstra(rand() % n);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;

    // VERY SLOW SHIT(O(V^3))
    /*std::cout << "Floyd-Warshall:\n";
    before = clock();
    ShortestPathSearcher::Matrix weights;
    ShortestPathSearcher::Matrix parents;
    ShortestPathSearcher::floydWarshall(g, weights, parents);
    after = clock();
    std::cout << "Elapsed: " << (after - before) << std::endl;*/

}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    int err = RUN_ALL_TESTS();
    if(err)
    {
        std::cout << "\nError in tests\n";
        return err;
    }
    std::cout << "\nTests completed\n";

    testPerformance();


    return 0;
}
