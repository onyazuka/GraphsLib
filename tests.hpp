#include <gtest/gtest.h>
#include "graphs.hpp"
#include "graphalgorithms.hpp"
#include "utils.hpp"

TEST(TestCountingSort, UtilsTests)
{
    std::vector<int> vi{2,1,3};
    countingSort<std::vector<int>::iterator>(vi.begin(), vi.end(), 3);
    EXPECT_EQ(vi[0], 1);
    EXPECT_EQ(vi[1], 2);
    EXPECT_EQ(vi[2], 3);

    std::vector<int> vi2{2,1,3,8,5};
    countingSort(vi2.begin(), vi2.end());
    EXPECT_EQ(vi2[0], 1);
    EXPECT_EQ(vi2[1], 2);
    EXPECT_EQ(vi2[2], 3);
    EXPECT_EQ(vi2[3], 5);
    EXPECT_EQ(vi2[4], 8);

    struct A
    {
        int a;
        int b;
        bool operator==(const A& other)
        {
            return a == other.a && b == other.b;
        }
    };
    A a1;
    a1.a = 2;
    a1.b = 7;
    A a2;
    a2.a = 1;
    a2.b = 6;
    A a3;
    a3.a = 3;
    a3.b = 4;
    std::vector<A> va{a1,a2,a3};
    countingSortTransformed(va.begin(), va.end(), [](const A& a){return a.a;});
    std::vector<A> expectedRes1{a2,a1,a3};
    EXPECT_TRUE(std::equal(va.begin(), va.end(), expectedRes1.begin()));

    std::vector<A> va2{a1,a2,a3};
    countingSortTransformed(va2.begin(), va2.end(), [](const A& a){return a.b;});
    std::vector<A> expectedRes2{a3,a2,a1};
    EXPECT_TRUE(std::equal(va2.begin(), va2.end(), expectedRes2.begin()));

}

TEST(GraphTotalTests, GraphTests)
{
    // Undirected graph + adjacency lists
    UndirectedGraph ug(5, UndirectedGraph::Interfaces::AdjacencyLists);
    EXPECT_FALSE(ug.isDirected());
    EXPECT_EQ(ug.vertexCount(), 5);
    ug.addVertex();
    EXPECT_EQ(ug.vertexCount(), 6);
    EXPECT_FALSE(ug.areConnected(0,1));
    EXPECT_EQ(ug.degree(0), 0);
    ug.addEdgeBetween(0,1);
    EXPECT_EQ(ug.degree(0), 1);
    EXPECT_TRUE(ug.areConnected(0,1));
    EXPECT_TRUE(ug.areConnected(1,0));
    EXPECT_FALSE(ug.areConnected(1,2));
    EXPECT_THROW(ug.getEdgeByVertices(1,2), std::out_of_range);
    EXPECT_THROW(ug.getEdgeByVertices(3,6), std::out_of_range);
    ug.addVertex();
    ug.addEdgeBetween(3,6);
    EXPECT_TRUE(ug.areConnected(3,6));
    EXPECT_TRUE(ug.areConnected(6,3));
    EXPECT_NO_THROW(ug.getEdgeByVertices(3,6));
    Edge e1 = ug.getEdgeByVertices(3,6);
    EXPECT_EQ(e1.v1().number(), 3);
    EXPECT_EQ(e1.v2().number(), 6);

    EXPECT_THROW(ug.getVertexByNumber(-1), std::out_of_range);
    EXPECT_THROW(ug.getVertexByNumber(7), std::out_of_range);
    EXPECT_EQ(ug.getVertexByNumber(4).number(), 4);
    EXPECT_EQ(ug.edgesCount(), 2);
    EXPECT_THROW(ug.addEdgeBetween(10,11), std::out_of_range);
    EXPECT_THROW(ug.areConnected(-1,2), std::out_of_range);

    ug.deleteEdgeBetween(0,1);
    EXPECT_FALSE(ug.areConnected(0, 1));
    EXPECT_FALSE(ug.areConnected(1, 0));
    EXPECT_TRUE(ug.areConnected(3,6));
    ug.addEdgeBetween(2,3);
    EXPECT_EQ(ug.degree(3), 2);
    EXPECT_TRUE(ug.areConnected(2,3));
    //ug.deleteVertex();



    // Directed graph + adjacency matrix
    DirectedGraph dg(100, UndirectedGraph::Interfaces::AdjacencyMatrix);
    EXPECT_TRUE(dg.isDirected());
    EXPECT_EQ(dg.vertexCount(), 100);
    dg.addVertex();
    EXPECT_EQ(dg.vertexCount(), 101);
    EXPECT_FALSE(dg.areConnected(0,1));
    dg.addEdgeBetween(0,1);
    EXPECT_EQ(dg.degree(0), 1);
    EXPECT_EQ(dg.inDegree(0), 0);
    EXPECT_EQ(dg.outDegree(0), 1);
    EXPECT_EQ(dg.degree(1), 1);
    EXPECT_EQ(dg.inDegree(1), 1);
    EXPECT_EQ(dg.outDegree(1), 0);
    EXPECT_TRUE(dg.areConnected(0,1));
    EXPECT_FALSE(dg.areConnected(1,0));
    EXPECT_FALSE(dg.areConnected(1,2));
    EXPECT_THROW(dg.getEdgeByVertices(1,2), std::out_of_range);
    EXPECT_THROW(dg.getEdgeByVertices(3,6), std::out_of_range);
    dg.addVertex();
    dg.addEdgeBetween(3,6);
    EXPECT_TRUE(dg.areConnected(3,6));
    EXPECT_FALSE(dg.areConnected(6,3));
    EXPECT_NO_THROW(dg.getEdgeByVertices(3,6));
    Edge e2 = dg.getEdgeByVertices(3,6);
    EXPECT_EQ(e2.v1().number(), 3);
    EXPECT_EQ(e2.v2().number(), 6);

    EXPECT_THROW(dg.getVertexByNumber(-1), std::out_of_range);
    EXPECT_THROW(dg.getVertexByNumber(102), std::out_of_range);
    EXPECT_NO_THROW(dg.getVertexByNumber(101));
    EXPECT_EQ(dg.getVertexByNumber(4).number(), 4);
    EXPECT_EQ(dg.edgesCount(), 2);
    EXPECT_THROW(dg.areConnected(-1,2), std::out_of_range);
    dg.addEdgeBetween(100,101);
    EXPECT_TRUE(dg.areConnected(100, 101));

    EXPECT_FALSE(dg.areConnected(101,100));
    dg.deleteEdgeBetween(100,101);
    EXPECT_FALSE(dg.areConnected(100, 101));
    EXPECT_FALSE(dg.areConnected(101, 100));
    EXPECT_TRUE(dg.areConnected(3,6));
    dg.addEdgeBetween(2,3);
    EXPECT_EQ(dg.degree(3), 2);
    EXPECT_EQ(dg.inDegree(3), 1);
    EXPECT_EQ(dg.outDegree(3), 1);
    EXPECT_TRUE(dg.areConnected(2,3));

    dg.transpose();
    EXPECT_FALSE(dg.areConnected(2,3));
    EXPECT_TRUE(dg.areConnected(3,2));
    EXPECT_FALSE(dg.areConnected(3,6));
    EXPECT_TRUE(dg.areConnected(6,3));

    // directed graph - adjacency lists
    DirectedGraph dgal(10, DirectedGraph::Interfaces::AdjacencyLists);
    dgal.addEdgeBetween(5,6);
    dgal.addEdgeBetween(0, 9);

    EXPECT_TRUE(dgal.areConnected(5,6));
    EXPECT_FALSE(dgal.areConnected(6,5));
    EXPECT_TRUE(dgal.areConnected(0,9));
    EXPECT_FALSE(dgal.areConnected(9,0));
    dgal.transpose();
    EXPECT_TRUE(dgal.areConnected(6,5));
    EXPECT_FALSE(dgal.areConnected(5,6));
    EXPECT_TRUE(dgal.areConnected(9,0));
    EXPECT_FALSE(dgal.areConnected(0,9));


}

TEST(AdjacentEdgesVectorTest, GraphTests)
{
    UndirectedGraph ug(10, UndirectedGraph::Interfaces::AdjacencyLists);
    ug.addEdgeBetween(3,4);
    ug.addEdgeBetween(3,6);
    ug.addEdgeBetween(3,7);
    Graph::VertexNumbers adjeug3 = ug.getAdjacentVerticesFor(3);
    EXPECT_EQ(adjeug3.size(), 3);
    Graph::VertexNumbers adjeug7 = ug.getAdjacentVerticesFor(7);
    EXPECT_EQ(adjeug7.size(), 1);
    Graph::VertexNumbers adjeug0 = ug.getAdjacentVerticesFor(0);
    EXPECT_EQ(adjeug0.size(), 0);
    EXPECT_THROW(ug.getAdjacentVerticesFor(11), std::out_of_range);

    DirectedGraph dg(10, UndirectedGraph::Interfaces::AdjacencyMatrix);
    dg.addEdgeBetween(3,4);
    dg.addEdgeBetween(3,6);
    dg.addEdgeBetween(3,7);
    Graph::VertexNumbers adjedg3 = dg.getAdjacentVerticesFor(3);
    EXPECT_EQ(adjedg3.size(), 3);
    Graph::VertexNumbers adjedg7 = dg.getAdjacentVerticesFor(7);
    EXPECT_EQ(adjedg7.size(), 0);
    Graph::VertexNumbers adjedg0 = dg.getAdjacentVerticesFor(0);
    EXPECT_EQ(adjedg0.size(), 0);
    EXPECT_THROW(dg.getAdjacentVerticesFor(11), std::out_of_range);
}

TEST(BFSTest, GraphAlgorithmsTests)
{
    UndirectedGraph ug(8, UndirectedGraph::Interfaces::AdjacencyLists);
    ug.addEdgeBetween(0, 1);
    ug.addEdgeBetween(1,2);
    ug.addEdgeBetween(2,3);
    ug.addEdgeBetween(3,4);
    ug.addEdgeBetween(3,5);
    ug.addEdgeBetween(4,5);
    ug.addEdgeBetween(4,6);
    ug.addEdgeBetween(5,6);
    ug.addEdgeBetween(5,7);
    ug.addEdgeBetween(6,7);
    BFS(ug, 2);
    const Vertex::AttributeId DistanceAttrId = ug.getVertexAttributeIdByName("distance");
    EXPECT_EQ(ug.getVertexByNumber(0).getAttribute(DistanceAttrId), 2);
    EXPECT_EQ(ug.getVertexByNumber(1).getAttribute(DistanceAttrId), 1);
    EXPECT_EQ(ug.getVertexByNumber(2).getAttribute(DistanceAttrId), 0);
    EXPECT_EQ(ug.getVertexByNumber(3).getAttribute(DistanceAttrId), 1);
    EXPECT_EQ(ug.getVertexByNumber(4).getAttribute(DistanceAttrId), 2);
    EXPECT_EQ(ug.getVertexByNumber(5).getAttribute(DistanceAttrId), 2);
    EXPECT_EQ(ug.getVertexByNumber(6).getAttribute(DistanceAttrId), 3);
    EXPECT_EQ(ug.getVertexByNumber(7).getAttribute(DistanceAttrId), 3);

    DirectedGraph dg(6, DirectedGraph::Interfaces::AdjacencyMatrix);
    dg.addEdgeBetween(0,1);
    dg.addEdgeBetween(0,3);
    dg.addEdgeBetween(1,4);
    dg.addEdgeBetween(2,4);
    dg.addEdgeBetween(2,5);
    dg.addEdgeBetween(3,1);
    dg.addEdgeBetween(4,3);
    dg.addEdgeBetween(5,5);
    BFS(dg, 2);
    const Vertex::AttributeId DistanceAttrId1 = ug.getVertexAttributeIdByName("distance");
    const Vertex::AttributeId ParentAttrId1 = ug.getVertexAttributeIdByName("parent");
    EXPECT_EQ(dg.getVertexByNumber(0).getAttribute(DistanceAttrId1), -1);
    EXPECT_EQ(dg.getVertexByNumber(0).getAttribute(ParentAttrId1), -1);
    EXPECT_EQ(dg.getVertexByNumber(1).getAttribute(DistanceAttrId1), 3);
    EXPECT_EQ(dg.getVertexByNumber(1).getAttribute(ParentAttrId1), 3);
    EXPECT_EQ(dg.getVertexByNumber(2).getAttribute(DistanceAttrId1), 0);
    EXPECT_EQ(dg.getVertexByNumber(2).getAttribute(ParentAttrId1), -1);
    EXPECT_EQ(dg.getVertexByNumber(3).getAttribute(DistanceAttrId1), 2);
    EXPECT_EQ(dg.getVertexByNumber(3).getAttribute(ParentAttrId1), 4);
    EXPECT_EQ(dg.getVertexByNumber(4).getAttribute(DistanceAttrId1), 1);
    EXPECT_EQ(dg.getVertexByNumber(4).getAttribute(ParentAttrId1), 2);
    EXPECT_EQ(dg.getVertexByNumber(5).getAttribute(DistanceAttrId1), 1);
    EXPECT_EQ(dg.getVertexByNumber(5).getAttribute(ParentAttrId1), 2);
}

TEST(DFSTest, GraphAlgorithmsTests)
{
    DirectedGraph dg(6, DirectedGraph::Interfaces::AdjacencyMatrix);
    dg.addEdgeBetween(0,1);
    dg.addEdgeBetween(0,3);
    dg.addEdgeBetween(1,4);
    dg.addEdgeBetween(2,4);
    dg.addEdgeBetween(2,5);
    dg.addEdgeBetween(3,1);
    dg.addEdgeBetween(4,3);
    dg.addEdgeBetween(5,5);

    DFS{dg};
    const Vertex::AttributeId DistanceAttrId = dg.getVertexAttributeIdByName("distance");
    const Vertex::AttributeId FinishAttrId = dg.getVertexAttributeIdByName("finish");
    EXPECT_EQ(dg.getVertexByNumber(0).getAttribute(DistanceAttrId), 1);
    EXPECT_EQ(dg.getVertexByNumber(0).getAttribute(FinishAttrId), 8);
    EXPECT_EQ(dg.getVertexByNumber(1).getAttribute(DistanceAttrId), 3);
    EXPECT_EQ(dg.getVertexByNumber(1).getAttribute(FinishAttrId), 6);
    EXPECT_EQ(dg.getVertexByNumber(2).getAttribute(DistanceAttrId), 9);
    EXPECT_EQ(dg.getVertexByNumber(2).getAttribute(FinishAttrId), 12);
    EXPECT_EQ(dg.getVertexByNumber(3).getAttribute(DistanceAttrId), 2);
    EXPECT_EQ(dg.getVertexByNumber(3).getAttribute(FinishAttrId), 7);
    EXPECT_EQ(dg.getVertexByNumber(4).getAttribute(DistanceAttrId), 4);
    EXPECT_EQ(dg.getVertexByNumber(4).getAttribute(FinishAttrId), 5);
    EXPECT_EQ(dg.getVertexByNumber(5).getAttribute(DistanceAttrId), 10);
    EXPECT_EQ(dg.getVertexByNumber(5).getAttribute(FinishAttrId), 11);

    DirectedGraph dg1(10, DirectedGraph::Interfaces::AdjacencyLists);
    dg1.addEdgeBetween(0,2);
    dg1.addEdgeBetween(2,5);
    dg1.addEdgeBetween(5,6);
    dg1.addEdgeBetween(6,2);
    dg1.addEdgeBetween(0,3);
    dg1.addEdgeBetween(3,7);
    dg1.addEdgeBetween(7,9);
    dg1.addEdgeBetween(9,7);
    dg1.addEdgeBetween(3,8);
    dg1.addEdgeBetween(0,6);
    dg1.addEdgeBetween(8,0);
    dg1.addEdgeBetween(1,4);
    dg1.addEdgeBetween(4,8);
    dg1.addEdgeBetween(1,8);

    DFS{dg1};
    const Vertex::AttributeId DistanceAttrId1 = dg.getVertexAttributeIdByName("distance");
    const Vertex::AttributeId FinishAttrId1 = dg.getVertexAttributeIdByName("finish");
    const Edge::AttributeId TypeAttrId1 = dg.getEdgeAttributeIdByName("type");
    EXPECT_EQ(dg1.getVertexByNumber(3).getAttribute(DistanceAttrId1), 2);
    EXPECT_EQ(dg1.getVertexByNumber(3).getAttribute(FinishAttrId1), 9);
    EXPECT_EQ(dg1.getVertexByNumber(7).getAttribute(DistanceAttrId1), 3);
    EXPECT_EQ(dg1.getVertexByNumber(7).getAttribute(FinishAttrId1), 6);
    EXPECT_EQ(dg1.getVertexByNumber(0).getAttribute(DistanceAttrId1), 1);
    EXPECT_EQ(dg1.getVertexByNumber(0).getAttribute(FinishAttrId1), 16);
    EXPECT_EQ(dg1.getEdgeByVertices(0, 2).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(2, 5).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(5, 6).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(6, 2).getAttribute(TypeAttrId1), DFS::EdgeTypes::Back);
    EXPECT_EQ(dg1.getEdgeByVertices(0, 3).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(3, 7).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(7, 9).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(9, 7).getAttribute(TypeAttrId1), DFS::EdgeTypes::Back);
    EXPECT_EQ(dg1.getEdgeByVertices(3, 8).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(0, 6).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(8, 0).getAttribute(TypeAttrId1), DFS::EdgeTypes::Back);
    EXPECT_EQ(dg1.getEdgeByVertices(1, 4).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(dg1.getEdgeByVertices(4, 8).getAttribute(TypeAttrId1), DFS::EdgeTypes::Cross);
    EXPECT_EQ(dg1.getEdgeByVertices(1, 8).getAttribute(TypeAttrId1), DFS::EdgeTypes::Cross);

    UndirectedGraph ugsm(6);
    ugsm.addEdgeBetween(0,1);
    ugsm.addEdgeBetween(0,2);
    ugsm.addEdgeBetween(1,4);
    ugsm.addEdgeBetween(4,2);
    ugsm.addEdgeBetween(2,3);
    ugsm.addEdgeBetween(2,5);
    ugsm.addEdgeBetween(3,5);

    DFS{ugsm};
    EXPECT_EQ(ugsm.getVertexByNumber(0).getAttribute(DistanceAttrId1), 1);
    EXPECT_EQ(ugsm.getVertexByNumber(0).getAttribute(FinishAttrId1), 12);
    EXPECT_EQ(ugsm.getVertexByNumber(4).getAttribute(DistanceAttrId1), 3);
    EXPECT_EQ(ugsm.getVertexByNumber(4).getAttribute(FinishAttrId1), 10);
    EXPECT_EQ(ugsm.getEdgeByVertices(2,3).getAttribute(TypeAttrId1), DFS::EdgeTypes::Tree);
    EXPECT_EQ(ugsm.getEdgeByVertices(2,5).getAttribute(TypeAttrId1), DFS::EdgeTypes::Back);
}

TEST(TopologicalSortTest, GraphAlgorithmsTests)
{
    DirectedGraph dg(14, DirectedGraph::Interfaces::AdjacencyLists);
    dg.addEdgeBetween(0,4);
    dg.addEdgeBetween(0,5);
    dg.addEdgeBetween(0,11);
    dg.addEdgeBetween(1,4);
    dg.addEdgeBetween(1,8);
    dg.addEdgeBetween(1,2);
    dg.addEdgeBetween(2,5);
    dg.addEdgeBetween(2,6);
    dg.addEdgeBetween(2,9);
    dg.addEdgeBetween(3,2);
    dg.addEdgeBetween(3,5);
    dg.addEdgeBetween(3,13);
    dg.addEdgeBetween(4,7);
    dg.addEdgeBetween(5,8);
    dg.addEdgeBetween(5,12);
    dg.addEdgeBetween(6,5);
    dg.addEdgeBetween(8,7);
    dg.addEdgeBetween(9,11);
    dg.addEdgeBetween(9,10);
    dg.addEdgeBetween(10,13);
    dg.addEdgeBetween(12,9);

    std::vector<Vertex> sortedVertices;
    TopologicalSort<std::vector<Vertex>>(dg, sortedVertices);
    // RIGHT topological order
    std::vector<Vertex> expectedRes {3,1,2,6,0,4,5,12,9,10,13,11,8,7};
    EXPECT_TRUE(std::equal(sortedVertices.begin(), sortedVertices.end(), expectedRes.begin(),
                           [](const Vertex& v1, const Vertex& v2) {return v1.number() == v2.number();}));
    EXPECT_FALSE(hasCycle(dg));
}

TEST(hasCycleTest, GraphAlgorithmsTests)
{
    UndirectedGraph udg(5);
    udg.addEdgeBetween(0,2);
    udg.addEdgeBetween(1,3);
    udg.addEdgeBetween(2,4);
    udg.addEdgeBetween(4,0);
    EXPECT_TRUE(hasCycle(udg));

    DirectedGraph dg(5);
    dg.addEdgeBetween(0,2);
    dg.addEdgeBetween(1,3);
    dg.addEdgeBetween(2,4);
    dg.addEdgeBetween(4,3);
    EXPECT_FALSE(hasCycle(dg));
}

TEST(StronglyConnectedComponentsTest, GraphAlgorithmsTests)
{
    DirectedGraph dg1(10, DirectedGraph::Interfaces::AdjacencyMatrix);
    dg1.addEdgeBetween(0,2);
    dg1.addEdgeBetween(2,5);
    dg1.addEdgeBetween(5,6);
    dg1.addEdgeBetween(6,2);
    dg1.addEdgeBetween(0,3);
    dg1.addEdgeBetween(3,7);
    dg1.addEdgeBetween(7,9);
    dg1.addEdgeBetween(9,7);
    dg1.addEdgeBetween(3,8);
    dg1.addEdgeBetween(0,6);
    dg1.addEdgeBetween(8,0);
    dg1.addEdgeBetween(1,4);
    dg1.addEdgeBetween(4,8);
    dg1.addEdgeBetween(1,8);

    StronglyConnectedComponents::Components scdg;
    StronglyConnectedComponents(dg1, scdg);

    EXPECT_EQ(scdg.size(), 5);
    EXPECT_EQ(scdg[0].size(), 1);
    EXPECT_EQ(scdg[1].size(), 1);
    EXPECT_EQ(scdg[2].size(), 3);
    EXPECT_EQ(scdg[3].size(), 2);
    EXPECT_EQ(scdg[4].size(), 3);

    EXPECT_EQ(scdg[0][0], 1);
    EXPECT_EQ(scdg[1][0], 4);
    EXPECT_EQ(scdg[2][0], 0);
    EXPECT_EQ(scdg[2][1], 8);
    EXPECT_EQ(scdg[2][2], 3);
    EXPECT_EQ(scdg[3][0], 7);
    EXPECT_EQ(scdg[3][1], 9);
    EXPECT_EQ(scdg[4][0], 6);
    EXPECT_EQ(scdg[4][1], 5);
    EXPECT_EQ(scdg[4][2], 2);
}

TEST(UndirectedComponentsSplitterTest, GraphAlgorithmsTests)
{
    UndirectedGraph ug(10);
    ug.addEdgeBetween(0,1);
    ug.addEdgeBetween(0,2);
    ug.addEdgeBetween(1,2);
    ug.addEdgeBetween(1,3);
    ug.addEdgeBetween(4,5);
    ug.addEdgeBetween(4,6);
    ug.addEdgeBetween(7,8);
    UndirectedComponentsSplitter::Components components;
    UndirectedComponentsSplitter(ug, components);
    EXPECT_EQ(components.size(), 4);
    EXPECT_TRUE(std::find(components[0].begin(), components[0].end(), 0) != components[0].end());
    EXPECT_TRUE(std::find(components[0].begin(), components[0].end(), 1) != components[0].end());
    EXPECT_TRUE(std::find(components[0].begin(), components[0].end(), 2) != components[0].end());
    EXPECT_TRUE(std::find(components[0].begin(), components[0].end(), 3) != components[0].end());
    EXPECT_TRUE(std::find(components[1].begin(), components[1].end(), 4) != components[1].end());
    EXPECT_TRUE(std::find(components[1].begin(), components[1].end(), 5) != components[1].end());
    EXPECT_TRUE(std::find(components[1].begin(), components[1].end(), 6) != components[1].end());
    EXPECT_TRUE(std::find(components[2].begin(), components[2].end(), 7) != components[2].end());
    EXPECT_TRUE(std::find(components[2].begin(), components[2].end(), 8) != components[2].end());
    EXPECT_TRUE(std::find(components[3].begin(), components[3].end(), 9) != components[3].end());

    UndirectedGraph udg(8);
    udg.addEdgeBetween(0,1);
    udg.addEdgeBetween(0,3);
    udg.addEdgeBetween(1,2);
    udg.addEdgeBetween(1,4);
    udg.addEdgeBetween(1,5);
    udg.addEdgeBetween(2,1);
    udg.addEdgeBetween(2,3);
    udg.addEdgeBetween(2,4);
    udg.addEdgeBetween(2,6);
    udg.addEdgeBetween(3,0);
    udg.addEdgeBetween(3,2);
    udg.addEdgeBetween(3,6);
    udg.addEdgeBetween(4,1);
    udg.addEdgeBetween(4,2);
    udg.addEdgeBetween(4,7);
    udg.addEdgeBetween(5,1);
    udg.addEdgeBetween(5,3);
    udg.addEdgeBetween(5,4);
    udg.addEdgeBetween(5,6);
    udg.addEdgeBetween(6,3);
    udg.addEdgeBetween(6,5);
    udg.addEdgeBetween(6,7);
    udg.addEdgeBetween(7,4);
    udg.addEdgeBetween(7,6);

    UndirectedComponentsSplitter::Components components1;
    UndirectedComponentsSplitter(udg, components1);
    EXPECT_EQ(components1.size(), 1);
}


TEST(EulerTests, GraphAlgorithmsTests)
{
    DirectedGraph dg1(10, DirectedGraph::Interfaces::AdjacencyMatrix);
    dg1.addEdgeBetween(0,2);
    dg1.addEdgeBetween(2,5);
    dg1.addEdgeBetween(5,6);
    dg1.addEdgeBetween(6,2);
    dg1.addEdgeBetween(0,3);
    dg1.addEdgeBetween(3,7);
    dg1.addEdgeBetween(7,9);
    dg1.addEdgeBetween(9,7);
    dg1.addEdgeBetween(3,8);
    dg1.addEdgeBetween(0,6);
    dg1.addEdgeBetween(8,0);
    dg1.addEdgeBetween(1,4);
    dg1.addEdgeBetween(4,8);
    dg1.addEdgeBetween(1,8);

    EulerianBuilder eb(dg1);
    EXPECT_FALSE(eb.hasEulerianCircuit());
    EXPECT_FALSE(eb.hasEulerianPath());

    UndirectedGraph udg(8);
    udg.addEdgeBetween(0,1);
    udg.addEdgeBetween(0,3);
    udg.addEdgeBetween(1,2);
    udg.addEdgeBetween(1,4);
    udg.addEdgeBetween(1,5);
    udg.addEdgeBetween(2,1);
    udg.addEdgeBetween(2,3);
    udg.addEdgeBetween(2,4);
    udg.addEdgeBetween(2,6);
    udg.addEdgeBetween(3,0);
    udg.addEdgeBetween(3,2);
    udg.addEdgeBetween(3,6);
    udg.addEdgeBetween(4,1);
    udg.addEdgeBetween(4,2);
    udg.addEdgeBetween(4,7);
    udg.addEdgeBetween(5,1);
    udg.addEdgeBetween(5,3);
    udg.addEdgeBetween(5,4);
    udg.addEdgeBetween(5,6);
    udg.addEdgeBetween(6,3);
    udg.addEdgeBetween(6,5);
    udg.addEdgeBetween(6,7);
    udg.addEdgeBetween(7,4);
    udg.addEdgeBetween(7,6);
    EulerianBuilder ebu(udg);

    EXPECT_TRUE(ebu.hasEulerianCircuit());
    EulerianBuilder::Eulerian circuit = ebu.getEulerianCircuit();
    EulerianBuilder::Eulerian check{0,3,2,1,5,3,6,2,4,5,6,7,4,1,0};
    EXPECT_TRUE(std::equal(check.begin(), check.end(), circuit.begin()));

    DirectedGraph dg2(4);
    dg2.addEdgeBetween(0,1);
    dg2.addEdgeBetween(0,3);
    dg2.addEdgeBetween(1,2);
    dg2.addEdgeBetween(2,0);
    dg2.addEdgeBetween(3,2);

    EulerianBuilder ebd{dg2};
    EXPECT_FALSE(ebd.hasEulerianCircuit());
    EXPECT_TRUE(ebd.hasEulerianPath());

    EulerianBuilder::Eulerian path = ebd.getEulerianPath();
    EulerianBuilder::Eulerian check1{0,3,2,0,1,2};
    EXPECT_TRUE(std::equal(check1.begin(), check1.end(), path.begin()));
}

TEST(MinimalSpanningTreeTests, GraphAlgorithmsTests)
{
    // Kruskal

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
    EXPECT_EQ(spanningTree.edgesCount(), 8);
    EXPECT_TRUE(spanningTree.areConnected(0,1));
    EXPECT_TRUE(spanningTree.areConnected(0,7));
    EXPECT_TRUE(spanningTree.areConnected(6,7));
    EXPECT_TRUE(spanningTree.areConnected(2,8));
    EXPECT_TRUE(spanningTree.areConnected(2,3));
    EXPECT_TRUE(spanningTree.areConnected(2,5));
    EXPECT_TRUE(spanningTree.areConnected(3,4));
    EXPECT_TRUE(spanningTree.areConnected(5,6));

    // Prim
    UndirectedGraph spanningTree2 = minimalSpanningTreePrim(ug, 0);
    EXPECT_EQ(spanningTree2.edgesCount(), 8);
    EXPECT_TRUE(spanningTree2.areConnected(0,1));
    EXPECT_TRUE(spanningTree2.areConnected(1,2));
    EXPECT_TRUE(spanningTree2.areConnected(6,7));
    EXPECT_TRUE(spanningTree2.areConnected(2,8));
    EXPECT_TRUE(spanningTree2.areConnected(2,3));
    EXPECT_TRUE(spanningTree2.areConnected(2,5));
    EXPECT_TRUE(spanningTree2.areConnected(3,4));
    EXPECT_TRUE(spanningTree2.areConnected(5,6));
}

TEST(ShortestPathTests, GraphAlgorithmsTests)
{
    // Bellman-Ford
    DirectedGraph dg(5, DirectedGraph::Interfaces::AdjacencyMatrix);
    dg.addEdgeBetween(0, 1);
    dg.addEdgeBetween(1, 2);
    dg.addEdgeBetween(2, 1);
    dg.addEdgeBetween(0, 3);
    dg.addEdgeBetween(1, 3);
    dg.addEdgeBetween(1, 4);
    dg.addEdgeBetween(3, 2);
    dg.addEdgeBetween(3, 4);
    dg.addEdgeBetween(4, 2);
    dg.addEdgeBetween(4, 0);
    const Edge::AttributeId WeightAttrId = dg.addEdgeAttribute("weight");
    dg.getEdgeByVertices(0,1).setAttribute(WeightAttrId, 6);
    dg.getEdgeByVertices(1,2).setAttribute(WeightAttrId, 5);
    dg.getEdgeByVertices(2,1).setAttribute(WeightAttrId, -2);
    dg.getEdgeByVertices(0,3).setAttribute(WeightAttrId, 7);
    dg.getEdgeByVertices(1,3).setAttribute(WeightAttrId, 8);
    dg.getEdgeByVertices(1,4).setAttribute(WeightAttrId, -4);
    dg.getEdgeByVertices(3,2).setAttribute(WeightAttrId, -3);
    dg.getEdgeByVertices(3,4).setAttribute(WeightAttrId, 9);
    dg.getEdgeByVertices(4,2).setAttribute(WeightAttrId, 7);
    dg.getEdgeByVertices(4,0).setAttribute(WeightAttrId, 2);

    const Edge::AttributeId DistanceAttrId = dg.addEdgeAttribute("distance");
    ShortestPathSearcher sps1(&dg);
    sps1.bellmanFord(0);

    EXPECT_EQ(dg.getVertexByNumber(0).getAttribute(DistanceAttrId), 0);
    EXPECT_EQ(dg.getVertexByNumber(1).getAttribute(DistanceAttrId), 2);
    EXPECT_EQ(dg.getVertexByNumber(2).getAttribute(DistanceAttrId), 4);
    EXPECT_EQ(dg.getVertexByNumber(3).getAttribute(DistanceAttrId), 7);
    EXPECT_EQ(dg.getVertexByNumber(4).getAttribute(DistanceAttrId), -2);

    sps1.bellmanFord(4);

    // Dag(Directed Acyclic Graph - shortest paths)
    DirectedGraph dguc(6);
    dguc.addEdgeBetween(0, 1);
    dguc.addEdgeBetween(1, 2);
    dguc.addEdgeBetween(2, 3);
    dguc.addEdgeBetween(3, 4);
    dguc.addEdgeBetween(4, 5);
    dguc.addEdgeBetween(0, 2);
    dguc.addEdgeBetween(1, 3);
    dguc.addEdgeBetween(3, 5);
    dguc.addEdgeBetween(2, 4);
    dguc.addEdgeBetween(2, 5);

    const Edge::AttributeId WeightAttrId1 = dguc.addEdgeAttribute("weight");

    dguc.getEdgeByVertices(0, 1).setAttribute(WeightAttrId1, 5);
    dguc.getEdgeByVertices(1, 2).setAttribute(WeightAttrId1, 2);
    dguc.getEdgeByVertices(2, 3).setAttribute(WeightAttrId1, 7);
    dguc.getEdgeByVertices(3, 4).setAttribute(WeightAttrId1, -1);
    dguc.getEdgeByVertices(4, 5).setAttribute(WeightAttrId1, -2);
    dguc.getEdgeByVertices(0, 2).setAttribute(WeightAttrId1, 3);
    dguc.getEdgeByVertices(1, 3).setAttribute(WeightAttrId1, 6);
    dguc.getEdgeByVertices(3, 5).setAttribute(WeightAttrId1, 1);
    dguc.getEdgeByVertices(2, 4).setAttribute(WeightAttrId1, 4);
    dguc.getEdgeByVertices(2, 5).setAttribute(WeightAttrId1, 2);

    ShortestPathSearcher sps2(&dguc);
    sps2.dagShortestPaths(1);
    const Vertex::AttributeId DistanceAttrId1 = dguc.getVertexAttributeIdByName("distance");
    EXPECT_EQ(dguc.getVertexByNumber(0).getAttribute(DistanceAttrId1), Vertex::Limits::Inf);
    EXPECT_EQ(dguc.getVertexByNumber(1).getAttribute(DistanceAttrId1), 0);
    EXPECT_EQ(dguc.getVertexByNumber(2).getAttribute(DistanceAttrId1), 2);
    EXPECT_EQ(dguc.getVertexByNumber(3).getAttribute(DistanceAttrId1), 6);
    EXPECT_EQ(dguc.getVertexByNumber(4).getAttribute(DistanceAttrId1), 5);
    EXPECT_EQ(dguc.getVertexByNumber(5).getAttribute(DistanceAttrId1), 3);

    sps2.dagShortestPaths(0);
    const Vertex::AttributeId DistanceAttrId2 = dguc.getVertexAttributeIdByName("distance");
    EXPECT_EQ(dguc.getVertexByNumber(0).getAttribute(DistanceAttrId2), 0);
    EXPECT_EQ(dguc.getVertexByNumber(1).getAttribute(DistanceAttrId2), 5);
    EXPECT_EQ(dguc.getVertexByNumber(2).getAttribute(DistanceAttrId2), 3);
    EXPECT_EQ(dguc.getVertexByNumber(3).getAttribute(DistanceAttrId2), 10);
    EXPECT_EQ(dguc.getVertexByNumber(4).getAttribute(DistanceAttrId2), 7);
    EXPECT_EQ(dguc.getVertexByNumber(5).getAttribute(DistanceAttrId2), 5);

    // Dijkstra

    DirectedGraph dg2(5);
    dg2.addEdgeBetween(0, 1);
    dg2.addEdgeBetween(0, 3);
    dg2.addEdgeBetween(1, 2);
    dg2.addEdgeBetween(1, 3);
    dg2.addEdgeBetween(2, 4);
    dg2.addEdgeBetween(3, 1);
    dg2.addEdgeBetween(3, 2);
    dg2.addEdgeBetween(3, 4);
    dg2.addEdgeBetween(4, 0);
    dg2.addEdgeBetween(4, 2);

    const Edge::AttributeId WeightAttrId2 = dg2.addEdgeAttribute("weight");

    dg2.getEdgeByVertices(0,1).setAttribute(WeightAttrId2, 10);
    dg2.getEdgeByVertices(0,3).setAttribute(WeightAttrId2, 5);
    dg2.getEdgeByVertices(1,2).setAttribute(WeightAttrId2, 1);
    dg2.getEdgeByVertices(1,3).setAttribute(WeightAttrId2, 2);
    dg2.getEdgeByVertices(2,4).setAttribute(WeightAttrId2, 4);
    dg2.getEdgeByVertices(3,1).setAttribute(WeightAttrId2, 3);
    dg2.getEdgeByVertices(3,2).setAttribute(WeightAttrId2, 9);
    dg2.getEdgeByVertices(3,4).setAttribute(WeightAttrId2, 2);
    dg2.getEdgeByVertices(4,0).setAttribute(WeightAttrId2, 7);
    dg2.getEdgeByVertices(4,2).setAttribute(WeightAttrId2, 6);

    ShortestPathSearcher spsdij(&dg2);
    spsdij.dijkstra(0);
    const Vertex::AttributeId DistanceAttrId3 = dg2.getVertexAttributeIdByName("distance");

    EXPECT_EQ(dg2.getVertexByNumber(0).getAttribute(DistanceAttrId3), 0);
    EXPECT_EQ(dg2.getVertexByNumber(1).getAttribute(DistanceAttrId3), 8);
    EXPECT_EQ(dg2.getVertexByNumber(2).getAttribute(DistanceAttrId3), 9);
    EXPECT_EQ(dg2.getVertexByNumber(3).getAttribute(DistanceAttrId3), 5);
    EXPECT_EQ(dg2.getVertexByNumber(4).getAttribute(DistanceAttrId3), 7);

    DirectedGraph dg3{5};

    dg3.addEdgeBetween(0, 1);
    dg3.addEdgeBetween(0, 3);
    dg3.addEdgeBetween(1, 2);
    dg3.addEdgeBetween(1, 3);
    dg3.addEdgeBetween(2, 4);
    dg3.addEdgeBetween(3, 1);
    dg3.addEdgeBetween(3, 2);
    dg3.addEdgeBetween(3, 4);
    dg3.addEdgeBetween(4, 0);
    dg3.addEdgeBetween(4, 2);

    const Edge::AttributeId WeightAttrId4 = dg3.addEdgeAttribute("weight");

    dg3.getEdgeByVertices(0,1).setAttribute(WeightAttrId4, 3);
    dg3.getEdgeByVertices(0,3).setAttribute(WeightAttrId4, 5);
    dg3.getEdgeByVertices(1,2).setAttribute(WeightAttrId4, 6);
    dg3.getEdgeByVertices(1,3).setAttribute(WeightAttrId4, 2);
    dg3.getEdgeByVertices(2,4).setAttribute(WeightAttrId4, 2);
    dg3.getEdgeByVertices(3,1).setAttribute(WeightAttrId4, 1);
    dg3.getEdgeByVertices(3,2).setAttribute(WeightAttrId4, 4);
    dg3.getEdgeByVertices(3,4).setAttribute(WeightAttrId4, 6);
    dg3.getEdgeByVertices(4,0).setAttribute(WeightAttrId4, 3);
    dg3.getEdgeByVertices(4,2).setAttribute(WeightAttrId4, 7);

    ShortestPathSearcher spsdij2(&dg3);
    spsdij2.dijkstra(4);

    const Vertex::AttributeId DistanceAttrId4 = dg3.getVertexAttributeIdByName("distance");

    EXPECT_EQ(dg3.getVertexByNumber(0).getAttribute(DistanceAttrId4), 3);
    EXPECT_EQ(dg3.getVertexByNumber(1).getAttribute(DistanceAttrId4), 6);
    EXPECT_EQ(dg3.getVertexByNumber(2).getAttribute(DistanceAttrId4), 7);
    EXPECT_EQ(dg3.getVertexByNumber(3).getAttribute(DistanceAttrId4), 8);
    EXPECT_EQ(dg3.getVertexByNumber(4).getAttribute(DistanceAttrId4), 0);

    // Floyd-Warshall
    DirectedGraph dg4(5, DirectedGraph::Interfaces::AdjacencyMatrix);
    dg4.addEdgeBetween(0, 1);
    dg4.addEdgeBetween(0, 2);
    dg4.addEdgeBetween(0, 4);
    dg4.addEdgeBetween(1, 3);
    dg4.addEdgeBetween(1, 4);
    dg4.addEdgeBetween(2, 1);
    dg4.addEdgeBetween(3, 0);
    dg4.addEdgeBetween(3, 2);
    dg4.addEdgeBetween(4, 3);

    const Edge::AttributeId WeightAttrId5 = dg4.addEdgeAttribute("weight");

    dg4.getEdgeByVertices(0,1).setAttribute(WeightAttrId5, 3);
    dg4.getEdgeByVertices(0,2).setAttribute(WeightAttrId5, 8);
    dg4.getEdgeByVertices(0,4).setAttribute(WeightAttrId5, -4);
    dg4.getEdgeByVertices(1,3).setAttribute(WeightAttrId5, 1);
    dg4.getEdgeByVertices(1,4).setAttribute(WeightAttrId5, 7);
    dg4.getEdgeByVertices(2,1).setAttribute(WeightAttrId5, 4);
    dg4.getEdgeByVertices(3,0).setAttribute(WeightAttrId5, 2);
    dg4.getEdgeByVertices(3,2).setAttribute(WeightAttrId5, -5);
    dg4.getEdgeByVertices(4,3).setAttribute(WeightAttrId5, 6);

    ShortestPathSearcher::Matrix weights;
    ShortestPathSearcher::Matrix parents;
    ShortestPathSearcher spsfw1(&dg4);
    spsfw1.floydWarshall(weights, parents);

    ShortestPathSearcher::Matrix expectedRes;
    ShortestPathSearcher::Matrix expectedParents;
    expectedRes.push_back({0,1,-3,2,-4});
    expectedRes.push_back({3,0,-4,1,-1});
    expectedRes.push_back({7,4,0,5,3});
    expectedRes.push_back({2,-1,-5,0,-2});
    expectedRes.push_back({8,5,1,6,0});

    Vertex::AttributeType np = ShortestPathSearcher::NoParent;
    Vertex::AttributeType inf = ShortestPathSearcher::WeightInf;
    expectedParents.push_back({np,2,3,4,0});
    expectedParents.push_back({3,np,3,1,0});
    expectedParents.push_back({3,2,np,1,0});
    expectedParents.push_back({3,2,3,np,0});
    expectedParents.push_back({3,2,3,4,np});

    EXPECT_EQ(weights, expectedRes);
    EXPECT_EQ(parents, expectedParents);

    DirectedGraph dg5(6, DirectedGraph::Interfaces::AdjacencyMatrix);
    dg5.addEdgeBetween(0,4);
    dg5.addEdgeBetween(1,3);
    dg5.addEdgeBetween(2,1);
    dg5.addEdgeBetween(2,5);
    dg5.addEdgeBetween(3,0);
    dg5.addEdgeBetween(3,4);
    dg5.addEdgeBetween(4,1);
    dg5.addEdgeBetween(5,1);
    dg5.addEdgeBetween(5,2);

    const Edge::AttributeId WeightAttrId6 = dg5.addEdgeAttribute("weight");

    dg5.getEdgeByVertices(0,4).setAttribute(WeightAttrId6, -1);
    dg5.getEdgeByVertices(1,3).setAttribute(WeightAttrId6, 2);
    dg5.getEdgeByVertices(2,1).setAttribute(WeightAttrId6, 2);
    dg5.getEdgeByVertices(2,5).setAttribute(WeightAttrId6, -8);
    dg5.getEdgeByVertices(3,0).setAttribute(WeightAttrId6, -4);
    dg5.getEdgeByVertices(3,4).setAttribute(WeightAttrId6, 3);
    dg5.getEdgeByVertices(4,1).setAttribute(WeightAttrId6, 7);
    dg5.getEdgeByVertices(5,1).setAttribute(WeightAttrId6, 5);
    dg5.getEdgeByVertices(5,2).setAttribute(WeightAttrId6, 10);

    ShortestPathSearcher::Matrix w1;
    ShortestPathSearcher::Matrix p1;
    ShortestPathSearcher spsfw2(&dg5);
    spsfw2.floydWarshall(w1, p1);

    ShortestPathSearcher::Matrix expectedRes1;
    expectedRes1.push_back({0,6,inf,8,-1,inf});
    expectedRes1.push_back({-2,0,inf,2,-3,inf});
    expectedRes1.push_back({-5,-3,0,-1,-6,-8});
    expectedRes1.push_back({-4,2,inf,0,-5,inf});
    expectedRes1.push_back({5,7,inf,9,0,inf});
    expectedRes1.push_back({3,5,10,7,2,0});
    EXPECT_EQ(w1, expectedRes1);
}

TEST(ArticulationPointsTest, GraphAlgorithmsTest)
{
    UndirectedGraph ugsm(6);
    ugsm.addEdgeBetween(0,1);
    ugsm.addEdgeBetween(0,2);
    ugsm.addEdgeBetween(1,4);
    ugsm.addEdgeBetween(4,2);
    ugsm.addEdgeBetween(2,3);
    ugsm.addEdgeBetween(2,5);
    ugsm.addEdgeBetween(3,5);
    Graph::VertexNumbers vnsm;
    std::vector<Edge> bridges;
    ArticulationPointsSearcher<Graph::VertexNumbers, std::vector<Edge>>(ugsm, vnsm, bridges);

    Graph::VertexNumbers expectedRes{2};
    EXPECT_EQ(vnsm, expectedRes);

    UndirectedGraph ug(23);
    ug.addEdgeBetween(0,1);
    ug.addEdgeBetween(0,3);
    ug.addEdgeBetween(0,2);
    ug.addEdgeBetween(1,2);
    ug.addEdgeBetween(1,3);
    ug.addEdgeBetween(3,2);
    ug.addEdgeBetween(1,4);
    ug.addEdgeBetween(4,5);
    ug.addEdgeBetween(4,6);
    ug.addEdgeBetween(5,6);
    ug.addEdgeBetween(4,7);
    ug.addEdgeBetween(4,8);
    ug.addEdgeBetween(8,7);
    ug.addEdgeBetween(8,9);
    ug.addEdgeBetween(4,10);
    ug.addEdgeBetween(10,11);
    ug.addEdgeBetween(10,13);
    ug.addEdgeBetween(11,12);
    ug.addEdgeBetween(12,13);
    ug.addEdgeBetween(12,14);
    ug.addEdgeBetween(14,17);
    ug.addEdgeBetween(14,16);
    ug.addEdgeBetween(15,16);
    ug.addEdgeBetween(15,17);
    ug.addEdgeBetween(16,17);
    ug.addEdgeBetween(15,18);
    ug.addEdgeBetween(16,18);
    ug.addEdgeBetween(17,19);
    ug.addEdgeBetween(17,20);
    ug.addEdgeBetween(19,21);
    ug.addEdgeBetween(19,22);
    ug.addEdgeBetween(21,22);

    Graph::VertexNumbers vn;
    std::vector<Edge> bridges2;
    ArticulationPointsSearcher<Graph::VertexNumbers, std::vector<Edge>>{ug, vn, bridges2};

    Graph::VertexNumbers expectedRes2{1,4,8,10,12,14,17,19};
    EXPECT_EQ(vn, expectedRes2);
    EXPECT_EQ(bridges2.size(), 6);

    //start point case
    UndirectedGraph ugsm1(6);
    ugsm1.addEdgeBetween(0,2);
    ugsm1.addEdgeBetween(0,4);
    ugsm1.addEdgeBetween(0,3);
    ugsm1.addEdgeBetween(0,5);
    ugsm1.addEdgeBetween(1,2);
    ugsm1.addEdgeBetween(1,4);
    ugsm1.addEdgeBetween(3,5);
    Graph::VertexNumbers vnsm1;
    std::vector<Edge> bridges3;
    ArticulationPointsSearcher<Graph::VertexNumbers, std::vector<Edge>>(ugsm1, vnsm1, bridges3);

    Graph::VertexNumbers expectedRes3{0};
    EXPECT_EQ(vnsm1, expectedRes3);

}

TEST(FordFulkersonTest, GraphAlgorithmsTest)
{
    DirectedGraph dg(6);
    dg.addEdgeBetween(0,1);
    dg.addEdgeBetween(0,2);
    dg.addEdgeBetween(1,3);
    dg.addEdgeBetween(2,1);
    dg.addEdgeBetween(2,4);
    dg.addEdgeBetween(3,2);
    dg.addEdgeBetween(3,5);
    dg.addEdgeBetween(4,3);
    dg.addEdgeBetween(4,5);

    const Edge::AttributeId CurrentAttrId = dg.registerEdgeAttributeIfNotRegistered("current");

    dg.getEdgeByVertices(0,1).setAttribute(CurrentAttrId, 16);
    dg.getEdgeByVertices(0,2).setAttribute(CurrentAttrId, 13);
    dg.getEdgeByVertices(1,3).setAttribute(CurrentAttrId, 12);
    dg.getEdgeByVertices(2,1).setAttribute(CurrentAttrId, 4);
    dg.getEdgeByVertices(2,4).setAttribute(CurrentAttrId, 14);
    dg.getEdgeByVertices(3,2).setAttribute(CurrentAttrId, 9);
    dg.getEdgeByVertices(3,5).setAttribute(CurrentAttrId, 20);
    dg.getEdgeByVertices(4,3).setAttribute(CurrentAttrId, 7);
    dg.getEdgeByVertices(4,5).setAttribute(CurrentAttrId, 4);

    FordFulkerson(dg, 0, 5);

    const Edge::AttributeId FlowAttrId = dg.getEdgeAttributeIdByName("flow");

    EXPECT_EQ(dg.getEdgeByVertices(0,1).getAttribute(FlowAttrId), 12);
    EXPECT_EQ(dg.getEdgeByVertices(0,2).getAttribute(FlowAttrId), 11);
    EXPECT_EQ(dg.getEdgeByVertices(1,3).getAttribute(FlowAttrId), 12);
    EXPECT_EQ(dg.getEdgeByVertices(2,1).getAttribute(FlowAttrId), 0);
    EXPECT_EQ(dg.getEdgeByVertices(2,4).getAttribute(FlowAttrId), 11);
    EXPECT_EQ(dg.getEdgeByVertices(3,2).getAttribute(FlowAttrId), 0);
    EXPECT_EQ(dg.getEdgeByVertices(3,5).getAttribute(FlowAttrId), 19);
    EXPECT_EQ(dg.getEdgeByVertices(4,3).getAttribute(FlowAttrId), 7);
    EXPECT_EQ(dg.getEdgeByVertices(4,5).getAttribute(FlowAttrId), 4);

    // exception - incorrect network cases

    DirectedGraph dg2(6);
    dg2.addEdgeBetween(0,1);
    dg2.addEdgeBetween(1,0);
    dg2.addEdgeBetween(0,2);
    dg2.addEdgeBetween(1,3);
    dg2.addEdgeBetween(2,1);
    dg2.addEdgeBetween(2,4);
    dg2.addEdgeBetween(3,2);
    dg2.addEdgeBetween(3,5);
    dg2.addEdgeBetween(4,3);
    dg2.addEdgeBetween(4,5);

    EXPECT_THROW(FordFulkerson(dg2, 0, 5), GraphException);
    EXPECT_THROW(FordFulkerson(dg, 1, 5), GraphException);
    EXPECT_THROW(FordFulkerson(dg, 0, 2), GraphException);
}
