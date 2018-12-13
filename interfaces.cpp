#include "interfaces.hpp"

//----------------Adjacency lists interface

AdjacencyListsInterface::AdjacencyListsInterface(const AdjacencyListsInterface& other)
{
    adjLists = other.adjLists;
}

AdjacencyListsInterface& AdjacencyListsInterface::operator=(const AdjacencyListsInterface& other)
{
    adjLists = other.adjLists;
    return *this;
}

// O(1) (average)
bool AdjacencyListsInterface::hasEdge(Vertex::Number v1, Vertex::Number v2) const
{
    auto fromVertexList = adjLists.at(v1);
    bool contains = std::find(fromVertexList.begin(), fromVertexList.end(), v2) != fromVertexList.end();
    return contains;
}

// adding 'count' vertices to interface
// O(1)
void AdjacencyListsInterface::addVertex(Vertex::Number count)
{
    // adding new vertex into adjacency lists vector
    adjLists.resize(adjLists.size() + count);
}

// O(1)
void AdjacencyListsInterface::addEdge(Vertex::Number v1, Vertex::Number v2, bool directed)
{;
    adjLists[v1].insert(v2);
    // for not directed edges adding second direction
    if(!directed)
    {
        adjLists[v2].insert(v1);
    }
}

// O(1)
void AdjacencyListsInterface::deleteEdge(Vertex::Number v1, Vertex::Number v2, bool directed)
{
    adjLists[v1].erase(v2);
    // for not directed edges erasing second direction
    if(!directed)
    {
        adjLists[v2].erase(v1);
    }
}

//O(|V| + |E|) - because we have no more iterations, that we have edges(but some vertices can not have neighboors).
void AdjacencyListsInterface::transpose()
{
    // creating new adjacency lists
    AdjacencyLists newAdjLists;
    newAdjLists.resize(adjLists.size());
    // newAdjLists[i][j] = adjLists[j][i]
    for(size_t list = 0; list < adjLists.size(); ++list)
    {
        for(auto item: adjLists[list])
        {
            newAdjLists[item].insert(list);
        }
    }
    adjLists = newAdjLists;
}

// O(|E|)
Vertex::Number AdjacencyListsInterface::inDegree(Vertex::Number v) const
{
    Vertex::Number indeg = 0;
    for(auto list: adjLists)
    {
        indeg += list.count(v);
    }
    return indeg;
}

// O(1)
Vertex::Number AdjacencyListsInterface::outDegree(Vertex::Number v) const
{
    return adjLists[v].size();
}

// O(|E|)
Vertex::Number AdjacencyListsInterface::degree(Vertex::Number v) const
{
    return inDegree(v) + outDegree(v);
}

// grabs degrees of ALL vertices
// Complexity: O(|E|).
void AdjacencyListsInterface::getDegreesList(VertexNumbers& inDegContainer, VertexNumbers& outDegContainer, VertexNumbers& degContainer) const
{
    inDegContainer.resize(adjLists.size(), 0);
    outDegContainer.resize(adjLists.size(), 0);
    degContainer.resize(adjLists.size(), 0);
    for(size_t adjList = 0; adjList < adjLists.size(); ++adjList)
    {
        for(auto vNum: adjLists[adjList])
        {
            ++inDegContainer[vNum];
            ++outDegContainer[adjList];
            ++degContainer[vNum];
            ++degContainer[adjList];
        }
    }
}

// O(1) - adjacent vertex order is NOT DETERMINED
typename AdjacencyListsInterface::VertexNumbers AdjacencyListsInterface::getAdjacentVerticesFor(Vertex::Number v) const
{
    std::vector<Vertex::Number> adjVector;
    adjVector.reserve(adjLists[v].size());
    adjVector.insert(adjVector.begin(), adjLists[v].begin(), adjLists[v].end());
    return adjVector;
}

//----------------/Adjacency lists interface

//----------------Adjacency matrix interface

AdjacencyMatrixInterface::AdjacencyMatrixInterface(const AdjacencyMatrixInterface& other)
{
    adjMatrix = other.adjMatrix;
}

AdjacencyMatrixInterface& AdjacencyMatrixInterface::operator=(const AdjacencyMatrixInterface& other)
{
    adjMatrix = other.adjMatrix;
    return *this;
}

// O(1)
bool AdjacencyMatrixInterface::hasEdge(Vertex::Number v1, Vertex::Number v2) const
{
    return adjMatrix[v1][v2] != NotPresented;
}

// adding 'count' vertices to interface
// O(|V|)
void AdjacencyMatrixInterface::addVertex(Vertex::Number count)
{
    // adding new vertex as new row in the matrix and as new column in each row of the matrix
    adjMatrix.resize(adjMatrix.size() + count);
    for(size_t i = 0; i < adjMatrix.size(); ++i)
    {
        adjMatrix[i].resize(adjMatrix.size(), NotPresented);
    }
}

// O(1)
void AdjacencyMatrixInterface::addEdge(Vertex::Number v1, Vertex::Number v2, bool directed)
{
    adjMatrix[v1][v2] = Presented;
    if(!directed)
    {
        adjMatrix[v2][v1] = Presented;
    }
}

// O(1)
void AdjacencyMatrixInterface::deleteEdge(Vertex::Number v1, Vertex::Number v2, bool directed)
{
    adjMatrix[v1][v2] = NotPresented;
    if(!directed)
    {
        adjMatrix[v2][v1] = NotPresented;
    }
}

// O(|V|^2)
void AdjacencyMatrixInterface::transpose()
{
    // transposing by diagonal
    for(size_t i = 0; i < adjMatrix.size(); ++i)
    {
        for(size_t j = i; j < adjMatrix[i].size(); ++j)
        {
            std::swap(adjMatrix[i][j], adjMatrix[j][i]);
        }
    }
}

// O(|V|)
Vertex::Number AdjacencyMatrixInterface::inDegree(Vertex::Number v) const
{
    Vertex::Number indeg = 0;
    for(size_t row = 0; row < adjMatrix.size(); ++row)
    {
        if(adjMatrix[row][v] == Presented)
        {
            ++indeg;
        }
    }
    return indeg;
}

// O(|V|)
Vertex::Number AdjacencyMatrixInterface::outDegree(Vertex::Number v) const
{
    Vertex::Number outdeg = 0;
    for(size_t col = 0; col < adjMatrix[v].size(); ++col)
    {
        if(adjMatrix[v][col] == Presented)
        {
            ++outdeg;
        }
    }
    return outdeg;
}

// O(|V|)
Vertex::Number AdjacencyMatrixInterface::degree(Vertex::Number v) const
{
    return inDegree(v) + outDegree(v);
}

// grabs degrees of ALL vertices
// Complexity: O(|V|^2).
// WARNING! Only has sense for adjacency lists, but as we need to provide unified functionality, also done it here.
void AdjacencyMatrixInterface::getDegreesList(VertexNumbers& inDegContainer, VertexNumbers& outDegContainer, VertexNumbers& degContainer) const
{
    inDegContainer.resize(adjMatrix.size(), 0);
    outDegContainer.resize(adjMatrix.size(), 0);
    degContainer.resize(adjMatrix.size(), 0);
    for(size_t i = 0; i < adjMatrix.size(); ++i)
    {
        for(size_t j = 0; j < adjMatrix.size(); ++j)
        {
            if(adjMatrix[i][j] == Presented)
            {
                ++inDegContainer[j];
                ++outDegContainer[i];
                ++degContainer[i];
                ++degContainer[j];
            }
        }
    }

}

// O(|V|)
// resulting vector is already sorted
typename AdjacencyMatrixInterface::VertexNumbers AdjacencyMatrixInterface::getAdjacentVerticesFor(Vertex::Number v) const
{
    std::vector<Vertex::Number> adjVector;
    adjVector.reserve(adjMatrix[v].size());
    for(size_t j = 0; j < adjMatrix[v].size(); ++j)
    {
        if(adjMatrix[v][j] == Presented)
        {
            adjVector.push_back(j);
        }
    }
    adjVector.shrink_to_fit();
    return adjVector;
}

//----------------/Adjacency matrix interface
