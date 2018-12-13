#pragma once
#include <memory>

template<typename T>
struct DisjointSet;

template<typename T>
using PDisjointSet = std::shared_ptr<DisjointSet<T>>;

/*
    Disjoint set node
*/
template<typename T>
struct DisjointSet
{
    PDisjointSet<T> pparent;
    T value;
    int rank;
};

template<typename T>
PDisjointSet<T> makeSet(const T& val)
{
    PDisjointSet<T> newSet = std::make_shared<DisjointSet<T>>(DisjointSet<T>{});
    newSet->pparent = newSet;
    newSet->value = val;
    newSet->rank = 0;
    return newSet;
}

template<typename T>
PDisjointSet<T> findSet(PDisjointSet<T> dsnode)
{
    if(dsnode->pparent != dsnode)
    {
        dsnode->pparent = findSet(dsnode->pparent);
    }
    return dsnode->pparent;
}

template<typename T>
void link(PDisjointSet<T> s1, PDisjointSet<T> s2)
{
    if(s1->rank > s2->rank)
    {
        s2->pparent = s1;
    }
    else
    {
        s1->pparent = s2;
        if(s1->rank == s2->rank)
        {
            ++s2->rank;
        }
    }
}

template<typename T>
void Union(PDisjointSet<T> s1, PDisjointSet<T> s2)
{
    link(findSet(s1), findSet(s2));
}
