#pragma once
#include <vector>
#include <algorithm>

/*
    Some useful functions
*/

template<typename InputIterator>
void countingSort(InputIterator first, InputIterator last, typename std::iterator_traits<InputIterator>::value_type max)
{
    if(first == last)
    {
        return;
    }
    std::vector<size_t> counts;
    counts.resize(max + 1, 0);

    InputIterator start = first;
    while(start != last)
    {
        ++counts[*start];
        ++start;
    }

    for(size_t i = 0; i < counts.size(); ++i)
    {
        std::fill_n(first, counts[i], i);
        std::advance(first, counts[i]);
    }
}

// without provided max
template<typename InputIterator>
void countingSort(InputIterator first, InputIterator last)
{
    if(first == last)
    {
        return;
    }
    typename std::iterator_traits<InputIterator>::value_type max = *std::max_element(first, last);
    countingSort(first, last, max);
}

/*
    Before counting each element is transformed -
        it can be used, for example, for extracting attributes of classes and more and more...
*/
template<typename InputIterator, typename Transformer>
void countingSortTransformed(InputIterator first, InputIterator last, Transformer transformer, size_t max)
{
    if(first == last)
    {
        return;
    }
    std::vector<size_t> counts;
    counts.resize(max + 1, 0);

    InputIterator start = first;
    while(start != last)
    {
        ++counts[transformer(*start)];
        ++start;
    }

    for(size_t i = 1; i < counts.size(); ++i)
    {
        counts[i] += counts[i-1];
    }

    typedef typename std::iterator_traits<InputIterator>::value_type Value;
    size_t n = std::distance(first, last);
    std::vector<Value> resVector;
    resVector.resize(n);
    start = first;
    while(start != last)
    {
        size_t val = transformer(*start);
        resVector[counts[val] - 1] = *start;
        --counts[val];;
        ++start;
    }
    std::move(resVector.begin(), resVector.end(), first);
}

template<typename InputIterator, typename Transformer>
void countingSortTransformed(InputIterator first, InputIterator last, Transformer transformer)
{
    if(first == last)
    {
        return;
    }
    typedef typename std::iterator_traits<InputIterator>::value_type Value;
    size_t max = transformer(*std::max_element(first, last,
                                  [transformer](const Value& v1, const Value& v2){return transformer(v1) < transformer(v2);}));
    countingSortTransformed(first, last, transformer, max);
}

