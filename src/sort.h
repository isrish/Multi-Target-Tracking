/**
* Software License Agreement (BSD License)
*
*           Copyright (C) 2015 by Israel D. Gebru,
*           Perception Team, INRIA-Grenoble, France
*                   All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the FreeBSD Project.
*/

#ifndef SORT_H
#define SORT_H
#include <vector>
#include <algorithm>

namespace MYSORT {

    // Act like matlab's [Y,I] = SORT(X)
    // Input:
    //   unsorted  unsorted vector
    // Output:
    //   sorted     sorted vector, allowed to be same as unsorted
    //   index_map  an index map such that sorted[i] = unsorted[index_map[i]]
    template <class T>
    void sort(
            std::vector<T> &unsorted,
            std::vector<T> &sorted,
            std::vector<size_t> &index_map);

    // Act like matlab's Y = X[I]
    // where I contains a vector of indices so that after,
    // Y[j] = X[I[j]] for index j
    // this implies that Y.size() == I.size()
    // X and Y are allowed to be the same reference
    template< class T >
    void reorder(
            std::vector<T> & unordered,
            std::vector<size_t> const & index_map,
            std::vector<T> & ordered);

    ////////////////////////////////////////////////////////////////////////////////
    // Implementation
    ////////////////////////////////////////////////////////////////////////////////


    template<class T> struct index_cmp
    {
        index_cmp(const T arr) : arr(arr) {}
        bool operator()(const size_t a, const size_t b) const
        {
            return arr[a] < arr[b];
        }
        const T arr;
    };

    template <class T>
    void sort(
            std::vector<T> & unsorted,
            std::vector<T> & sorted,
            std::vector<size_t> & index_map)
    {
        // Original unsorted index map
        index_map.resize(unsorted.size());
        for(size_t i=0;i<unsorted.size();i++)
            {
                index_map[i] = i;
            }
        // Sort the index map, using unsorted for comparison
        sort(
                    index_map.begin(),
                    index_map.end(),
                    index_cmp<std::vector<T>& >(unsorted));

        sorted.resize(unsorted.size());
        reorder(unsorted,index_map,sorted);
    }

    // This implementation is O(n), but also uses O(n) extra memory
    template< class T >
    void reorder(
            std::vector<T> & unordered,
            std::vector<size_t> const & index_map,
            std::vector<T> & ordered)
    {
        // copy for the reorder according to index_map, because unsorted may also be
        // sorted
        std::vector<T> copy = unordered;
        ordered.resize(index_map.size());
        for(unsigned int i = 0; i<index_map.size();i++)
            {
                ordered[i] = copy[index_map[i]];
            }
    }
}

#endif // SORT_H

