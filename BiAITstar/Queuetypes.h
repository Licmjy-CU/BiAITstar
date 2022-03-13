//
// Created by Chenming LI on 29/10/2021.
//

#ifndef BIAIT_DEV_QUEUETYPES_H
#define BIAIT_DEV_QUEUETYPES_H

#include <memory>

#include "ompl/base/Cost.h"
#include "ompl/datastructures/BinaryHeap.h"


namespace ompl::geometric::biait {

    class Edge;
    class Vertex;
    class WeakEdge;

    class Queuetypes {
    public:

        using EdgeQueue = ompl::BinaryHeap<Edge, std::function<bool(const Edge &, const Edge &)> >;

        using MeetLazyEdgeQueue = ompl::BinaryHeap<WeakEdge, std::function<bool(const WeakEdge &, const WeakEdge &)> >;

        // The cost is the meetEdgeCost, didn't use for sorting;
        using CostEdgePair = std::pair<base::Cost, Edge>;

        using MeetValidEdgeQueue = ompl::BinaryHeap<CostEdgePair, std::function<bool(const CostEdgePair &, const CostEdgePair &)> >;

        // A type for elements in the vertex queue;
        using KeyVertexPair = std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex> >;

        using VertexQueue =
            ompl::BinaryHeap<KeyVertexPair, std::function<bool(const KeyVertexPair &, const KeyVertexPair &)> >;

//        template<class elemType>
//        std::function<bool(const elemType &lhs, const elemType &rhs)> lambdaEdgeQueue{[this](const elemType &lhs, const elemType &rhs) { return isEdgeBetter(lhs, rhs);};

    };

}




#endif //BIAIT_DEV_QUEUETYPES_H
