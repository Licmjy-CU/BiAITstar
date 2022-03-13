//
// Created by Chenming LI on 18/1/2022.
//

#include "BiAITstar/WeakEdge.h"

ompl::geometric::biait::WeakEdge::WeakEdge(std::weak_ptr<Vertex> parent, std::weak_ptr<Vertex> child,
                                           const ompl::base::Cost &meetLazyEdgeKey)
                                           : parent_(parent)
                                           , child_(child)
                                           , meetLazyEdgeKey_(meetLazyEdgeKey) {
}

ompl::geometric::biait::WeakEdge::WeakEdge(const ompl::geometric::biait::Edge &edge)
    : parent_(edge.getParent())
    , child_(edge.getChild())
    , meetLazyEdgeKey_(base::Cost(std::numeric_limits<double>::signaling_NaN())) {
}

ompl::geometric::biait::WeakEdge::WeakEdge()
    : parent_()
    , child_()
    , meetLazyEdgeKey_(base::Cost(std::numeric_limits<double>::signaling_NaN())){

}
