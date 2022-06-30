//
// Created by licm on 28/10/2021.
//

#include "BiAITstar/Edge.h"

//#include <utility>

namespace ompl::geometric::biait {

Edge::Edge()
    : parent_(),
      child_(),
      category_(0b00000),
      edgeKey_({base::Cost(std::numeric_limits<double>::signaling_NaN()),
                base::Cost(std::numeric_limits<double>::signaling_NaN()),
                base::Cost(std::numeric_limits<double>::signaling_NaN())}) {}

Edge::Edge(std::shared_ptr<Vertex> parent, std::shared_ptr<Vertex> child,
           std::bitset<5> category, const std::array<base::Cost, 3u>& edgeKey)
    : parent_(std::move(parent)),
      child_(std::move(child)),
      category_(category),
      edgeKey_(edgeKey) {}

}  // namespace ompl::geometric::biait
