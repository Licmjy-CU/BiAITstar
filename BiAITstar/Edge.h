//
// Created by licm on 28/10/2021.
//

#ifndef BIAIT_DEV_EDGE_H
#define BIAIT_DEV_EDGE_H

#include <ompl/base/Cost.h>
#include <ompl/datastructures/BinaryHeap.h>

#include <array>
#include <bitset>
#include <limits>
#include <memory>

namespace ompl::geometric::biait {

class Vertex;
class Edge;
using EdgeQueue =
    ompl::BinaryHeap<Edge, std::function<bool(const Edge &, const Edge &)> >;

// The cost is the meetEdgeCost, didn't use for sorting;
using CostEdgePair = std::pair<base::Cost, Edge>;

using MeetValidEdgeQueue =
    ompl::BinaryHeap<CostEdgePair, std::function<bool(const CostEdgePair &,
                                                      const CostEdgePair &)> >;

class Edge {
 public:
  //  Needs a default constructor for the binary heap;
  Edge();

  // Constructor for regular edges;
  Edge(std::shared_ptr<Vertex> parent, std::shared_ptr<Vertex> child,
       std::bitset<5> category, const std::array<base::Cost, 3u> &edgeKey);

  ~Edge() = default;

  /* ##########  ##########  ##########  ##########  ########## */
  /*                     Setter and Getter                      */
  /* ##########  ##########  ##########  ##########  ########## */

  std::shared_ptr<Vertex> getParent() const { return parent_; }

  std::shared_ptr<Vertex> getChild() const { return child_; }

  std::bitset<5> getCategory() const { return category_; }

  const std::array<base::Cost, 3u> &getEdgeKey() const { return edgeKey_; }

  void setEdgeKey(const std::array<base::Cost, 3u> &edgeKey) {
    edgeKey_ = edgeKey;
  }  // maybe the r-value reference works better;

  const base::Cost &getMeetValidEdgeKey() const { return meetValidEdgeKey_; }

  void setMeetValidEdgeKey(const base::Cost meetValidEdgeKey) {
    meetValidEdgeKey_ = meetValidEdgeKey;
  }

  void setCategory(std::size_t position, bool flag) {
    category_[position] = flag;
  }

 private:
  /* ##########  ##########  ##########  ##########  ########## */
  /*                          Private                           */
  /* ##########  ##########  ##########  ##########  ########## */

  std::shared_ptr<Vertex> parent_{nullptr};

  std::shared_ptr<Vertex> child_{nullptr};

  // five bytes represent forward valid, forward lazy, meet, reverse valid, and
  // reverse lazy, respectively. assert(category_.count() == 1 ||
  // (category_.count() == 0 && statePtr_ == nullptr))
  std::bitset<5> category_{0b00000};

  std::array<base::Cost, 3u> edgeKey_;

  // Valid edge is only for the situation where both parent and child are valid
  // vertex; And parent is close to the start, child close to the goal(s);
  base::Cost meetValidEdgeKey_;
};

}  // namespace ompl::geometric::biait

#endif  // BIAIT_DEV_EDGE_H
