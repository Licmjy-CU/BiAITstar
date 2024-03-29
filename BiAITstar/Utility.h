//
// Created by Chenming LI on 11/1/2022.
//

#ifndef BIAIT_DEV_UTILITY_H
#define BIAIT_DEV_UTILITY_H

#include "BiAITstar/Edge.h"
#include "BiAITstar/ImplicitGraph.h"
#include "BiAITstar/Vertex.h"
#include "ompl/base/OptimizationObjective.h"

namespace ompl::geometric::biait {

class Utility {
 public:
  // clang-format off
  /**
   *
   * @tparam elemType
   * @param container
   * @param value
   * @brief a wrapper for std::vector<>.erase;
   */
  // clang-format on
  template <class elemType>
  static inline void removeFromVectorByValue(std::vector<elemType>& container,
                                             const elemType& value);

  // clang-format off
  /**
   *
   * @tparam elemType
   * @param container
   * @param value
   * @bried if container contains value, return true, else false;
   */
  // clang-format on
  template <class elemType>
  static inline bool containsByValue(const std::vector<elemType>& container,
                                     const elemType& value);

  static base::Cost worseCost(const base::OptimizationObjectivePtr& optObjPtr,
                              const base::Cost& lhs, const base::Cost& rhs);
};

template <class elemType>
void Utility::removeFromVectorByValue(std::vector<elemType>& container,
                                      const elemType& value) {
  container.erase(std::remove(container.begin(), container.end(), value),
                  container.end());
}

template <class elemType>
bool Utility::containsByValue(const std::vector<elemType>& container,
                              const elemType& value) {
  if (std::find(std::begin(container), std::end(container), value) !=
      std::end(container)) {
    return true;
  } else {
    return false;
  }
}

}  // namespace ompl::geometric::biait

#endif  // BIAIT_DEV_UTILITY_H
