//
// Created by Chenming LI on 17/1/2022.
//

#include "BiAITstar/Utility.h"

namespace ompl::geometric::biait {

base::Cost Utility::worseCost(const base::OptimizationObjectivePtr &optObjPtr,
                              const base::Cost &lhs, const base::Cost &rhs) {
  if (optObjPtr->isCostBetterThan(lhs, rhs)) {
    return rhs;
  } else {
    return lhs;
  }
}

}  // namespace ompl::geometric::biait
