//
// Created by Chenming LI on 11/1/2022.
//

#ifndef BIAIT_DEV_GENERATEID_H
#define BIAIT_DEV_GENERATEID_H

#include <iostream>

namespace ompl::geometric::biait {
static std::size_t generateId() {
  static std::size_t id{0u};
  return id++;
}
}  // namespace ompl::geometric::biait

#endif  // BIAIT_DEV_GENERATEID_H
