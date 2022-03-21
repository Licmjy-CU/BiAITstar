//
// Created by Chenming LI on 11/1/2022.
//

#ifndef BIAIT_DEV_UTILITY_H
#define BIAIT_DEV_UTILITY_H


#include "BiAITstar/ImplicitGraph.h"
#include "BiAITstar/Vertex.h"
#include "BiAITstar/Edge.h"

#include "ompl/base/OptimizationObjective.h"

namespace ompl::geometric::biait{

    class Utility {
    public:

        template<class elemType>
        static inline void removeFromVectorByValue(std::vector<elemType>& container, const elemType& value);

        template<class elemType>
        static inline bool containsByValue(const std::vector<elemType>&  container, const elemType& value);

        static base::Cost worseCost(const base::OptimizationObjectivePtr & optObjPtr, const base::Cost & lhs, const base::Cost & rhs);

    };


    template<class elemType>
    void Utility::removeFromVectorByValue(std::vector<elemType>& container, const elemType& value) {
        container.erase(std::remove(container.begin(), container.end(), value), container.end());
    }

    template<class elemType>
    bool Utility::containsByValue(const std::vector<elemType> &container, const elemType &value) {
        if(std::find(std::begin(container), std::end(container), value) != std::end(container)){
            return true;
        } else {
             return false;
        }
    }

}


#endif //BIAIT_DEV_UTILITY_H
