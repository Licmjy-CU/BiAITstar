//
// Created by licm on 28/10/2021.
//

#ifndef BIAIT_DEV_EDGE_H
#define BIAIT_DEV_EDGE_H

#include <array>
#include <limits>
#include <memory>
#include <bitset>

#include <ompl/base/Cost.h>

#include "BiAITstar/include/Queuetypes.h"


namespace ompl::geometric::biait {

    class Vertex;

    class Edge {
    public:
        //  Needs a default constructor for the binary heap;
        Edge();

        // Constructor for regular edges;
        Edge(std::shared_ptr<Vertex> parent,
             std::shared_ptr<Vertex> child,
             std::bitset<5> category,
             const std::array<base::Cost, 3u> &edgeKey);

        ~Edge() = default;

        /* ##########  ##########  ##########  ##########  ########## */
        /*                     Setter and Getter                      */
        /* ##########  ##########  ##########  ##########  ########## */

        std::shared_ptr<Vertex> getParent() const { return parent_; }

        std::shared_ptr<Vertex> getChild() const { return child_; }

        std::bitset<5> getCategory() const { return category_; }

        const std::array<base::Cost, 3u> & getEdgeKey() const { return edgeKey_; }

        void setEdgeKey(const std::array<base::Cost, 3u> & edgeKey) { edgeKey_ = edgeKey; } // maybe the r-value reference works better;

        // Move to the class Vertex;
        // mutable Queuetypes::EdgeQueue::Element * meetLazyEdgeQueuePointer_{nullptr};

        const base::Cost & getMeetValidEdgeKey() const { return meetValidEdgeKey_; }

        void setMeetValidEdgeKey(const base::Cost meetValidEdgeKey) { meetValidEdgeKey_ = meetValidEdgeKey; }

        void setCategory(std::size_t position, bool flag) { category_[position] = flag; }


    private:

        /* ##########  ##########  ##########  ##########  ########## */
        /*                          Private                           */
        /* ##########  ##########  ##########  ##########  ########## */

        std::shared_ptr<Vertex> parent_{nullptr};

        std::shared_ptr<Vertex> child_{nullptr};

        // five bytes represent forward valid, forward lazy, meet, reverse valid, and reverse lazy, respectively.
        // assert(category_.count() == 1 || (category_.count() == 0 && statePtr_ == nullptr))
        std::bitset<5> category_{0b00000};

        std::array<base::Cost, 3u> edgeKey_;

        // Valid edge is only for the situation where both parent and child are valid vertex;
        // And parent is close to the start, child close to the goal(s);
        base::Cost meetValidEdgeKey_;

    };

}


#endif //BIAIT_DEV_EDGE_H
