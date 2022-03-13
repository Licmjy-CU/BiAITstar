//
// Created by Chenming LI on 18/1/2022.
//

#ifndef BIAIT_DEV_WEAKEDGE_H
#define BIAIT_DEV_WEAKEDGE_H

#include <array>
#include <limits>
#include <memory>
#include <bitset>

#include <ompl/base/Cost.h>

//#include "BiAITstar/Queuetypes.h"
#include "BiAITstar/Edge.h"


namespace ompl::geometric::biait {

    class WeakEdge {

    public:
        WeakEdge();

        // Constructor for meet edges, default assign the category_ as 0b00100, and edgeKey as signaling_NaN;
        WeakEdge(std::weak_ptr<Vertex> parent,
                std::weak_ptr<Vertex> child,
                const base::Cost & meetLazyEdgeKey);

        explicit WeakEdge(const Edge& edge);

        ~WeakEdge() = default;

        /* ##########  ##########  ##########  ##########  ########## */
        /*                     Setter and Getter                      */
        /* ##########  ##########  ##########  ##########  ########## */

        std::weak_ptr<Vertex> getParent() const { return parent_; }

        std::weak_ptr<Vertex> getChild() const { return child_; }

        const base::Cost & getMeetLazyEdgeKey() const { return meetLazyEdgeKey_; }

        void setMeetLazyEdgeKey(const base::Cost meetLazyEdgeKey) { meetLazyEdgeKey_ = meetLazyEdgeKey; }

    private:

        /* ##########  ##########  ##########  ##########  ########## */
        /*                          Private                           */
        /* ##########  ##########  ##########  ##########  ########## */
        std::weak_ptr<Vertex> parent_;

        std::weak_ptr<Vertex> child_;

        // We have to make sure the parent of MeetEdge is to the start, and child to the goal(s);
        base::Cost meetLazyEdgeKey_;
    };

}


#endif //BIAIT_DEV_WEAKEDGE_H
