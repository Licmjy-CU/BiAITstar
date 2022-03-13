//
// Created by licm on 27/10/2021.
//

#ifndef BIAIT_DEV_VERTEX_H
#define BIAIT_DEV_VERTEX_H

#include "BiAITstar/include/GenerateId.h"
#include "BiAITstar/include/Queuetypes.h"
#include "BiAITstar/include/Edge.h"
#include "BiAITstar/include/Utility.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/State.h>

#include <algorithm>


namespace ompl::geometric::biait{

    class Vertex {
    public:

        Vertex(const base::SpaceInformationPtr & spaceInformationPtr,
               const base::ProblemDefinitionPtr & problemDefinitionPtr,
               const std::size_t & batchId);

//        explicit Vertex(const std::shared_ptr<Vertex> & other);

        ~Vertex();


        // Invalidate the cost to the whole branch rooted at this vertex;
        // TODO: whether update the cost of the opposite tree?
        // Recursively invalidates the branch;
        std::vector<std::weak_ptr<Vertex> > invalidateForwardValidBranch(
                std::vector<Queuetypes::MeetValidEdgeQueue::Element *> & vectorOfMeetValidEdgesToBePruned);

        std::vector<std::weak_ptr<Vertex> > invalidateReverseValidBranch(
                std::vector<Queuetypes::MeetValidEdgeQueue::Element *> & vectorOfMeetValidEdgesToBePruned);

        std::vector<std::weak_ptr<Vertex> > invalidateForwardLazyBranch();

        std::vector<std::weak_ptr<Vertex> > invalidateReverseLazyBranch();

        /* ##########  ##########  ##########  ##########  ########## */
        void addToForwardLazyChildren(const std::shared_ptr<Vertex> & vertex);

        void removeFromForwardLazyChildren(std::size_t vertexId);

        // To keep this handler consistent, define this getter here;
        std::vector<std::shared_ptr<Vertex> > getForwardLazyChildren() const;

        bool hasForwardLazyChild() const { return !forwardLazyChildren_.empty(); }

        bool hasForwardLazyChild(const std::shared_ptr<Vertex> & vertex) const;

        void resetForwardLazyChildren() { forwardLazyChildren_.clear(); }

        /* ##########  ##########  ##########  ##########  ########## */
        void addToReverseLazyChildren(const std::shared_ptr<Vertex> & vertex);

        void removeFromReverseLazyChildren(std::size_t vertexId);

        // To keep this handler consistent, define this getter here;
        std::vector<std::shared_ptr<Vertex> > getReverseLazyChildren() const;

        bool hasReverseLazyChild() const { return !reverseLazyChildren_.empty(); }

        bool hasReverseLazyChild(const std::shared_ptr<Vertex> & vertex) const;

        void resetReverseLazyChildren() { reverseLazyChildren_.clear(); }

        /* ##########  ##########  ##########  ##########  ########## */
        void addToForwardValidChildren(const std::shared_ptr<Vertex> & vertex);

        void removeFromForwardValidChildren(std::size_t vertexId);

        // To keep this handler consistent, define this getter here;
        std::vector<std::shared_ptr<Vertex> > getForwardValidChildren() const;

        std::vector<std::weak_ptr<Vertex> > getForwardValidWeakChildren() const;

        bool hasForwardValidChild() const { return !forwardValidChildren_.empty(); }

        void resetForwardValidChildren() { forwardValidChildren_.clear(); }

        /* ##########  ##########  ##########  ##########  ########## */
        void addToReverseValidChildren(const std::shared_ptr<Vertex> & vertex);

        void removeFromReverseValidChildren(std::size_t vertexId);

        // To keep this handler consistent, define this getter here;
        std::vector<std::shared_ptr<Vertex> > getReverseValidChildren() const;

        std::vector<std::weak_ptr<Vertex> > getReverseValidWeakChildren() const;

        bool hasReverseValidChild() const { return !reverseValidChildren_.empty(); }

        void resetReverseValidChildren() { reverseValidChildren_.clear(); }

        // Do removeFromForwardLazyChildren and removeFromForwardValidChildren;
        void removeFromForwardChildren(std::size_t vertexId);

        // Do removeFromReverseLazyChildren and removeFromReverseValidChildren;
        void removeFromReverseChildren(std::size_t vertexId);

        // Do `removeFromForwardChildren` and `removeFromReverseChildren`;
        void removeFromChildren(std::size_t vertexId);

        bool hasLazyChild() const { return hasForwardLazyChild() || hasReverseLazyChild(); }

        bool hasLazyChild(const std::shared_ptr<Vertex> & vertex) const;

        // Return whether the vertex is consistent:
        // Whether its cost-to-come is equal to the cost-to-come wehn it was last expanded;

        bool isConsistentInForwardLazySearch() const;

        bool isConsistentInReverseLazySearch() const;

        // Recursively call it on all valid children on the branch;
        void callOnValidChildrenBranch(const std::function<void(const std::shared_ptr<Vertex> &)> &function);

        // Recursively call it on all lazy children on the branch;
        void callOnLazyChildrenBranch(const std::function<void(const std::shared_ptr<Vertex> &)> &function);


        /* ##########  ##########  ##########  ##########  ########## */
        /* setter and getter */
        /* ##########  ##########  ##########  ##########  ########## */

        std::size_t getId() const { return vertexId_; }

        base::State * getState() {return statePtr_; }

        const base::State * getState() const { return statePtr_; }

        base::ScopedState<> getScopedState() const { return {spaceInformationPtr_->getStateSpace(), statePtr_}; }

        // Forward valid parent;
        bool hasForwardValidParent() const { return static_cast<bool>(forwardValidParent_.lock()); }

        void setForwardValidParent(const std::shared_ptr<Vertex> & vertex, const base::Cost & edgeCost);

        void resetForwardValidParent() { forwardValidParent_.reset();   category_[4] = false; }

        std::shared_ptr<Vertex> getForwardValidParent() const { return forwardValidParent_.lock(); }

        std::weak_ptr<Vertex> getForwardValidWeakParent() const { return forwardValidParent_; }

        // Reverse valid parent;
        bool hasReverseValidParent() const { return static_cast<bool>(reverseValidParent_.lock()); }

        void setReverseValidParent(const std::shared_ptr<Vertex> & vertex, const base::Cost & edgeCost);

        void resetReverseValidParent() { reverseValidParent_.reset();   category_[1] = false; }

        std::shared_ptr<Vertex> getReverseValidParent() const { return reverseValidParent_.lock(); }

        std::weak_ptr<Vertex> getReverseValidWeakParent() const { return reverseValidParent_; }

        // Forward lazy parent;
        bool hasForwardLazyParent() const { return static_cast<bool>(forwardLazyParent_.lock()); }

        void setForwardLazyParent(const std::shared_ptr<Vertex> & vertex);

        void resetForwardLazyParent() { forwardLazyParent_.reset();   category_[3] = false; }

        std::shared_ptr<Vertex> getForwardLazyParent() const { return forwardLazyParent_.lock(); }

        // Reverse lazy parent;
        bool hasReverseLazyParent() const { return static_cast<bool>(reverseLazyParent_.lock()); }

        void setReverseLazyParent(const std::shared_ptr<Vertex> & vertex);

        void resetReverseLazyParent() { reverseLazyParent_.reset();   category_[0] = false; }

        std::shared_ptr<Vertex> getReverseLazyParent() const { return reverseLazyParent_.lock(); }


        void whitelistAsChild(const std::shared_ptr<Vertex> &vertex) const { whitelistedChildren_.emplace_back(vertex); }

        bool hasWhitelistedChild(const std::shared_ptr<Vertex> &vertex) const;

        void blacklistAsChild(const std::shared_ptr<Vertex> &vertex) const { blacklistedChildren_.emplace_back(vertex); }

        bool hasBlacklistedChild(const std::shared_ptr<Vertex> &vertex) const;

        // Returns whether the vertex knows its nearest neighbors on the current approximation.
        bool hasCachedNeighbors() const { return cachedNeighborBatchId_ == batchId_; }

        void cacheNeighbors(const std::vector<std::shared_ptr<Vertex> > & neighbors) const;

        std::vector<std::shared_ptr<Vertex> > getCachedNeighbors() const;

        void setForwardLazyQueuePointer(typename Queuetypes::VertexQueue::Element * pointer);

        void setReverseLazyQueuePointer(typename Queuetypes::VertexQueue::Element * pointer);

        void resetForwardLazyQueuePointer() { forwardLazyQueuePointer_ = nullptr; }

        void resetReverseLazyQueuePointer() { reverseLazyQueuePointer_ = nullptr; }

        void addToValidQueueIncomingLookupFromStart(Queuetypes::EdgeQueue::Element *pointer);

        void addToValidQueueOutgoingLookupFromStart(Queuetypes::EdgeQueue::Element *pointer);

        void addToValidQueueIncomingLookupFromGoal(Queuetypes::EdgeQueue::Element *pointer);

        void addToValidQueueOutgoingLookupFromGoal(Queuetypes::EdgeQueue::Element *pointer);

        void removeFromValidQueueIncomingLookupFromStart(Queuetypes::EdgeQueue::Element * pointer);

        void removeFromValidQueueOutgoingLookupFromStart(Queuetypes::EdgeQueue::Element * pointer);

        void removeFromValidQueueIncomingLookupFromGoal(Queuetypes::EdgeQueue::Element * pointer);

        void removeFromValidQueueOutgoingLookupFromGoal(Queuetypes::EdgeQueue::Element * pointer);

        std::bitset<5> getCategory() const { return category_; }

        void setCategory(const std::bitset<5> category) { category_ = category; }

        void setCategory(std::size_t index, bool flag) { category_[index] = flag; }

        void resetCategory() { category_.reset(); }

        // Expose the costs to public interface, since they will be frequently used;
        // rhs means one-step lookahead cost heuristic; We follow the definition in LPA instead of AIT;
        base::Cost heuristicCost_rhs_ForwardLazy_{};

        base::Cost heuristicCost_rhs_ReverseLazy_{};

        // g means root distance (cost) FL and RL search; We follow the definition in LPA instead of AIT;
        base::Cost heuristicCost_g_ForwardLazy_{};

        base::Cost heuristicCost_g_ReverseLazy_{};

        base::Cost edgeCostFromStart_{};    // For valid edge;

        base::Cost edgeCostFromGoal_{};     // For valid edge;

        base::Cost costToStart_{};   // For forward valid vertex;

        base::Cost costToGoal_{};     // For reverse valid vertex;

        void setCostsToInfinity();

        void setHeuristicCostsToInfinity();

        mutable typename Queuetypes::VertexQueue::Element * forwardLazyQueuePointer_{nullptr};

        mutable typename Queuetypes::VertexQueue::Element * reverseLazyQueuePointer_{nullptr};

        mutable std::vector<Queuetypes::EdgeQueue::Element *> validQueueIncomingLookupFromStart_{};

        mutable std::vector<Queuetypes::EdgeQueue::Element *> validQueueOutgoingLookupFromStart_{};

        mutable std::vector<Queuetypes::EdgeQueue::Element *> validQueueIncomingLookupFromGoal_{};

        mutable std::vector<Queuetypes::EdgeQueue::Element *> validQueueOutgoingLookupFromGoal_{};

        mutable std::vector<Queuetypes::MeetValidEdgeQueue::Element *> meetValidEdgeQueuePointer_{};

        mutable std::vector<Queuetypes::MeetLazyEdgeQueue::Element *> meetLazyEdgeQueuePointer_{};

        mutable int * debugVertex{nullptr};

    private:
        /* ##########  ##########  ##########  ##########  ########## */
        /*                          Members                           */
        /* ##########  ##########  ##########  ##########  ########## */

        base::SpaceInformationPtr spaceInformationPtr_{nullptr};

        base::ProblemDefinitionPtr problemDefinitionPtr_{nullptr};

        base::OptimizationObjectivePtr optObjPtr_{nullptr};

        base::State * statePtr_{nullptr};

        const std::size_t vertexId_{0u};

        const std::size_t& batchId_;

        // batch ID for which the cached neighbor list is valid;
        mutable std::size_t cachedNeighborBatchId_{0u};

        std::weak_ptr<Vertex> forwardValidParent_;

        std::weak_ptr<Vertex> reverseValidParent_;

        std::weak_ptr<Vertex> forwardLazyParent_;

        std::weak_ptr<Vertex> reverseLazyParent_;

        // four byte represent forward valid, forward lazy, reverse valid, and reverse lazy, respectively.
        // assert(category_.count() == 1 || (category_.count() == 0 && statePtr_ == nullptr))
        std::bitset<5> category_{0b00000};

        // When isFrontLineVertex is true, parent_ become nullptr, otherwise frontLineParent_ is {nullptr, nullptr};
//        FrontLineParent frontLineParent_;

        std::vector<std::weak_ptr<Vertex> > forwardValidChildren_{};

        std::vector<std::weak_ptr<Vertex> > reverseValidChildren_{};

        std::vector<std::weak_ptr<Vertex> > forwardLazyChildren_{};

        std::vector<std::weak_ptr<Vertex> > reverseLazyChildren_{};

        mutable std::vector<std::weak_ptr<Vertex> > cachedNeighbors_{};

        mutable std::vector<std::weak_ptr<Vertex> > whitelistedChildren_{};

        mutable std::vector<std::weak_ptr<Vertex> > blacklistedChildren_{};


    }; // class Vertex;

} // namespace;

#endif //BIAIT_DEV_VERTEX_H