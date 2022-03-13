//
// Created by licm on 27/10/2021.
//

#include "BiAITstar/Vertex.h"


using ompl::geometric::biait::Vertex;


Vertex::Vertex(const base::SpaceInformationPtr &spaceInformationPtr,
                                       const base::ProblemDefinitionPtr &problemDefinitionPtr,
                                       const size_t &batchId)
                                       : spaceInformationPtr_(spaceInformationPtr)
                                       , problemDefinitionPtr_(problemDefinitionPtr)
                                       , optObjPtr_(problemDefinitionPtr->getOptimizationObjective())
                                       , forwardValidChildren_()
                                       , reverseValidChildren_()
                                       , forwardLazyChildren_()
                                       , reverseLazyChildren_()
                                       , forwardValidParent_()
                                       , reverseValidParent_()
                                       , forwardLazyParent_()
                                       , reverseLazyParent_()
                                       , statePtr_(spaceInformationPtr->allocState())
                                       , vertexId_(generateId())
                                       , batchId_(batchId) {
    setCostsToInfinity();
    debugVertex = new int(128);
}


//Vertex::Vertex(const std::shared_ptr<Vertex> &other)
//    : spaceInformationPtr_(other->spaceInformationPtr_)
//    , problemDefinitionPtr_(other->problemDefinitionPtr_)
//    , optObjPtr_(other->optObjPtr_)
//    , forwardValidChildren_(other->forwardLazyChildren_)
//    , reverseValidChildren_(other->reverseValidChildren_)
//    , forwardLazyChildren_(other->forwardLazyChildren_)
//    , reverseLazyChildren_(other->reverseLazyChildren_)
//    , forwardValidParent_(other->forwardValidParent_)
//    , reverseValidParent_(other->reverseValidParent_)
//    , forwardLazyParent_(other->forwardLazyParent_)
//    , reverseLazyParent_(other->reverseLazyParent_)
//    , statePtr_(spaceInformationPtr_->allocState())
//    , heuristicCost_g_ForwardLazy_(other->heuristicCost_g_ForwardLazy_)
//    , heuristicCost_rhs_ForwardLazy_(other->heuristicCost_rhs_ForwardLazy_)
//    , heuristicCost_g_ReverseLazy_(other->heuristicCost_g_ReverseLazy_)
//    , heuristicCost_rhs_ReverseLazy_(other->heuristicCost_rhs_ReverseLazy_)
//    , edgeCostFromStart_(other->edgeCostFromStart_)
//    , edgeCostFromGoal_(other->edgeCostFromGoal_)
//    , costToStart_(other->costToStart_)
//    , costToGoal_(other->costToGoal_)
//    , vertexId_(other->vertexId_)
//    , batchId_(other->batchId_) {
//    spaceInformationPtr_->copyState(statePtr_, other->statePtr_);
//}
//

Vertex::~Vertex() {
    spaceInformationPtr_->freeState(statePtr_);
    delete debugVertex;
}


std::vector<std::weak_ptr<Vertex> > Vertex::invalidateForwardValidBranch(
        std::vector<Queuetypes::MeetValidEdgeQueue::Element *> & vectorOfMeetValidEdgesToBePruned) {
    std::vector<std::weak_ptr<Vertex> > accumulatedChildren = forwardValidChildren_;
    // accumulatedChildren.insert(accumulatedChildren.end(), forwardLazyChildren_.begin(), forwardLazyChildren_.end());
    for(const auto & elem: accumulatedChildren){
        if(auto elemSharedPtr = elem.lock()){
            elemSharedPtr->costToStart_ =  optObjPtr_->infiniteCost();
            if(elemSharedPtr->hasForwardValidParent())
                elemSharedPtr->resetForwardValidParent();
            auto elem_sAccumulatedChildren = elemSharedPtr->invalidateForwardValidBranch(
                    vectorOfMeetValidEdgesToBePruned);   // s for shared_ptr;
            accumulatedChildren.insert(accumulatedChildren.end(),
                                       elem_sAccumulatedChildren.begin(), elem_sAccumulatedChildren.end());
        }
    }
    if(!meetValidEdgeQueuePointer_.empty()){
        for(const auto meetValidEdgePtr: meetValidEdgeQueuePointer_) {
            if(meetValidEdgePtr != nullptr && !Utility::containsByValue(vectorOfMeetValidEdgesToBePruned, meetValidEdgePtr)){
                Utility::removeFromVectorByValue(meetValidEdgePtr->data.second.getChild()->meetValidEdgeQueuePointer_, meetValidEdgePtr);
                Utility::removeFromVectorByValue(meetValidEdgePtr->data.second.getParent()->meetValidEdgeQueuePointer_, meetValidEdgePtr);
                vectorOfMeetValidEdgesToBePruned.emplace_back(meetValidEdgePtr);
            }
        }
    }
    forwardValidChildren_.clear();
    return accumulatedChildren;
}


std::vector<std::weak_ptr<Vertex> > Vertex::invalidateReverseValidBranch(
        std::vector<Queuetypes::MeetValidEdgeQueue::Element *> & vectorOfMeetValidEdgesToBePruned) {
    std::vector<std::weak_ptr<Vertex> > accumulatedChildren = reverseValidChildren_;
    // accumulatedChildren.insert(accumulatedChildren.end(), reverseLazyChildren_.begin(), reverseLazyChildren_.end());
    for(const auto & elem: accumulatedChildren){
        if(auto elemSharedPtr = elem.lock()){
            elemSharedPtr->costToGoal_ = optObjPtr_->infiniteCost();
            if(elemSharedPtr->hasForwardValidParent())
                elemSharedPtr->resetReverseValidParent();
            auto elem_sAccumulatedChildren = elemSharedPtr->invalidateReverseValidBranch(
                    vectorOfMeetValidEdgesToBePruned);   // s for shared_ptr;
            accumulatedChildren.insert(accumulatedChildren.end(),
                                       elem_sAccumulatedChildren.begin(), elem_sAccumulatedChildren.end());
        }
    }
    if(!meetValidEdgeQueuePointer_.empty()){
        for(const auto meetValidEdgePtr: meetValidEdgeQueuePointer_) {
            if(meetValidEdgePtr != nullptr && !Utility::containsByValue(vectorOfMeetValidEdgesToBePruned, meetValidEdgePtr)){
                Utility::removeFromVectorByValue(meetValidEdgePtr->data.second.getChild()->meetValidEdgeQueuePointer_, meetValidEdgePtr);
                Utility::removeFromVectorByValue(meetValidEdgePtr->data.second.getParent()->meetValidEdgeQueuePointer_, meetValidEdgePtr);
                vectorOfMeetValidEdgesToBePruned.emplace_back(meetValidEdgePtr);
            }
        }
    }
    reverseValidChildren_.clear();
    return accumulatedChildren;
}


std::vector<std::weak_ptr<Vertex> > Vertex::invalidateForwardLazyBranch() {

    std::vector<std::weak_ptr<Vertex> > accumulatedLazyChildren = forwardLazyChildren_;
    for(const auto & elem: accumulatedLazyChildren) {
        if(auto elemSharedPtr = elem.lock()) {
            elemSharedPtr->costToStart_ =  optObjPtr_->infiniteCost();
            elemSharedPtr->resetForwardLazyParent();
            auto elem_sAccumulatedChildren = elemSharedPtr->invalidateForwardLazyBranch();
            accumulatedLazyChildren.insert(accumulatedLazyChildren.end(),
                                           elem_sAccumulatedChildren.begin(), elem_sAccumulatedChildren.end());
        }
    }
    forwardLazyChildren_.clear();
    return accumulatedLazyChildren;
}


std::vector<std::weak_ptr<Vertex> > Vertex::invalidateReverseLazyBranch() {

    std::vector<std::weak_ptr<Vertex> > accumulatedLazyChildren = reverseLazyChildren_;
    for(const auto & elem: accumulatedLazyChildren) {
        if(auto elemSharedPtr = elem.lock()) {
            elemSharedPtr->costToGoal_ = optObjPtr_->infiniteCost();
            elemSharedPtr->resetReverseLazyParent();
            auto elem_sAccumulatedChildren = elemSharedPtr->invalidateReverseLazyBranch();
            accumulatedLazyChildren.insert(accumulatedLazyChildren.end(),
                                           elem_sAccumulatedChildren.begin(), elem_sAccumulatedChildren.end());
        }
    }
    reverseLazyChildren_.clear();
    return accumulatedLazyChildren;
}


/* ##########  ##########  ##########  ##########  ########## */
void Vertex::addToForwardLazyChildren(const std::shared_ptr<Vertex> & vertex) {
    vertex->setCategory(3, true);
    if(std::find_if(forwardLazyChildren_.begin(), forwardLazyChildren_.end(),
                    [vertex](const std::weak_ptr<Vertex>& child){ return vertex->getId() == child.lock()->getId();})
                    == forwardLazyChildren_.end()){
        forwardLazyChildren_.emplace_back(vertex);
    }
}


void Vertex::removeFromForwardLazyChildren(std::size_t vertexId) {
    auto iter = std::find_if(
            forwardLazyChildren_.begin(), forwardLazyChildren_.end(),
            [vertexId](const std::weak_ptr<Vertex> & child) { return vertexId == child.lock()->getId(); }
            );
    if(iter == forwardLazyChildren_.end()){
        return;
    }
    // Reset the iter's category;
    iter->lock()->setCategory(3, false);
    // Swap and pop;
    std::iter_swap(iter, forwardLazyChildren_.rbegin());
    forwardLazyChildren_.pop_back();
}


std::vector<std::shared_ptr<Vertex> > Vertex::getForwardLazyChildren() const {
    std::vector<std::shared_ptr<Vertex> > outputLazyChildren;
    for(const auto & elem: forwardLazyChildren_) {
//        assert(!elem.expired());
        if(elem.expired()) continue;
        outputLazyChildren.emplace_back(elem.lock());
    }
    return outputLazyChildren;
}


bool Vertex::hasForwardLazyChild(const std::shared_ptr<Vertex> & vertex) const {
    return std::any_of(forwardLazyChildren_.begin(), forwardLazyChildren_.end(),
                [&vertex](const std::weak_ptr<Vertex>& elem){ return elem.lock()->getId() == vertex->getId();} );
}


/* ##########  ##########  ##########  ##########  ########## */
void Vertex::addToReverseLazyChildren(const std::shared_ptr<Vertex> & vertex) {
    vertex->setCategory(0, true);
    if(std::find_if(reverseLazyChildren_.begin(), reverseLazyChildren_.end(),
                    [vertex](const std::weak_ptr<Vertex>& child){ return vertex->getId() == child.lock()->getId();})
       == reverseLazyChildren_.end()){
        reverseLazyChildren_.emplace_back(vertex);
    }
}


void Vertex::removeFromReverseLazyChildren(std::size_t vertexId) {
    auto iter = std::find_if(
            reverseLazyChildren_.begin(), reverseLazyChildren_.end(),
            [vertexId](const std::weak_ptr<Vertex> & child) { return vertexId == child.lock()->getId(); }
    );
    if(iter == reverseLazyChildren_.end()){
        return;
    }
    // Reset the iter's category;
    iter->lock()->setCategory(0, false);
    // Swap and pop;
    std::iter_swap(iter, reverseLazyChildren_.rbegin());
    reverseLazyChildren_.pop_back();
}


std::vector<std::shared_ptr<Vertex> > Vertex::getReverseLazyChildren() const {
    std::vector<std::shared_ptr<Vertex> > outputLazyChildren;
    for(const auto & elem: reverseLazyChildren_) {
//        assert(!elem.expired());
        if(elem.expired()) continue;
        outputLazyChildren.emplace_back(elem.lock());
    }
    return outputLazyChildren;
}


bool Vertex::hasReverseLazyChild(const std::shared_ptr<Vertex> & vertex) const {
    return std::any_of(reverseLazyChildren_.begin(), reverseLazyChildren_.end(),
                       [&vertex](const std::weak_ptr<Vertex>& elem){ return elem.lock()->getId() == vertex->getId();} );
}


/* ##########  ##########  ##########  ##########  ########## */
void Vertex::addToForwardValidChildren(const std::shared_ptr<Vertex> & vertex) {
    vertex->setCategory(4, true);
    forwardValidChildren_.emplace_back(vertex);
}


void Vertex::removeFromForwardValidChildren(std::size_t vertexId) {
    auto iter = std::find_if(
            forwardValidChildren_.begin(), forwardValidChildren_.end(),
            [vertexId](const std::weak_ptr<Vertex> & child) { return vertexId == child.lock()->getId(); }
            );
    if(iter == forwardValidChildren_.end()){
        return;
    }
    // Reset the iter's category;
    iter->lock()->setCategory(4, false);
    // Swap and pop;
    std::iter_swap(iter, forwardValidChildren_.rbegin());
    forwardValidChildren_.pop_back();
}


std::vector<std::shared_ptr<Vertex> > Vertex::getForwardValidChildren() const {
    std::vector<std::shared_ptr<Vertex> > outputValidChildren;
    for(const auto & elem: forwardValidChildren_) {
//        assert(!elem.expired());
        outputValidChildren.emplace_back(elem.lock());
    }
    return outputValidChildren;
}


std::vector<std::weak_ptr<Vertex> > Vertex::getForwardValidWeakChildren() const {
    std::vector<std::weak_ptr<Vertex> > outputWeakChildren;
    for(const auto & elem: forwardValidChildren_) {
        outputWeakChildren.emplace_back(elem);
    }
    return outputWeakChildren;
}


/* ##########  ##########  ##########  ##########  ########## */
void Vertex::addToReverseValidChildren(const std::shared_ptr<Vertex> & vertex) {
    vertex->setCategory(1, true);
    reverseValidChildren_.emplace_back(vertex);
}


void Vertex::removeFromReverseValidChildren(std::size_t vertexId) {
    auto iter = std::find_if(
            reverseValidChildren_.begin(), reverseValidChildren_.end(),
            [vertexId](const std::weak_ptr<Vertex> & child) { return vertexId == child.lock()->getId(); }
    );
    if(iter == reverseValidChildren_.end()){
        return;
    }
    // Reset the iter's category;
    iter->lock()->setCategory(1, false);
    // Swap and pop;
    std::iter_swap(iter, reverseValidChildren_.rbegin());
    reverseValidChildren_.pop_back();
}


std::vector<std::shared_ptr<Vertex> > Vertex::getReverseValidChildren() const {
    std::vector<std::shared_ptr<Vertex> > outputValidChildren;
    for(const auto & elem: reverseValidChildren_) {
//        assert(!elem.expired());
        outputValidChildren.emplace_back(elem.lock());
    }
    return outputValidChildren;
}


std::vector<std::weak_ptr<Vertex> > Vertex::getReverseValidWeakChildren() const {
    std::vector<std::weak_ptr<Vertex> > outputWeakChildren;
    for(const auto & elem: reverseValidChildren_) {
        outputWeakChildren.emplace_back(elem);
    }
    return outputWeakChildren;
}


/* ##########  ##########  ##########  ##########  ########## */
void Vertex::removeFromForwardChildren(std::size_t vertexId){
    removeFromForwardLazyChildren(vertexId);
    removeFromForwardValidChildren(vertexId);
}


void Vertex::removeFromReverseChildren(std::size_t vertexId){
    removeFromReverseLazyChildren(vertexId);
    removeFromReverseValidChildren(vertexId);
}


void Vertex::removeFromChildren(std::size_t vertexId) {
    removeFromForwardChildren(vertexId);
    removeFromReverseChildren(vertexId);
}


bool Vertex::hasLazyChild(const std::shared_ptr<Vertex> & vertex) const{
    return hasForwardLazyChild(vertex) || hasReverseLazyChild(vertex);
}


//bool Vertex::isConsistent() const {
//    if(category_[3] && optObjPtr_->isFinite(heuristicCost_forwardLazy_))
//        return optObjPtr_->isCostEquivalentTo(heuristicCost_expandForwardLazy_, heuristicCost_forwardLazy_);
//    else if(category_[0] && optObjPtr_->isFinite(heuristicCost_reverseLazy_))
//        return optObjPtr_->isCostEquivalentTo(heuristicCost_expandReverseLazy_, heuristicCost_reverseLazy_);
//    return false;
//}

bool Vertex::isConsistentInForwardLazySearch() const {
    return optObjPtr_->isCostEquivalentTo(heuristicCost_rhs_ForwardLazy_, heuristicCost_g_ForwardLazy_);
}


bool Vertex::isConsistentInReverseLazySearch() const {
    return optObjPtr_->isCostEquivalentTo(heuristicCost_rhs_ReverseLazy_, heuristicCost_g_ReverseLazy_);
}


void Vertex::callOnValidChildrenBranch(const std::function<void(const std::shared_ptr<Vertex> &)> &function) {
    // TODO: leave blank;
}


void Vertex::callOnLazyChildrenBranch(const std::function<void(const std::shared_ptr<Vertex> &)> &function) {
    // TODO: leave blank;
}



/* ##########  ##########  ##########  ##########  ########## */
/* setter and getter */
/* ##########  ##########  ##########  ##########  ########## */

void Vertex::setForwardValidParent(const std::shared_ptr<Vertex> &vertex, const base::Cost &edgeCost) {
    if(auto parentSharedPtr_ = forwardValidParent_.lock()){
        parentSharedPtr_->removeFromForwardValidChildren(vertexId_);
    }
    edgeCostFromStart_ = edgeCost;
    forwardValidParent_ = std::weak_ptr<Vertex>(vertex);
    costToStart_ = optObjPtr_->combineCosts(vertex->costToStart_, edgeCost);
}


void Vertex::setReverseValidParent(const std::shared_ptr<Vertex> &vertex, const base::Cost &edgeCost) {
    if(auto parentSharedPtr_ = reverseValidParent_.lock()){
        parentSharedPtr_->removeFromReverseValidChildren(vertexId_);
    }
    edgeCostFromGoal_ = edgeCost;
    reverseValidParent_ = std::weak_ptr<Vertex>(vertex);
    costToGoal_ = optObjPtr_->combineCosts(vertex->costToGoal_, edgeCost);
}


void Vertex::setForwardLazyParent(const std::shared_ptr<Vertex> &vertex) {
    // renew the old parent's lazy children;
    if(auto parentSharedPtr_ = forwardLazyParent_.lock()){
        parentSharedPtr_->removeFromForwardLazyChildren(vertexId_);
    }
    // Cache the new parent;
    forwardLazyParent_ = std::weak_ptr<Vertex>(vertex);
    setCategory(3, true);
    setCategory(0, false);
}

void Vertex::setReverseLazyParent(const std::shared_ptr<Vertex> &vertex) {
    // renew the old parent's lazy children;
    if(auto parentSharedPtr_ = reverseLazyParent_.lock()){
        parentSharedPtr_->removeFromReverseLazyChildren(vertexId_);
    }
    // Cache the new parent;
    reverseLazyParent_ = std::weak_ptr<Vertex>(vertex);
    setCategory(0, true);
    setCategory(3, false);
}


void Vertex::removeFromValidQueueIncomingLookupFromStart(Queuetypes::EdgeQueue::Element * pointer) {
    validQueueIncomingLookupFromStart_.erase(
        std::remove(validQueueIncomingLookupFromStart_.begin(), validQueueIncomingLookupFromStart_.end(), pointer), validQueueIncomingLookupFromStart_.end()
    );
}


void Vertex::removeFromValidQueueOutgoingLookupFromStart(Queuetypes::EdgeQueue::Element * pointer) {
    validQueueOutgoingLookupFromStart_.erase(
        std::remove(validQueueOutgoingLookupFromStart_.begin(), validQueueOutgoingLookupFromStart_.end(), pointer), validQueueOutgoingLookupFromStart_.end()
    );
}


void Vertex::removeFromValidQueueIncomingLookupFromGoal(Queuetypes::EdgeQueue::Element * pointer) {
    validQueueIncomingLookupFromGoal_.erase(
            std::remove(validQueueIncomingLookupFromGoal_.begin(), validQueueIncomingLookupFromGoal_.end(), pointer), validQueueIncomingLookupFromGoal_.end()
    );
}


void Vertex::removeFromValidQueueOutgoingLookupFromGoal(Queuetypes::EdgeQueue::Element * pointer) {
    validQueueOutgoingLookupFromGoal_.erase(
            std::remove(validQueueOutgoingLookupFromGoal_.begin(), validQueueOutgoingLookupFromGoal_.end(), pointer), validQueueOutgoingLookupFromGoal_.end()
    );
}


/* ##########  ##########  ##########  ##########  ########## */
/* setter and getter */
/* ##########  ##########  ##########  ##########  ########## */


bool Vertex::hasWhitelistedChild(const std::shared_ptr<Vertex> &vertex) const {
    // Check if the vertex is whitelisted by iterating over all whitelisted children.
    // It this detects an invalid vertex, e.g., a vertex that was once whitelisted but
    // has been pruned since, remove the vertex from the list of whitelisted children.
    auto iter = whitelistedChildren_.begin();
    while( iter != whitelistedChildren_.end() ){    // can only use while-loop here;
        if(const auto elem = iter->lock()){
            if(elem->getId() == vertex->getId()){
                return true;
            }
            ++iter;
        } else {
            iter = whitelistedChildren_.erase(iter);
        }
    }
    return false;
}


bool Vertex::hasBlacklistedChild(const std::shared_ptr<Vertex> &vertex) const {
    auto iter = blacklistedChildren_.begin();
    while(iter != blacklistedChildren_.end()){
        if(const auto elem = iter->lock()){
            if(elem->getId() == vertex->getId()){
                return true;
            }
            ++iter;
        } else {
            iter = blacklistedChildren_.erase(iter);
        }
    }
    return false;
}


void Vertex::cacheNeighbors(const std::vector<std::shared_ptr<Vertex>> &neighbors) const {
    cachedNeighbors_.clear();
    cachedNeighbors_.reserve(neighbors.size());
    cachedNeighbors_.insert(cachedNeighbors_.begin(), neighbors.begin(), neighbors.end());
    cachedNeighborBatchId_ = batchId_;
}


std::vector<std::shared_ptr<Vertex> > Vertex::getCachedNeighbors() const {
    if(cachedNeighborBatchId_ != batchId_){
        throw ompl::Exception("Requested neighbors from vertex of outdated approximation");
    }

    std::vector<std::shared_ptr<Vertex> > cachedNeighbors{};
    cachedNeighbors.reserve(cachedNeighbors_.size());
    for(const auto &neighbor : cachedNeighbors_){
        assert(neighbor.lock());
        cachedNeighbors.emplace_back(neighbor.lock());
    }
    return cachedNeighbors;
}


void Vertex::setForwardLazyQueuePointer(typename Queuetypes::VertexQueue::Element * pointer) {
    forwardLazyQueuePointer_ = pointer;
}


void Vertex::setReverseLazyQueuePointer(typename Queuetypes::VertexQueue::Element * pointer) {
    reverseLazyQueuePointer_ = pointer;
}


//ompl::BinaryHeap<ompl::geometric::biait::Queuetypes::KeyVertexPair,
//                 std::function<bool(const ompl::geometric::biait::Queuetypes::KeyVertexPair &,
//                                    const ompl::geometric::biait::Queuetypes::KeyVertexPair &)>>::Element * Vertex::getOppositeQueuePointer() const {
//    if(batchId_ != lazyQueuePointerId_){
//        lazyQueuePointer_ = nullptr;
//    }
//    return lazyQueuePointer_;
//}


void Vertex::addToValidQueueIncomingLookupFromStart(Queuetypes::EdgeQueue::Element *pointer) {
    validQueueIncomingLookupFromStart_.emplace_back(pointer);
}


void Vertex::addToValidQueueOutgoingLookupFromStart(Queuetypes::EdgeQueue::Element *pointer) {
    validQueueOutgoingLookupFromStart_.emplace_back(pointer);
}


void Vertex::addToValidQueueIncomingLookupFromGoal(Queuetypes::EdgeQueue::Element *pointer) {
    validQueueIncomingLookupFromGoal_.emplace_back(pointer);
}


void Vertex::addToValidQueueOutgoingLookupFromGoal(Queuetypes::EdgeQueue::Element *pointer) {
    validQueueOutgoingLookupFromGoal_.emplace_back(pointer);
}


void ompl::geometric::biait::Vertex::setCostsToInfinity() {
    heuristicCost_rhs_ForwardLazy_ = optObjPtr_->infiniteCost();
    heuristicCost_rhs_ReverseLazy_ = optObjPtr_->infiniteCost();
    heuristicCost_g_ForwardLazy_ = optObjPtr_->infiniteCost();
    heuristicCost_g_ReverseLazy_ = optObjPtr_->infiniteCost();
    edgeCostFromStart_ = optObjPtr_->infiniteCost();
    edgeCostFromGoal_ = optObjPtr_->infiniteCost();
    costToStart_ = optObjPtr_->infiniteCost();
    costToGoal_ = optObjPtr_->infiniteCost();
}


void ompl::geometric::biait::Vertex::setHeuristicCostsToInfinity() {
    heuristicCost_rhs_ForwardLazy_ = optObjPtr_->infiniteCost();
    heuristicCost_rhs_ReverseLazy_ = optObjPtr_->infiniteCost();
    heuristicCost_g_ForwardLazy_ = optObjPtr_->infiniteCost();
    heuristicCost_g_ReverseLazy_ = optObjPtr_->infiniteCost();
}






