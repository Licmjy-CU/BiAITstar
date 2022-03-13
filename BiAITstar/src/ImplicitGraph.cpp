//
// Created by licm on 28/10/2021.
//

#include "BiAITstar/ImplicitGraph.h"

#include <boost/math/constants/constants.hpp>

namespace ompl::geometric::biait {

    ImplicitGraph::ImplicitGraph(const ompl::base::Cost & solutionCost_)
        : batchId_(1u)
        , solutionCost_(solutionCost_)
        , verticesGNAT_(8, 4, 12, 50, 500, false){
        // verticesGNAT_ default param: degree = 8, minDegree = 4, maxDegree = 12, maxNumPtsPerLeaf = 50,
        // removedCacheSize = 500, rebalancing = false, (estimatedDimension = 6.0);
    }


    void ImplicitGraph::setup(base::SpaceInformationPtr &spaceInformationPtr,
                              base::ProblemDefinitionPtr &problemDefinitionPtr,
                              ompl::base::PlannerInputStates *plannerInputStatesPtr) {
        spaceInformationPtr_ = spaceInformationPtr;
        problemDefinitionPtr_ = problemDefinitionPtr;
        optObjPtr_ = problemDefinitionPtr_ -> getOptimizationObjective();

        verticesGNAT_.setDistanceFunction(
                [this](const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b) {
                    return spaceInformationPtr_->distance(a->getState(), b->getState()); }
                );

        k_RGG_ = static_cast<std::size_t>( boost::math::constants::e<double>() +
                 boost::math::constants::e<double>() / static_cast<double>(spaceInformationPtr_->getStateDimension()));

        updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition(), plannerInputStatesPtr);

        bridgeTestValidStateSamplerPtr_ = std::make_unique<ompl::base::BridgeTestValidStateSampler>(spaceInformationPtr.get());
    }


    void ImplicitGraph::clear() {
        batchId_ = 1u;
        radius_ = std::numeric_limits<double>::infinity();
        numNeighbors_ = std::numeric_limits<std::size_t>::max();
        verticesGNAT_.clear();
        startVertices_.clear();
        goalVertices_.clear();
        prunedStartVertices_.clear();
        prunedGoalVertices_.clear();
        numSampledStates_ = 0u;
        numValidSamples_ = 0u;
        numNearestNeighborsCalls_ = 0u;
    }


    std::vector<Queuetypes::MeetValidEdgeQueue::Element *> ImplicitGraph::prune() {
        vectorOfMeetValidEdgesToBePruned_.clear();
        if(!optObjPtr_->isFinite(solutionCost_)){
            return vectorOfMeetValidEdgesToBePruned_;
        }
        std::vector<std::shared_ptr<Vertex> > vertices{};
        verticesGNAT_.list(vertices);
        std::vector<std::shared_ptr<Vertex> > verticesToBePruned{};

        // Check each vertex whether it can be pruned;
        for(const auto & elem: vertices){
            if(!canPossiblyImproveSolution(elem)){
                if(isGoal(elem)){
                    prunedGoalVertices_.emplace_back(elem);
                } else if (isStart(elem)) {
                    prunedStartVertices_.emplace_back(elem);
                }
                verticesToBePruned.emplace_back(elem);
            }
        }
        // Remove all vertices to be pruned;
        for(const auto & elem: verticesToBePruned){
            if(elem->hasForwardValidParent()){
                elem->getForwardValidParent()->removeFromChildren(elem->getId());
                elem->invalidateForwardValidBranch(vectorOfMeetValidEdgesToBePruned_);
            } else if (elem->hasReverseValidParent()) {
                elem->getReverseValidParent()->removeFromChildren(elem->getId());
                elem->invalidateReverseValidBranch(vectorOfMeetValidEdgesToBePruned_);
            }
            if (elem->hasForwardLazyParent()) {
                elem->getForwardLazyParent()->removeFromChildren(elem->getId());
                elem->resetForwardLazyParent();
                elem->invalidateForwardLazyBranch();
            } else if (elem->hasReverseLazyParent()) {
                elem->getReverseLazyParent()->removeFromChildren(elem->getId());
                elem->resetReverseLazyParent();
                elem->invalidateReverseLazyBranch();
            }
            if(!elem->meetValidEdgeQueuePointer_.empty()){
                for(const auto meetValidEdgePtr: elem->meetValidEdgeQueuePointer_) {
                    if(meetValidEdgePtr != nullptr && !Utility::containsByValue(vectorOfMeetValidEdgesToBePruned_, meetValidEdgePtr)){
                        Utility::removeFromVectorByValue(meetValidEdgePtr->data.second.getChild()->meetValidEdgeQueuePointer_, meetValidEdgePtr);
                        Utility::removeFromVectorByValue(meetValidEdgePtr->data.second.getParent()->meetValidEdgeQueuePointer_, meetValidEdgePtr);
                        vectorOfMeetValidEdgesToBePruned_.emplace_back(meetValidEdgePtr);
                    }
                }
            }
            verticesGNAT_.remove(elem);
        }
        return vectorOfMeetValidEdgesToBePruned_;
    }


    void ImplicitGraph::registerStartState(const base::State *const startStateCPtr) {
        auto startVertexPtr = std::make_shared<Vertex>(spaceInformationPtr_, problemDefinitionPtr_, batchId_);
        spaceInformationPtr_->copyState(startVertexPtr->getState(), startStateCPtr);
        startVertexPtr->costToStart_ = optObjPtr_->identityCost();
        verticesGNAT_.add(startVertexPtr);
        startVertices_.emplace_back(startVertexPtr);
    }


    void ImplicitGraph::registerGoalState(const base::State *const goalStateCPtr) {
        auto goalVertexPtr = std::make_shared<Vertex>(spaceInformationPtr_, problemDefinitionPtr_, batchId_);
        spaceInformationPtr_->copyState(goalVertexPtr->getState(), goalStateCPtr);
        goalVertexPtr->costToGoal_ = optObjPtr_->identityCost();
        verticesGNAT_.add(goalVertexPtr);
        goalVertices_.emplace_back(goalVertexPtr);
    }


    void ImplicitGraph::updateStartAndGoalStates(const base::PlannerTerminationCondition &terminationCondition,
                                                 base::PlannerInputStates *plannerInputStatesPtr) {
        // track whether a new goal and/or a new start has been added;
        bool addedNewGoalState = false;         bool addedNewStartState = false;

        // First update the goals. We have to call inputStates->nextGoal(terminationCondition) at least once
        // (regardless of the return value of inputStates->moreGoalStates()) in case the termination condition
        // wants us to wait for a goal.
        do {
            auto newGoalState = plannerInputStatesPtr->nextGoal(terminationCondition);
            if(static_cast<bool>(newGoalState)){
                registerGoalState(newGoalState);
                addedNewGoalState = true;
            }
        } while(plannerInputStatesPtr->haveMoreGoalStates() && goalVertices_.size() <= maxNumberOfGoals_);

        // Update starts;
        do {
            auto newStartState = plannerInputStatesPtr->nextStart();
            if(static_cast<bool>(newStartState)){
                registerStartState(newStartState);
                addedNewStartState = true;
            }
        } while(plannerInputStatesPtr->haveMoreStartStates());

        // If added a new start and have previously pruned goals, try to revive the goal;
        if(addedNewStartState && !prunedGoalVertices_.empty()){
            std::vector< std::vector< std::shared_ptr<Vertex> >::iterator > revivedGoals;
            for(auto it=prunedGoalVertices_.begin(); it!=prunedGoalVertices_.end(); ++it){
                auto heuristicCost = optObjPtr_->infiniteCost();
                for(const auto & elem: startVertices_){
                    heuristicCost = optObjPtr_->betterCost(
                            heuristicCost, optObjPtr_->motionCostHeuristic(elem->getState(), (*it)->getState())
                            );
                }
                if(optObjPtr_->isCostBetterThan(heuristicCost, solutionCost_)){
                    registerGoalState((*it)->getState());
                    addedNewGoalState = true;
                    revivedGoals.emplace_back(it);
                }
            }
            // remove the revived goals from the pruned goals;
            for(const auto & elem: revivedGoals){
                std::iter_swap(elem, prunedGoalVertices_.rbegin());
                prunedGoalVertices_.pop_back();
            }
        }

        // If added a new goal and have previously pruned start states, try to revive the start;
        if(addedNewGoalState && !prunedStartVertices_.empty()){
            std::vector< std::vector< std::shared_ptr<Vertex> >::iterator > revivedStarts;
            for(auto it = prunedStartVertices_.begin(); it != prunedStartVertices_.end(); ++it){
                auto heuristicCost = optObjPtr_->infiniteCost();
                for(const auto & elem : goalVertices_){
                    heuristicCost = optObjPtr_->betterCost(
                            heuristicCost, optObjPtr_->motionCostHeuristic(elem->getState(), (*it)->getState())
                            );
                }
                if(optObjPtr_->isCostBetterThan(heuristicCost, solutionCost_)){
                    registerStartState((*it)->getState());
                    addedNewStartState = true;
                    revivedStarts.emplace_back(it);
                }
            }
            // Remove all revived starts from the pruned starts;
            for(const auto & elem: revivedStarts){
                std::iter_swap(elem, prunedStartVertices_.rbegin());
                prunedStartVertices_.pop_back();
            }
        }

        if(addedNewGoalState || addedNewStartState){
            if(!startVertices_.empty() && !goalVertices_.empty()){
                informedSamplerPtr_ = optObjPtr_->allocInformedStateSampler(problemDefinitionPtr_,
                                                                            std::numeric_limits<unsigned int>::max());
            }
        }
        if(!goalVertices_.empty() && startVertices_.empty()){
            OMPL_ERROR("BiAIT find at least one goal, but no source point");
        }
    }


    bool ImplicitGraph::addSamples(std::size_t numNewSamples,
                                   const base::PlannerTerminationCondition &terminationCondition) {
        if(numNewSamples == 0) return true;
        newSamples_.reserve(numNewSamples);
        do {
            newSamples_.emplace_back(std::make_shared<Vertex>(spaceInformationPtr_, problemDefinitionPtr_, batchId_));
            do {
                if(rng_0_1_.uniform01() < bridgeSampleRate_){
                    std::shared_ptr<Vertex> newSample = std::make_shared<Vertex>(spaceInformationPtr_,
                                                                                 problemDefinitionPtr_,
                                                                                 batchId_);
                    bridgeTestValidStateSamplerPtr_->sample(newSample->getState());
                    if(canPossiblyImproveSolution(newSample)){
                        spaceInformationPtr_->copyState(newSamples_.back()->getState(), newSample->getState());
                    } else {
                        informedSamplerPtr_->sampleUniform(newSamples_.back()->getState(), solutionCost_);
                    }
                } else {
                    informedSamplerPtr_->sampleUniform(newSamples_.back()->getState(), solutionCost_);
                }
                ++numSampledStates_;
            } while( !spaceInformationPtr_->getStateValidityChecker()->isValid(newSamples_.back()->getState()) );
            if(problemDefinitionPtr_->getGoal()->isSatisfied(newSamples_.back()->getState())){
                goalVertices_.emplace_back(newSamples_.back());
                newSamples_.back()->costToGoal_ = optObjPtr_->identityCost();
            }
            //TODO: Left the state which located in the start region unprocessed;
            ++numValidSamples_;
        } while(newSamples_.size() < numNewSamples && !terminationCondition);


        if(newSamples_.size() == numNewSamples) {
            auto numSamplesInInformedSet = computeNumberOfSamplesInInformedSet();
            if(useKNearest_){
                numNeighbors_ = computeNumberOfNeighbors(numSamplesInInformedSet + numNewSamples - startVertices_.size() - goalVertices_.size());
            } else {
                radius_ = computeConnectionRadius(numSamplesInInformedSet + numNewSamples - startVertices_.size() - goalVertices_.size());
            }
            verticesGNAT_.add(newSamples_);
            newSamples_.clear();
            ++batchId_;
            return true;
        }
        return false;
    }


    std::vector<std::shared_ptr<Vertex> > ImplicitGraph::getNeighbors(const std::shared_ptr<Vertex> &vertex) const {
        if(vertex->hasCachedNeighbors()){   // TODO: check the batchID, maybe this is a bug;
            return vertex->getCachedNeighbors();
        } else {
            ++numNearestNeighborsCalls_;
            std::vector<std::shared_ptr<Vertex> > neighbors{};
            if(useKNearest_){
                verticesGNAT_.nearestK(vertex, numNeighbors_, neighbors);
            } else {
                verticesGNAT_.nearestR(vertex, radius_, neighbors);
            }
            vertex->cacheNeighbors(neighbors);
            return neighbors;
        }
    }


//    std::vector<std::shared_ptr<Vertex> > ImplicitGraph::getExpandRadiusNeighbors(const std::shared_ptr<Vertex> &vertex) const {
//        double factor{1.5};
//        std::vector<std::shared_ptr<Vertex> > neighbors{};
//        if(useKNearest_){
//            verticesGNAT_.nearestK(vertex, static_cast<std::size_t>(static_cast<double>(numNeighbors_) * factor), neighbors);
//        } else {
//            verticesGNAT_.nearestR(vertex, radius_ * factor, neighbors);
//        }
//        vertex->cacheNeighbors(neighbors);
//        return neighbors;
//    }


    bool ImplicitGraph::isStart(const std::shared_ptr<Vertex> &vertex) const {
//        for(const auto &elem: startVertices_){
//            if(elem->getId() == vertex->getId()){
//                return true;
//            }
//        }
        if( std::any_of(startVertices_.begin(), startVertices_.end(),
                        [&vertex](const std::shared_ptr<Vertex>& elem){ return elem->getId() == vertex->getId();} ) )
            return true;
        return false;
    }


    bool ImplicitGraph::isGoal(const std::shared_ptr<Vertex> &vertex) const {
        if( std::any_of(goalVertices_.begin(), goalVertices_.end(),
                        [&vertex](const std::shared_ptr<Vertex>& elem){ return elem->getId() == vertex->getId();} ) )
            return true;
        return false;
    }


    std::vector<std::shared_ptr<Vertex> > ImplicitGraph::getVertices() const {
        std::vector<std::shared_ptr<Vertex> > vertices;
        verticesGNAT_.list(vertices);
        return vertices;
    }


    /* ##########  ##########  ##########  ##########  ########## */
    // Private Functions;
    /* ##########  ##########  ##########  ##########  ########## */

    std::size_t ImplicitGraph::computeNumberOfSamplesInInformedSet() const {
        // Loop over verticesGNAT_ and count the ones in the informed set;
        std::size_t numberOfSamplesInInformedSet{0u};
        for(const auto & elemVertex: getVertices()){
            // Get best cost-to-come from root;
            auto bestCostToRoot = optObjPtr_->infiniteCost();
            for(const auto & elemStart: startVertices_){
                bestCostToRoot = optObjPtr_->betterCost(
                        bestCostToRoot, optObjPtr_->motionCostHeuristic(elemStart->getState(), elemVertex->getState())
                        );
            }

            // Get best cost-to-go to opposite root;
            auto bestCostToOppositeRoot = optObjPtr_->infiniteCost();
            for(const auto & elemGoal: goalVertices_){
                bestCostToOppositeRoot = optObjPtr_->betterCost(
                        bestCostToOppositeRoot, optObjPtr_->motionCostHeuristic(elemVertex->getState(), elemGoal->getState())
                        );
            }

            if(optObjPtr_->isCostBetterThan(optObjPtr_->combineCosts(bestCostToRoot, bestCostToOppositeRoot),
                                            solutionCost_)){
                ++numberOfSamplesInInformedSet;
            }
        }
        return numberOfSamplesInInformedSet;
    }


    double ImplicitGraph::computeConnectionRadius(std::size_t numSamples) const {
        auto dimension = static_cast<double>(spaceInformationPtr_->getStateDimension());
        double rewireFactor = std::pow(2.0*(1.0 + 1.0 / dimension) *
                                       (informedSamplerPtr_->getInformedMeasure(solutionCost_) /
                                       unitNBallMeasure(static_cast<unsigned int>(dimension)) ) *
                                       (std::log(static_cast<double>(numSamples)) / static_cast<double>(numSamples)),
                                       1.0 / dimension);
        return rewireFactor;
    }


    std::size_t ImplicitGraph::computeNumberOfNeighbors(std::size_t numSamples) const {
        return std::ceil(rewireFactor_ * static_cast<double>(k_RGG_) * std::log(static_cast<double>(numSamples)));
    }


    bool ImplicitGraph::canPossiblyImproveSolution(const std::shared_ptr<Vertex> &vertex) const {
        // TODO: why dont we store the bestCostToRoot in the Vertex class;
        auto bestCostToRoot = optObjPtr_->infiniteCost();
        for(const auto & elemStart : startVertices_){
            bestCostToRoot = optObjPtr_->betterCost(bestCostToRoot,
                                                    optObjPtr_->motionCostHeuristic(elemStart->getState(), vertex->getState()));
        }

        return optObjPtr_->isCostBetterThan(
                optObjPtr_->combineCosts(
                        bestCostToRoot, optObjPtr_->costToGo(vertex->getState(), problemDefinitionPtr_->getGoal().get())
                        ),
                solutionCost_
                );
    }



}
