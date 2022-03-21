//
// Created by Chenming LI on 18/11/2021.
//

#ifndef BIAIT_DEV_BIAIT_H
#define BIAIT_DEV_BIAIT_H

#include "BiAITstar/Edge.h"
#include "BiAITstar/WeakEdge.h"
#include "BiAITstar/ImplicitGraph.h"

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/util/Console.h>


using namespace ompl::geometric::biait;

namespace ompl::geometric{

        class BiAIT : public ompl::base::Planner {
        public:

            explicit BiAIT(const base::SpaceInformationPtr& spaceInformationPtr);

            ~BiAIT() override = default;

            void setup() override;

            void clear() override;

            base::PlannerStatus::StatusType ensureSetup();

            base::PlannerStatus::StatusType ensureStartAndGoalStates(const base::PlannerTerminationCondition & terminationCondition);

            base::PlannerStatus solve(const base::PlannerTerminationCondition & terminationCondition) override;

            /* ##########  ##########  ##########  ##########  ########## */
            /*                     Setter and Getter                      */
            /* ##########  ##########  ##########  ##########  ########## */

            void setUseKNearest(bool useKNearest) { implicitGraph_.setUseKNearest(useKNearest); }

            bool getUseKNearest() const { return implicitGraph_.getUseKNearest(); }

            void setRewireFactor(double rewireFactor) { implicitGraph_.setRewireFactor(rewireFactor); }

            double getRewireFactor() const { return implicitGraph_.getRewireFactor(); }

            void setBatchSize(std::size_t batchSize) { batchSize_ = batchSize; }

            std::size_t getBatchSize() const { return batchSize_; }

            void setEnablePruning(bool enablePruning) { isPruningEnabled_ = enablePruning; }

            bool isPruningEnabled() const { return isPruningEnabled_; }

            void setMaxNumberOfGoals(std::size_t maxNumberOfGoals) { implicitGraph_.setMaxNumberOfGoals(maxNumberOfGoals); }

            std::size_t getMaxNumberOfGoals() const { return implicitGraph_.getMaxNumberOfGoals(); }

        private:
            /* ##########  ##########  ##########  ##########  ########## */
            /*                     Private Functions                      */
            /* ##########  ##########  ##########  ##########  ########## */

            void insertStartVerticesIntoForwardLazyQueue();

            void insertGoalVerticesIntoReverseLazyQueue();

            void insertValidTreeIntoLazyQueue();

            void insertFVVertexIntoFLQueue(const std::shared_ptr<Vertex> & vertex);

            void insertRVVertexIntoRLQueue(const std::shared_ptr<Vertex> & vertex);

            void expandStartVerticesIntoForwardValidQueue();

            void expandGoalVerticesIntoReverseValidQueue();

            std::vector<Edge> getOutgoingEdges(const std::shared_ptr<Vertex> & vertex) const;

            static std::array<base::Cost, 3u> computeForwardEdgeKey(const base::OptimizationObjectivePtr & optObjPtr,
                                                                    const std::shared_ptr<Vertex> & parent,
                                                                    const std::shared_ptr<Vertex> & child);

            static std::array<base::Cost, 3u> computeReverseEdgeKey(const base::OptimizationObjectivePtr & optObjPtr,
                                                                    const std::shared_ptr<Vertex> & parent,
                                                                    const std::shared_ptr<Vertex> & child);

            std::array<base::Cost, 2u> computeForwardVertexKey(const std::shared_ptr<Vertex> & vertex) const;

            std::array<base::Cost, 2u> computeReverseVertexKey(const std::shared_ptr<Vertex> & vertex) const;

            base::Cost computeMeetLazyKey(const std::shared_ptr<Vertex> & parent, const std::shared_ptr<Vertex> & child);

            base::Cost computeMeetValidKey(const Edge & edge, const base::Cost & edgeCost) const;

            base::Cost computeCostToStartHeuristic(const std::shared_ptr<Vertex> & vertex) const { return computeBestCostHeuristic(implicitGraph_.getStartVertices(), vertex); }

            base::Cost computeCostToGoalHeuristic(const std::shared_ptr<Vertex> & vertex) const { return computeBestCostHeuristic(vertex, implicitGraph_.getGoalVertices()); }

            void insertOrUpdateForwardValidQueue(const Edge & edge);

            void insertOrUpdateForwardValidQueue(const std::vector<Edge> &edge);

            void insertOrUpdateReverseValidQueue(const Edge & edge);

            void insertOrUpdateReverseValidQueue(const std::vector<Edge> &edge);

            void insertOrUpdateForwardLazyQueue(const std::shared_ptr<Vertex> &vertex);

            void insertOrUpdateReverseLazyQueue(const std::shared_ptr<Vertex> &vertex);

            void insertOrUpdateMeetLazyQueue(const std::weak_ptr<Vertex> & parent, const std::weak_ptr<Vertex> & child);

            bool iterate(const base::PlannerTerminationCondition & terminationCondition);

            base::PlannerStatus::StatusType updateSolution();

            void updateExactSolution();

            void informAboutPlannerStatus(base::PlannerStatus::StatusType status) const;

            void informRunTimeStatus() const;

            void informNewSolution() const;

            // The efficiency of the following four functions is not important;
            std::size_t countNumVerticesInForwardValidPortion() const;

            std::size_t countNumVerticesInForwardLazyPortion() const;

            std::size_t countNumVerticesInReverseValidPortion() const;

            std::size_t countNumVerticesInReverseLazyPortion() const;

            std::shared_ptr<geometric::PathGeometric> getPathToGoal() const;

            bool isEdgeBetter(const biait::Edge& lhs, const biait::Edge & rhs) const;

            bool isMeetValidEdgeBetter(const biait::CostEdgePair & lhs, const biait::CostEdgePair & rhs) const {
                return optObjPtr_->isCostBetterThan(lhs.second.getMeetValidEdgeKey(), rhs.second.getMeetValidEdgeKey());
            }

            bool isMeetLazyEdgeBetter(const biait::WeakEdge & lhs, const biait::WeakEdge & rhs) const {
                return optObjPtr_->isCostBetterThan(lhs.getMeetLazyEdgeKey(), rhs.getMeetLazyEdgeKey());
            }

            bool isVertexBetter(const biait::KeyVertexPair &lhs, const biait::KeyVertexPair &rhs) const;

            base::Cost computeBestCostHeuristic(const std::shared_ptr<Vertex> & vertex, const std::vector<std::shared_ptr<Vertex> > & vectorVertex) const;

            base::Cost computeBestCostHeuristic(const std::vector<std::shared_ptr<Vertex> > & vectorVertex, const std::shared_ptr<Vertex> & vertex) const;

            void updateCostToStartOfForwardValidDescendant(const std::shared_ptr<Vertex> & vertex) const;

            void updateCostToGoalOfReverseValidDescendant(const std::shared_ptr<Vertex> & vertex) const;

            void updateCostToStartOfRVPredecessor(const std::shared_ptr<Vertex> & vertex) const;

            void insertOrUpdateMeetValidEdge(Edge & selectedValidEdge, const base::Cost & edgeCost);

            void insertOrUpdateMeetValidEdge(const std::shared_ptr<Vertex> & vertex) const;

            void insertOrUpdateMeetValidEdge(const std::shared_ptr<Vertex> & parent, const std::shared_ptr<Vertex> & child);

            void updateCostToStartOfReverseValidPrecedent(const std::shared_ptr<Vertex> & vertex) const;

            void updateCostToGoalOfForwardValidPrecedent(const std::shared_ptr<Vertex> & vertex) const;

            /* ##########  ##########  ##########  ##########  ########## */
            // Functions called in function `iterate()`;
            /* ##########  ##########  ##########  ##########  ########## */

            bool performForwardLazySearch();

            bool performReverseLazySearch();

            bool performForwardValidSearch();

            bool performReverseValidSearch();

            // Functions `iterateForwardLazySearch` and `iterateReverseLazySearch` only do one-step search when called;
            void iterateForwardLazySearch();

            void iterateReverseLazySearch();

            // Functions `iterateForwardValidSearch` and `iterateReverseValidSearch` only do one-step search when called;
            void iterateForwardValidSearch();

            void iterateReverseValidSearch();

            void clearLazyQueue();

            void clearForwardLazyQueue();

            void clearReverseLazyQueue();

            void clearValidQueue();

            void clearForwardValidQueue();

            void clearReverseValidQueue();

            void clearMeetLazyQueue();

            void invalidateForwardLazyBranch(const std::shared_ptr<Vertex> &vertex);

            void invalidateRLWhenInvalidatingFL(const std::shared_ptr<Vertex> &vertex, const std::shared_ptr<Vertex> &);

            void invalidateFLToStartWhenInvalidatingFL(const std::shared_ptr<Vertex> &vertex,
                                                       const std::shared_ptr<Vertex> &propagateFrom);

            void invalidateReverseLazyBranch(const std::shared_ptr<Vertex> &vertex);

            void invalidateFLWhenInvalidatingRL(const std::shared_ptr<Vertex> &vertex,
                                                const std::shared_ptr<Vertex> &propagateFrom);

            void invalidateRLToGoalWhenInvalidatingRL(const std::shared_ptr<Vertex> &vertex,
                                                      const std::shared_ptr<Vertex> &propagateFrom);

            void invalidateAllLazyComponent();

            // Functions `updateForwardLazySearchVertex` and `updateReverseLazySearchVertex` will update a vertex in the lazy queue;
            void updateVertexInFLSearch(const std::shared_ptr<Vertex> & vertex);

            void updateVertexInFLSearchToFLTree(const std::shared_ptr<Vertex> & vertex);

            void updateVertexInFLSearchToMeetQueue(const std::shared_ptr<Vertex> & vertex);

            // Find the best parent in FL search for vertex, and the bestCost via the `bestParent` is stored in the `bestCost`;
            void updateVertexInFLSearch_bestParent(const std::shared_ptr<Vertex> & vertex, std::shared_ptr<Vertex>& bestParent, base::Cost& bestCost);

            // Can not pass a weak_ptr pointing to nullptr to function, so the returned flag shows whether found the bestParent;
            bool updateVertexInFLSearch_bestWeakParent(const std::shared_ptr<Vertex> & vertex, std::weak_ptr<Vertex>& bestParent, base::Cost& bestCost);

            void updateVertexInRLSearch(const std::shared_ptr<Vertex> & vertex);

            void updateVertexInRLSearchToRLTree(const std::shared_ptr<Vertex> & vertex);

            void updateVertexInRLSearchToMeetQueue(const std::shared_ptr<Vertex> & vertex);

            // Find the best parent in RL search for vertex, and the bestCost via the `bestParent` is stored in the `bestCost`;
            void updateVertexInRLSearch_bestParent(const std::shared_ptr<Vertex> & vertex, std::shared_ptr<Vertex>& bestParent, base::Cost& bestCost);

            // Can not pass a weak_ptr pointing to nullptr to function, so the returned flag shows whether found the bestParent;
            bool updateVertexInRLSearch_bestWeakParent(const std::shared_ptr<Vertex> & vertex, std::weak_ptr<Vertex>& bestParent, base::Cost& bestCost);

            void updateForwardLazyNeighbors(const std::shared_ptr<Vertex> & vertex);

            void updateReverseLazyNeighbors(const std::shared_ptr<Vertex> & vertex);

            void propagateCostFromStartInRLTree(const std::shared_ptr<Vertex> & vertex);

            void propagateCostToGoalInFLTree(const std::shared_ptr<Vertex> & vertex);

            void updateValidQueueIfVertexInside(const std::shared_ptr<Vertex> & vertex);

            void updateFVValidQueueIfVertexInside(const std::shared_ptr<Vertex> & vertex);

            void updateRVValidQueueIfVertexInside(const std::shared_ptr<Vertex> & vertex);

            // If found, return the pointer to that in the meetLazyQueue, otherwise, return nullptr;
            MeetLazyEdgeQueue::Element * findInMeetLazyQueue(const Edge & edge);

            // If found, return the pointer to that in the meetLazyQueue, otherwise, return nullptr;
            MeetLazyEdgeQueue::Element * findInMeetLazyQueue(const WeakEdge & edge);

            // If found, return the pointer to that in the meetValidQueue, otherwise, return nullptr;
            MeetValidEdgeQueue::Element * findInMeetValidQueue(const Edge & edge);

            void removeFromMeetValidQueue(const std::vector<MeetValidEdgeQueue::Element *>& vectorOfMeetValidEdgesToBePruned);

            void checkMeetLazyEdgeToValid(MeetLazyEdgeQueue::Element * meetLazyEdgePointer);

            /* ##########  ##########  ##########  ##########  ########## */
            /*                          Members                           */
            /* ##########  ##########  ##########  ##########  ########## */

            base::Cost solutionCost_;

            biait::ImplicitGraph implicitGraph_;

            // In forward valid queue, the parent is close to the start and child is far from the start;
            biait::EdgeQueue forwardValidQueue_;

            // In forward valid queue, the parent is close to the goal and child is far from the goal;
            biait::EdgeQueue reverseValidQueue_;

            // We have to make sure the parent of MeetEdge is close to the start, and child is close to the goal(s);
            // We use this one for the valid trees meet;
            mutable biait::MeetValidEdgeQueue meetValidEdgeQueue_;

            // \hat{h}_{g-FL}(x_p) + \hat{c}(x_p, x_c) + \hat{h}_{g-RL}(x_c);
            mutable biait::MeetLazyEdgeQueue meetLazyEdgeQueue_;

            biait::VertexQueue forwardLazyQueue_;

            biait::VertexQueue reverseLazyQueue_;

            std::size_t numIterations_{0u};

            std::size_t batchSize_{500u};

            bool isPruningEnabled_{true};

            base::OptimizationObjectivePtr optObjPtr_{nullptr};

            base::SpaceInformationPtr spaceInformationPtr_{nullptr};

            base::MotionValidatorPtr motionValidatorPtr_{nullptr};

            std::size_t numProcessedEdges_{0u};

            std::size_t numEdgeCollisionChecks_{0u};

            std::size_t numInconsistentOrUnconnectedTargetsInForward_{0u};

            std::size_t numInconsistentOrUnconnectedTargetsInReverse_{0u};

            std::size_t numFLIteration{0u};

            std::size_t numRLIteration{0u};

            std::size_t numFVIteration{0u};

            std::size_t numRVIteration{0u};
        };
}


#endif //BIAIT_DEV_BIAIT_H
