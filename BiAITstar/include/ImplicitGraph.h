//
// Created by licm on 28/10/2021.
//

#ifndef BIAIT_DEV_IMPLICITGRAPH_H
#define BIAIT_DEV_IMPLICITGRAPH_H

#include "BiAITstar/include/Vertex.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>

#include <ompl/util/GeometricEquations.h>

#include <vector>



namespace ompl::geometric::biait {

    class ImplicitGraph {
    public:
        explicit ImplicitGraph(const ompl::base::Cost & solutionCost_);

        void setup(ompl::base::SpaceInformationPtr & spaceInformationPtr,
                   ompl::base::ProblemDefinitionPtr & problemDefinitionPtr,
                   ompl::base::PlannerInputStates * plannerInputStatesPtr);

        void clear();   // Reset;

        // Return: vector of ptr that point to the meet valid edges which can not help to improve current solution anymore;
        std::vector<Queuetypes::MeetValidEdgeQueue::Element *> prune();

        void registerStartState(const base::State * const startState);

        void registerGoalState(const base::State * const goalState);

        void updateStartAndGoalStates(const base::PlannerTerminationCondition & terminationCondition,
                                      base::PlannerInputStates * plannerInputStatesPtr);

        bool addSamples(std::size_t numNewSamples,
                        const ompl::base::PlannerTerminationCondition & terminationCondition);

        /* ##########  ##########  ##########  ##########  ########## */
        /* setter and getter */
        std::size_t getBatchId() const { return batchId_; }

        void setRewireFactor(double rewireFactor) { rewireFactor_ = rewireFactor; }

        double getRewireFactor() const { return rewireFactor_; }

        void setMaxNumberOfGoals(std::size_t maxNumberOfGoals) { maxNumberOfGoals_ = maxNumberOfGoals; }

        std::size_t getMaxNumberOfGoals() const { return maxNumberOfGoals_; }

        void setUseKNearest(bool useKNearest) { useKNearest_ = useKNearest; }

        bool getUseKNearest() const { return useKNearest_; }

        std::size_t getNumOfVerticesGNAT() const { return verticesGNAT_.size(); }

        double getConnectionRadius() const { return radius_; }

        bool hasAStartState() const { return !startVertices_.empty(); }

        bool hasAGoalState() const { return !goalVertices_.empty(); }

        // Get neighbors of a vertex;
        std::vector<std::shared_ptr<Vertex> > getNeighbors(const std::shared_ptr<Vertex> &vertex) const;

//        std::vector<std::shared_ptr<Vertex> > getExpandRadiusNeighbors(const std::shared_ptr<Vertex> &vertex) const;

        bool isStart(const std::shared_ptr<Vertex> & vertex) const;

        bool isGoal(const std::shared_ptr<Vertex> & vertex) const;

        const std::vector<std::shared_ptr<Vertex> > & getStartVertices() const { return startVertices_; }

        const std::vector<std::shared_ptr<Vertex> > & getGoalVertices() const { return goalVertices_; }

        // Get all vertices;
        std::vector<std::shared_ptr<Vertex> > getVertices() const;

        std::size_t getNumberOfSampledStates() const { return numSampledStates_; }

        std::size_t getNumberOfValidSamples() const { return numValidSamples_; }

        // Each sampled state is checked here, while the edges do not count here;
        std::size_t getNumberOfStateCollisionChecks() const { return numSampledStates_; }

        std::size_t getNumberOfNearestNeighborCalls() const { return numNearestNeighborsCalls_; }

        void setBridgeSampleRate(const double bridgeSampleRate) { bridgeSampleRate_ = bridgeSampleRate; }

        double getBridgeSampleRate() const { return bridgeSampleRate_; }

    private:
        /* ##########  ##########  ##########  ##########  ########## */

        std::size_t computeNumberOfSamplesInInformedSet() const;

        // for r-disc model;
        double computeConnectionRadius(std::size_t numSamples) const;

        // for k-nearest model;
        std::size_t computeNumberOfNeighbors(std::size_t numSamples) const;

        bool canPossiblyImproveSolution(const std::shared_ptr<Vertex> & vertex) const;


        /* ##########  ##########  ##########  ##########  ########## */

        base::SpaceInformationPtr spaceInformationPtr_{nullptr};

        base::ProblemDefinitionPtr problemDefinitionPtr_{nullptr};

        base::OptimizationObjectivePtr optObjPtr_{nullptr};

        std::size_t batchId_{0};

        double rewireFactor_{0.8};

        // Whether to use a k-nearest RGG. If false, uses an r-disc RGG;
        bool useKNearest_{true};

        std::size_t maxNumberOfGoals_{3u};

        // The radius that defines the neighborhood of a vertex if using an r-disc graph;
        double radius_{std::numeric_limits<double>::infinity()};

        // The number of neighbors that defines the neighborhood of a vertex if using a k-nearest graph;
        std::size_t numNeighbors_{std::numeric_limits<std::size_t>::max()};

        // A constant for the computation of the number of neighbors when using a k-nearest model;
        std::size_t k_RGG_{std::numeric_limits<std::size_t>::max()};

        const ompl::base::Cost & solutionCost_;

        ompl::base::InformedSamplerPtr informedSamplerPtr_{};

        RNG rng_0_1_;

        std::unique_ptr<ompl::base::BridgeTestValidStateSampler> bridgeTestValidStateSamplerPtr_;

        double bridgeSampleRate_{0.0};

        // The implicit graph;
        ompl::NearestNeighborsGNATNoThreadSafety<std::shared_ptr<Vertex> > verticesGNAT_;

        std::vector<std::shared_ptr<Vertex> > startVertices_{};

        std::vector<std::shared_ptr<Vertex> > goalVertices_{};

        // The start vertices that have been pruned. They are kept around because if the user decides to
        // add goal states after we've pruned some start states, we might want to add these pruned start states
        // again;
        std::vector<std::shared_ptr<Vertex> > prunedStartVertices_{};

        // The goal vertices that have been pruned. They are kept around because if the user decides to
        // add start states after we've pruned some goal states, we might want to add these pruned goal states
        // again.
        std::vector<std::shared_ptr<Vertex> > prunedGoalVertices_{};

        std::vector<std::shared_ptr<Vertex> > newSamples_{};

        mutable std::size_t numValidSamples_{0u};

        mutable std::size_t numSampledStates_{0u};

        // The number of state collision checks;
        mutable std::size_t numNearestNeighborsCalls_{0u};

        std::vector<Queuetypes::MeetValidEdgeQueue::Element *> vectorOfMeetValidEdgesToBePruned_{};
    };
}




#endif //BIAIT_DEV_IMPLICITGRAPH_H
