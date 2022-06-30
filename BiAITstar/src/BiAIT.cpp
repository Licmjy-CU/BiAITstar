//
// Created by Chenming LI on 18/11/2021.
//

#include "BiAITstar/BiAIT.h"

namespace ompl::geometric {

BiAIT::BiAIT(const base::SpaceInformationPtr &spaceInformationPtr)
    : base::Planner(spaceInformationPtr, "BiAIT"),
      solutionCost_(),
      implicitGraph_(solutionCost_),
      forwardValidQueue_([this](const auto &lhs, const auto &rhs) {
        return isEdgeBetter(lhs, rhs);
      }),
      reverseValidQueue_([this](const auto &lhs, const auto &rhs) {
        return isEdgeBetter(lhs, rhs);
      }),
      meetValidEdgeQueue_([this](const auto &lhs, const auto &rhs) {
        return isMeetValidEdgeBetter(lhs, rhs);
      }),
      meetLazyEdgeQueue_([this](const auto &lhs, const auto &rhs) {
        return isMeetLazyEdgeBetter(lhs, rhs);
      }),
      forwardLazyQueue_([this](const auto &lhs, const auto &rhs) {
        return isVertexBetter(lhs, rhs);
      }),
      reverseLazyQueue_([this](const auto &lhs, const auto &rhs) {
        return isVertexBetter(lhs, rhs);
      }),
      spaceInformationPtr_(spaceInformationPtr) {
  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.multithreaded = false;
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = true;
  specs_.directed = true;
  specs_.provingSolutionNonExistence = false;
  specs_.canReportIntermediateSolutions = false;

  // Parameters and setter & getter;
  declareParam<bool>("use_k_nearest", this, &BiAIT::setUseKNearest,
                     &BiAIT::getUseKNearest, "0,1");
  declareParam<double>("rewire_factor", this, &BiAIT::setRewireFactor,
                       &BiAIT::getRewireFactor, "1.0:0.01:3.0");
  declareParam<std::size_t>("batch_size", this, &BiAIT::setBatchSize,
                            &BiAIT::getBatchSize, "1:1:1000");
  declareParam<bool>("enable_pruning", this, &BiAIT::setEnablePruning,
                     &BiAIT::isPruningEnabled, "0,1");
  declareParam<std::size_t>("max_num_goals", this, &BiAIT::setMaxNumberOfGoals,
                            &BiAIT::getMaxNumberOfGoals, "1:1:1000");

  // Progress properties;
  addPlannerProgressProperty("iterations INTEGER", [this]() {
    return std::to_string(numIterations_);
  });
  addPlannerProgressProperty("best cost DOUBLE", [this]() {
    return std::to_string(solutionCost_.value());
  });
}

void BiAIT::setup() {
  Planner::setup();
  if (static_cast<bool>(Planner::pdef_)) {
    if (!Planner::pdef_->hasOptimizationObjective()) {
      OMPL_WARN(
          "%s: No optimization objective has been specified. Defaulting to "
          "path length.",
          Planner::getName().c_str());
      Planner::pdef_->setOptimizationObjective(
          std::make_shared<ompl::base::PathLengthOptimizationObjective>(
              Planner::si_));
    }
    if (static_cast<bool>(Planner::pdef_->getGoal())) {
      if (!(Planner::pdef_->getGoal()->hasType(base::GOAL_SAMPLEABLE_REGION))) {
        OMPL_ERROR("Goal should be ompl::base::GOAL_SAMPLEABLE_GOAL_REGION");
        setup_ = false;
        return;
      }
    }
    optObjPtr_ = Planner::pdef_->getOptimizationObjective();
    solutionCost_ = optObjPtr_->infiniteCost();
    motionValidatorPtr_ = spaceInformationPtr_->getMotionValidator();
    implicitGraph_.setup(spaceInformationPtr_, Planner::pdef_,
                         &pis_);  // pis: PlannerInputState
  } else {
    setup_ = false;
    OMPL_ERROR("Fail to setup");
  }
}

void BiAIT::clear() {
  forwardValidQueue_.clear();
  reverseValidQueue_.clear();
  forwardLazyQueue_.clear();
  reverseLazyQueue_.clear();
  meetValidEdgeQueue_.clear();
  clearMeetLazyQueue();
  solutionCost_ = optObjPtr_->infiniteCost();
  numIterations_ = 0u;
  numInconsistentOrUnconnectedTargetsInForward_ = 0u;
  numInconsistentOrUnconnectedTargetsInReverse_ = 0u;
  implicitGraph_.clear();
  Planner::clear();
  setup_ = false;
}

base::PlannerStatus::StatusType BiAIT::ensureSetup() {
  Planner::checkValidity();
  if (!setup_) {
    OMPL_ERROR("%s: The planner is not setup. ", name_.c_str());
    return base::PlannerStatus::StatusType::ABORT;
  }
  if (!spaceInformationPtr_->isSetup()) {
    OMPL_ERROR("%s: The spaceInformation is not setup. ", name_.c_str());
    return base::PlannerStatus::StatusType::ABORT;
  }
  return base::PlannerStatus::StatusType::UNKNOWN;
}

base::PlannerStatus::StatusType BiAIT::ensureStartAndGoalStates(
    const base::PlannerTerminationCondition &terminationCondition) {
  if (!implicitGraph_.hasAStartState() || !implicitGraph_.hasAGoalState()) {
    implicitGraph_.updateStartAndGoalStates(terminationCondition, &pis_);
    if (!implicitGraph_.hasAStartState()) {
      OMPL_WARN("%s: No solution can be found as no start states are available",
                name_.c_str());
      return base::PlannerStatus::StatusType::INVALID_START;
    }
    if (!implicitGraph_.hasAGoalState()) {
      OMPL_WARN("%s: No solution can be found as no goal states are available",
                name_.c_str());
      return base::PlannerStatus::StatusType::INVALID_GOAL;
    }
  }
  return base::PlannerStatus::StatusType::UNKNOWN;
}

base::PlannerStatus BiAIT::solve(
    const base::PlannerTerminationCondition &terminationCondition) {
  // Ensure planning problem is successfully set up;
  base::PlannerStatus::StatusType plannerStatus = ensureSetup();
  if (plannerStatus == base::PlannerStatus::StatusType::ABORT) {
    return plannerStatus;
  }
  base::PlannerStatus::StatusType startGoalStatus =
      ensureStartAndGoalStates(terminationCondition);
  if (startGoalStatus == base::PlannerStatus::StatusType::INVALID_START ||
      startGoalStatus == base::PlannerStatus::StatusType::INVALID_GOAL) {
    return startGoalStatus;
  }

  // Inform the initiated planning status;
  OMPL_INFORM(
      "%s: Solving the given planning problem. \nThe current best solution "
      "cost is %.4f. ",
      name_.c_str(), solutionCost_.value());

  insertStartVerticesIntoForwardLazyQueue();
  insertGoalVerticesIntoReverseLazyQueue();
  expandStartVerticesIntoForwardValidQueue();
  expandGoalVerticesIntoReverseValidQueue();

  while (!terminationCondition && !optObjPtr_->isSatisfied(solutionCost_)) {
    if (!iterate(terminationCondition)) break;
  }

  // Inform the solution status;
  plannerStatus = updateSolution();
  informAboutPlannerStatus(plannerStatus);
  return plannerStatus;
}

/* ##########  ##########  ##########  ##########  ########## */
/*                     Private Functions                      */
/* ##########  ##########  ##########  ##########  ########## */

void BiAIT::insertStartVerticesIntoForwardLazyQueue() {
  for (const auto &elem : implicitGraph_.getStartVertices()) {
    elem->setCategory(3, true);
    elem->heuristicCost_rhs_ForwardLazy_ = optObjPtr_->identityCost();
    elem->heuristicCost_g_ForwardLazy_ = optObjPtr_->infiniteCost();
    KeyVertexPair elemKeyPair(
        {computeCostToGoalHeuristic(elem), optObjPtr_->identityCost()}, elem);
    elem->setForwardLazyQueuePointer(forwardLazyQueue_.insert(elemKeyPair));
  }
}

void BiAIT::insertGoalVerticesIntoReverseLazyQueue() {
  for (const auto &elem : implicitGraph_.getGoalVertices()) {
    elem->setCategory(0, true);
    elem->heuristicCost_rhs_ReverseLazy_ = optObjPtr_->identityCost();
    elem->heuristicCost_g_ReverseLazy_ = optObjPtr_->infiniteCost();
    KeyVertexPair elemKeyPair(
        {computeCostToStartHeuristic(elem), optObjPtr_->identityCost()}, elem);
    elem->setReverseLazyQueuePointer(reverseLazyQueue_.insert(elemKeyPair));
  }
}

void BiAIT::insertValidTreeIntoLazyQueue() {
  auto verticesVector = implicitGraph_.getVertices();
  for (const auto &vertex : verticesVector) {
    if (vertex->getCategory()[4]) {
      insertFVVertexIntoFLQueue(vertex);
    }
    if (vertex->getCategory()[1]) {
      insertRVVertexIntoRLQueue(vertex);
    }
  }
}

void BiAIT::insertFVVertexIntoFLQueue(const std::shared_ptr<Vertex> &vertex) {
  vertex->setCategory(3, true);
  vertex->setForwardLazyParent(vertex->getForwardValidParent());
  for (const auto &elem : vertex->getForwardValidChildren()) {
    vertex->addToForwardLazyChildren(elem);
  }
  vertex->heuristicCost_rhs_ForwardLazy_ = vertex->costToStart_;
  vertex->heuristicCost_g_ForwardLazy_ = optObjPtr_->infiniteCost();
  KeyVertexPair elemKeyPair(
      {optObjPtr_->combineCosts(vertex->heuristicCost_rhs_ForwardLazy_,
                                computeCostToGoalHeuristic(vertex)),
       vertex->heuristicCost_rhs_ForwardLazy_},
      vertex);
  vertex->setForwardLazyQueuePointer(forwardLazyQueue_.insert(elemKeyPair));
}

void BiAIT::insertRVVertexIntoRLQueue(const std::shared_ptr<Vertex> &vertex) {
  vertex->setCategory(0, true);
  vertex->setReverseLazyParent(vertex->getReverseValidParent());
  for (const auto &elem : vertex->getReverseValidChildren()) {
    vertex->addToReverseLazyChildren(elem);
  }
  vertex->heuristicCost_rhs_ReverseLazy_ = vertex->costToGoal_;
  vertex->heuristicCost_g_ReverseLazy_ = optObjPtr_->infiniteCost();
  KeyVertexPair elemKeyPair(
      {optObjPtr_->combineCosts(vertex->heuristicCost_rhs_ReverseLazy_,
                                computeCostToStartHeuristic(vertex)),
       vertex->heuristicCost_rhs_ReverseLazy_},
      vertex);
  vertex->setReverseLazyQueuePointer(reverseLazyQueue_.insert(elemKeyPair));
}

void BiAIT::expandStartVerticesIntoForwardValidQueue() {
  for (const auto &elem : implicitGraph_.getStartVertices()) {
    elem->costToStart_ = optObjPtr_->identityCost();
    elem->setCategory(4, true);
    for (const auto &edge : getOutgoingEdges(elem)) {
      insertOrUpdateForwardValidQueue(edge);
    }
  }
}

void BiAIT::expandGoalVerticesIntoReverseValidQueue() {
  for (const auto &elem : implicitGraph_.getGoalVertices()) {
    elem->costToGoal_ = optObjPtr_->identityCost();
    elem->setCategory(1, true);
    for (const auto &edge : getOutgoingEdges(elem)) {
      insertOrUpdateReverseValidQueue(edge);
    }
  }
}

std::vector<Edge> BiAIT::getOutgoingEdges(
    const std::shared_ptr<Vertex> &vertex) const {
  // assert vertex is a valid vertex;
  assert(vertex->getCategory()[4] || vertex->getCategory()[1]);

  // And find the outgoing edge category;
  std::bitset<5> outgoingValidEdgeCategory{};
  std::bitset<5> outgoingLazyEdgeCategory{};
  std::function<std::array<base::Cost, 3u>(
      const base::OptimizationObjectivePtr &, const std::shared_ptr<Vertex> &,
      const std::shared_ptr<Vertex> &)>
      computeEdgeKey;

  // Insert the current valid children;
  std::vector<Edge> outgoingEdges{};
  if (vertex->getCategory()[4]) {
    outgoingValidEdgeCategory = 0b10000;
    outgoingLazyEdgeCategory = 0b01000;
    computeEdgeKey = ompl::geometric::BiAIT::computeForwardEdgeKey;
    for (const auto &elem : vertex->getForwardValidChildren()) {
      outgoingEdges.emplace_back(vertex, elem, outgoingValidEdgeCategory,
                                 computeEdgeKey(optObjPtr_, vertex, elem));
    }
  } else if (vertex->getCategory()[1]) {
    outgoingValidEdgeCategory = 0b00010;
    outgoingLazyEdgeCategory = 0b00001;
    computeEdgeKey = ompl::geometric::BiAIT::computeReverseEdgeKey;
    for (const auto &elem : vertex->getReverseValidChildren()) {
      outgoingEdges.emplace_back(vertex, elem, outgoingValidEdgeCategory,
                                 computeEdgeKey(optObjPtr_, vertex, elem));
    }
  } else {
    return outgoingEdges;
  }
  // Handle the current neighbors in the GNAT;
  for (const auto &elem : implicitGraph_.getNeighbors(vertex)) {
    // Avoid self loop;
    if (vertex->getId() == elem->getId()) continue;
    // has Parent, has elem as Parent, it will be explicit added later;
    if (vertex->hasLazyChild() && vertex->hasLazyChild(elem)) continue;
    // Avoid blacklist edges;
    if (elem->hasBlacklistedChild(vertex) || vertex->hasBlacklistedChild(elem))
      continue;
    outgoingEdges.emplace_back(vertex, elem, outgoingLazyEdgeCategory,
                               computeEdgeKey(optObjPtr_, vertex, elem));
  }
  // Handle the edge which has a lazy connection with vertex;
  if (vertex->hasForwardLazyChild()) {
    for (const auto &elem : vertex->getForwardLazyChildren()) {
      outgoingEdges.emplace_back(vertex, elem, 0b01000,
                                 computeEdgeKey(optObjPtr_, vertex, elem));
    }
  }
  if (vertex->hasReverseLazyChild()) {
    for (const auto &elem : vertex->getReverseLazyChildren()) {
      outgoingEdges.emplace_back(vertex, elem, 0b00001,
                                 computeEdgeKey(optObjPtr_, vertex, elem));
    }
  }
  return outgoingEdges;
}

std::array<base::Cost, 3u> BiAIT::computeForwardEdgeKey(
    const base::OptimizationObjectivePtr &optObjPtr,
    const std::shared_ptr<Vertex> &parent,
    const std::shared_ptr<Vertex> &child) {
  // Parent is closer to the start;
  // g_F(x_p) + \hat{c}(x_p, x_c) + g_{R}(x_c),
  // g_F(x_p) + \hat{c}(x_p, x_c) + \hat{h}_{R}(x_c),
  // g_F(x_p) + \hat{c}(x_p, x_c)
  // g_F(x_p)
  base::Cost edgeCostHeuristic =
      optObjPtr->motionCostHeuristic(parent->getState(), child->getState());
  return {optObjPtr->combineCosts(
              optObjPtr->combineCosts(parent->costToStart_, edgeCostHeuristic),
              child->heuristicCost_g_ReverseLazy_),
          optObjPtr->combineCosts(parent->costToStart_, edgeCostHeuristic),
          parent->costToStart_};
}

std::array<base::Cost, 3u> BiAIT::computeReverseEdgeKey(
    const base::OptimizationObjectivePtr &optObjPtr,
    const std::shared_ptr<Vertex> &parent,
    const std::shared_ptr<Vertex> &child) {
  // Parent is closer to the goal;
  //  g_R(x_p) + \hat{c}(x_p, x_c) + g_{F}(x_c),
  //  g_R(x_p) + \hat{c}(x_p, x_c) + \hat{h}_{F}(x_c),
  //  g_R(x_p) + \hat{c}(x_p, x_c)
  //  g_R(x_p)
  base::Cost edgeCostHeuristic =
      optObjPtr->motionCostHeuristic(parent->getState(), child->getState());
  return {optObjPtr->combineCosts(
              optObjPtr->combineCosts(parent->costToGoal_, edgeCostHeuristic),
              child->heuristicCost_g_ForwardLazy_),
          optObjPtr->combineCosts(parent->costToGoal_, edgeCostHeuristic),
          parent->costToGoal_};
}

std::array<base::Cost, 2u> BiAIT::computeForwardVertexKey(
    const std::shared_ptr<Vertex> &vertex) const {
  // { min(h_hat{g-FL}(x), h_hat{rhs-FL}(x)) + g_hat{goal}, min(h_hat{g-FL}(x),
  // h_hat{rhs-FL}(x))};
  auto betterHeuristic =
      optObjPtr_->betterCost(vertex->heuristicCost_g_ForwardLazy_,
                             vertex->heuristicCost_rhs_ForwardLazy_);
  return {optObjPtr_->combineCosts(betterHeuristic,
                                   computeCostToGoalHeuristic(vertex)),
          betterHeuristic};
}

std::array<base::Cost, 2u> BiAIT::computeReverseVertexKey(
    const std::shared_ptr<Vertex> &vertex) const {
  //  { min(h_hat{g-RL}(x), h_hat{rhs-RL}(x)) + g_hat{start},
  //  min(h_hat{g-RL}(x), h_hat{rhs-RL}(x))};
  auto betterHeuristic =
      optObjPtr_->betterCost(vertex->heuristicCost_g_ReverseLazy_,
                             vertex->heuristicCost_rhs_ReverseLazy_);
  return {optObjPtr_->combineCosts(betterHeuristic,
                                   computeCostToStartHeuristic(vertex)),
          betterHeuristic};
}

base::Cost BiAIT::computeMeetLazyKey(const std::shared_ptr<Vertex> &parent,
                                     const std::shared_ptr<Vertex> &child) {
  // Make sure parent in FL tree and child in RL tree;
  return optObjPtr_->combineCosts(
      optObjPtr_->combineCosts(parent->heuristicCost_g_ForwardLazy_,
                               optObjPtr_->motionCostHeuristic(
                                   parent->getState(), child->getState())),
      child->heuristicCost_g_ReverseLazy_);
}

base::Cost BiAIT::computeMeetValidKey(const Edge &edge,
                                      const base::Cost &edgeCost) const {
  // The meetEdge key is parent->costToCome + edgeCost + child->costToGo;
  return optObjPtr_->combineCosts(
      optObjPtr_->combineCosts(edge.getParent()->costToStart_, edgeCost),
      edge.getChild()->costToGoal_);
}

void BiAIT::insertOrUpdateForwardValidQueue(const Edge &edge) {
  // If the edge is an meetLazyEdge or the parent and child lies in different
  // lazy trees; Currently we omit this situation and leave it for the
  // meetValidEdge to handle;
  if (findInMeetLazyQueue(edge) || edge.getChild()->getCategory()[0] ||
      edge.getChild()->getCategory()[1]) {
    return;
  }
  if ((edge.getParent()->getCategory() | edge.getChild()->getCategory())[0] &&
      (edge.getParent()->getCategory() | edge.getChild()->getCategory())[3]) {
    return;
  }
  // If the edge is included in the reverseValidQueue, we do nothing and return;
  // TODO: May this be the chance for trees meet;
  const auto iter_ReverseQueue = std::find_if(
      edge.getParent()->validQueueIncomingLookupFromGoal_.begin(),
      edge.getParent()->validQueueIncomingLookupFromGoal_.end(),
      [&edge](const auto elem) {
        return elem->data.getParent()->getId() == edge.getChild()->getId();
      });
  if (iter_ReverseQueue !=
      edge.getParent()->validQueueIncomingLookupFromGoal_.end())
    return;

  // Check if the edge is already in the queue and can be updated;
  const auto iter_ForwardQueue = std::find_if(
      edge.getChild()->validQueueIncomingLookupFromStart_.begin(),
      edge.getChild()->validQueueIncomingLookupFromStart_.end(),
      [&edge](const auto elem) {
        return elem->data.getParent()->getId() == edge.getParent()->getId();
      });
  if (iter_ForwardQueue !=
      edge.getChild()->validQueueIncomingLookupFromStart_.end()) {
    // Use the incoming lookup of the child;
    // Assert it is already in the outgoing lookup in of the parent;
    assert(
        std::find_if(
            edge.getParent()->validQueueOutgoingLookupFromStart_.begin(),
            edge.getParent()->validQueueOutgoingLookupFromStart_.end(),
            [&edge](const auto elem) {
              return elem->data.getChild()->getId() == edge.getChild()->getId();
            }) != edge.getParent()->validQueueOutgoingLookupFromStart_.end());
    // This edge exists in the queue, if the new sortKey is better than the old,
    // update;
    if (isEdgeBetter(edge, (*iter_ForwardQueue)->data)) {
      (*iter_ForwardQueue)->data.setEdgeKey(edge.getEdgeKey());
      forwardValidQueue_.update(*iter_ForwardQueue);
    }
  } else {
    // The edge is not in the queue;
    auto elem = forwardValidQueue_.insert(edge);
    elem->data.getParent()->addToValidQueueOutgoingLookupFromStart(elem);
    elem->data.getChild()->addToValidQueueIncomingLookupFromStart(elem);
    // Increment the counter if the target is inconsistent;
    if (!edge.getChild()->isConsistentInReverseLazySearch() ||
        !optObjPtr_->isFinite(edge.getChild()->heuristicCost_g_ReverseLazy_)) {
      ++numInconsistentOrUnconnectedTargetsInReverse_;
    }
  }
}

void BiAIT::insertOrUpdateForwardValidQueue(const std::vector<Edge> &edges) {
  for (const auto &elem : edges) {
    insertOrUpdateForwardValidQueue(elem);
  }
}

void BiAIT::insertOrUpdateReverseValidQueue(const Edge &edge) {
  // If the edge is an meetLazyEdge or the parent and child lies in different
  // lazy trees; Currently we omit this situation and leave it for the
  // meetValidEdge to handle;
  if (findInMeetLazyQueue(edge) || edge.getChild()->getCategory()[3] ||
      edge.getChild()->getCategory()[4]) {
    return;
  }
  if ((edge.getParent()->getCategory() | edge.getChild()->getCategory())[0] &&
      (edge.getParent()->getCategory() | edge.getChild()->getCategory())[3]) {
    return;
  }
  // If the edge is included in the forwardValidQueue, we do nothing and return;
  // TODO: May this be the chance for trees meet;
  const auto iter_ForwardQueue = std::find_if(
      edge.getParent()->validQueueIncomingLookupFromStart_.begin(),
      edge.getParent()->validQueueIncomingLookupFromStart_.end(),
      [&edge](const auto elem) {
        return elem->data.getParent()->getId() == edge.getChild()->getId();
      });
  if (iter_ForwardQueue !=
      edge.getParent()->validQueueIncomingLookupFromStart_.end())
    return;

  // Check if the edge is already in the queue and can be updated;
  const auto iter_ReverseQueue = std::find_if(
      edge.getChild()->validQueueIncomingLookupFromGoal_.begin(),
      edge.getChild()->validQueueIncomingLookupFromGoal_.end(),
      [&edge](const auto elem) {
        return elem->data.getParent()->getId() == edge.getParent()->getId();
      });
  if (iter_ReverseQueue !=
      edge.getChild()->validQueueIncomingLookupFromGoal_.end()) {
    // Use the incoming lookup of the child;
    // Assert it is already in the outgoing lookup in of the parent;
    assert(std::find_if(
               edge.getParent()->validQueueOutgoingLookupFromGoal_.begin(),
               edge.getParent()->validQueueOutgoingLookupFromGoal_.end(),
               [&edge](const auto elem) {
                 return elem->data.getChild()->getId() ==
                        edge.getChild()->getId();
               }) != edge.getParent()->validQueueOutgoingLookupFromGoal_.end());
    // This edge exists in the queue, if the new sortKey is better than the old,
    // update;
    if (isEdgeBetter(edge, (*iter_ReverseQueue)->data)) {
      (*iter_ReverseQueue)->data.setEdgeKey(edge.getEdgeKey());
      reverseValidQueue_.update(*iter_ReverseQueue);
    }
  } else {
    // The edge is not in the queue;
    auto elem = reverseValidQueue_.insert(edge);
    elem->data.getParent()->addToValidQueueOutgoingLookupFromGoal(elem);
    elem->data.getChild()->addToValidQueueIncomingLookupFromGoal(elem);
    // Increment the counter if the target is inconsistent;
    if (!edge.getChild()->isConsistentInForwardLazySearch() ||
        !optObjPtr_->isFinite(edge.getChild()->heuristicCost_g_ForwardLazy_)) {
      ++numInconsistentOrUnconnectedTargetsInForward_;
    }
  }
}

void BiAIT::insertOrUpdateReverseValidQueue(const std::vector<Edge> &edges) {
  for (const auto &elem : edges) {
    insertOrUpdateReverseValidQueue(elem);
  }
}

void BiAIT::insertOrUpdateForwardLazyQueue(
    const std::shared_ptr<Vertex> &vertex) {
  // Get the pointer to the element in the queue;
  if (vertex->forwardLazyQueuePointer_ != nullptr) {
    // Update it as it is already in the queue;
    vertex->forwardLazyQueuePointer_->data.first =
        computeForwardVertexKey(vertex);
    forwardLazyQueue_.update(vertex->forwardLazyQueuePointer_);
  } else {
    // Insert it to the queue;
    const auto forwardVertexKey = computeForwardVertexKey(vertex);
    // This condition is for early truncation;
    std::pair<std::array<base::Cost, 2u>, std::shared_ptr<Vertex>> elemPair(
        forwardVertexKey, vertex);
    vertex->setForwardLazyQueuePointer(forwardLazyQueue_.insert(elemPair));
    vertex->setCategory(3, true);
  }
}

void BiAIT::insertOrUpdateReverseLazyQueue(
    const std::shared_ptr<Vertex> &vertex) {
  // Get the pointer to the element in the queue;
  if (vertex->reverseLazyQueuePointer_) {
    // Update it as it is already in the queue;
    vertex->reverseLazyQueuePointer_->data.first =
        computeReverseVertexKey(vertex);
    reverseLazyQueue_.update(vertex->reverseLazyQueuePointer_);
  } else {
    // Insert it to the queue;
    const auto reverseVertexKey = computeReverseVertexKey(vertex);
    // This condition is for early truncation;
    std::pair<std::array<base::Cost, 2u>, std::shared_ptr<Vertex>> elemPair(
        reverseVertexKey, vertex);
    vertex->setReverseLazyQueuePointer(reverseLazyQueue_.insert(elemPair));
    vertex->setCategory(0, true);
  }
}

void BiAIT::insertOrUpdateMeetLazyQueue(const std::weak_ptr<Vertex> &parent,
                                        const std::weak_ptr<Vertex> &child) {
  // Maybe not efficiency but this is the cheap-est way to implement;
  WeakEdge edge{parent, child, computeMeetLazyKey(parent.lock(), child.lock())};

  if (auto queueElemPointer = findInMeetLazyQueue(edge)) {
    if (isMeetLazyEdgeBetter(edge, queueElemPointer->data)) {
      meetLazyEdgeQueue_.update(queueElemPointer);
      checkMeetLazyEdgeToValid(queueElemPointer);
    }
  } else {
    auto pointerToNewElem = meetLazyEdgeQueue_.insert(edge);
    // Using the push_back instead of the emplace_back since the
    // pointerToNewElem will be emplaced multiple times;
    parent.lock()->meetLazyEdgeQueuePointer_.push_back(pointerToNewElem);
    child.lock()->meetLazyEdgeQueuePointer_.push_back(pointerToNewElem);
    checkMeetLazyEdgeToValid(pointerToNewElem);
  }
}

bool BiAIT::iterate(
    const base::PlannerTerminationCondition &terminationCondition) {
  ++numIterations_;
  bool performFL{performForwardLazySearch()};
  bool performFV{performForwardValidSearch()};
  bool performRL{performReverseLazySearch()};
  bool performRV{performReverseValidSearch()};
  if (performFL || performFV || performRL || performRV) {
    if ((performFL && !performFV) || (performRL && !performRV)) {
      if (performFL && numIterations_ % 2 == 1) {
        iterateForwardLazySearch();
        ++numFLIteration;
      } else if (performRL && numIterations_ % 2 == 0) {
        iterateReverseLazySearch();
        ++numRLIteration;
      }
    } else if (performFV || performRV) {
      if (performFV && numIterations_ % 2 == 1) {
        iterateForwardValidSearch();
        ++numFVIteration;
      } else if (performRV && numIterations_ % 2 == 0) {
        iterateReverseValidSearch();
        ++numRVIteration;
      }
    }
  } else {
    // Sample a new batch of samples;
    if (implicitGraph_.addSamples(batchSize_, terminationCondition)) {
      if (isPruningEnabled_) {
        removeFromMeetValidQueue(implicitGraph_.prune());
      }
      // `pis_` means plannerInputState;
      if (pis_.haveMoreStartStates() || pis_.haveMoreGoalStates()) {
        // Why always stop: see the comments in `updateStartAndGoalStates`;
        implicitGraph_.updateStartAndGoalStates(
            base::plannerAlwaysTerminatingCondition(), &pis_);
      }
      // Clear and re-initialize the queues;
      invalidateAllLazyComponent();
      clearLazyQueue();
      clearValidQueue();
      insertValidTreeIntoLazyQueue();
      expandStartVerticesIntoForwardValidQueue();
      expandGoalVerticesIntoReverseValidQueue();

    } else {
      // Wrong sample added;
      // May caused by the memory overflow;
      OMPL_ERROR("Wrong Samples added, stop iteration");
      return false;
    }
  }
  return true;
}

base::PlannerStatus::StatusType BiAIT::updateSolution() {
  updateExactSolution();
  if (optObjPtr_->isFinite(solutionCost_)) {
    return base::PlannerStatus::StatusType::EXACT_SOLUTION;
  } else {
    return base::PlannerStatus::StatusType::TIMEOUT;
  }
}

void BiAIT::updateExactSolution() {
  if (meetValidEdgeQueue_.empty()) return;
  // Check if the bestMeetEdge key is less than the current solution cost;
  if (optObjPtr_->isCostBetterThan(
          meetValidEdgeQueue_.top()->data.second.getMeetValidEdgeKey(),
          solutionCost_)) {
    auto pathToGoal = getPathToGoal();
    if (pathToGoal != nullptr) {
      base::PlannerSolution solution(pathToGoal);
      solutionCost_ =
          meetValidEdgeQueue_.top()->data.second.getMeetValidEdgeKey();
      solution.setPlannerName(name_);
      solution.setOptimized(optObjPtr_, solutionCost_,
                            optObjPtr_->isSatisfied(solutionCost_));
      pdef_->addSolutionPath(solution);
      informNewSolution();
    } else {
      return;
    }
  }
}

void BiAIT::informAboutPlannerStatus(
    base::PlannerStatus::StatusType status) const {
  switch (status) {
    case base::PlannerStatus::StatusType::EXACT_SOLUTION: {
      OMPL_INFORM("%s (%u iterations): Found an exact solution of cost %.4f.",
                  name_.c_str(), numIterations_, solutionCost_.value());
      break;
    }
    case base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION: {
      OMPL_ERROR(
          "%s (%u iterations): APPROXIMATE_SOLUTION is disabled. Solution not "
          "found.",
          name_.c_str(), numIterations_);
      break;
    }
    case base::PlannerStatus::StatusType::TIMEOUT: {
      OMPL_INFORM("%s (%u iterations): Solution not found.", name_.c_str(),
                  numIterations_);
      break;
    }
    case base::PlannerStatus::StatusType::UNKNOWN:
    case base::PlannerStatus::StatusType::INVALID_START:
    case base::PlannerStatus::StatusType::INVALID_GOAL:
    case base::PlannerStatus::StatusType::UNRECOGNIZED_GOAL_TYPE:
    case base::PlannerStatus::StatusType::CRASH:
    case base::PlannerStatus::StatusType::ABORT:
    case base::PlannerStatus::StatusType::TYPE_COUNT: {
      OMPL_INFORM(
          "%s (%u iterations): Unable to solve the given planning problem. The "
          "PlannerStatus::StatusType is"
          "UNKNOWN, INVALID_START, INVALID_GOAL, UNRECOGNIZED_GOAL_TYPE, "
          "CRASH, ABORT, or TYPE_COUNT",
          name_.c_str(), numIterations_);
    }
  }
  informRunTimeStatus();
}

void BiAIT::informRunTimeStatus() const {
  OMPL_INFORM(
      "Run Time Info: \t[ %s ]\t Iteration: [ %zu ]; \t BatchId: [ %zu ]; \t "
      "Sampled states [ %u ]; \t Valid samples: [ %zu ] (%.1f); \t "
      "MeetLazyEdgeQueue.size(): [ %zu ]\n"
      "\t \tProcessed edges: [ %u ]; Collision free edges: [ %u ] (%.1f); "
      "\t \tFV, FL, RV, RL branch sizes: [ %zu ] [ %zu ] [ %zu ] [ %zu ]; "
      "\t \tFV, FL, RV, RL queue sizes: [ %zu ] [ %zu ] [ %zu ] [ %zu ]; "
      "\n \n",
      name_.c_str(), numIterations_, implicitGraph_.getBatchId(),
      implicitGraph_.getNumberOfSampledStates(),
      implicitGraph_.getNumberOfValidSamples(),
      implicitGraph_.getNumberOfSampledStates() == 0u
          ? 0.0
          : 100.0 *
                (static_cast<double>(implicitGraph_.getNumberOfValidSamples()) /
                 static_cast<double>(
                     implicitGraph_.getNumberOfSampledStates())),
      meetLazyEdgeQueue_.size(), numProcessedEdges_, numEdgeCollisionChecks_,
      numProcessedEdges_ == 0u
          ? 0.0
          : 100.0 * (static_cast<float>(numEdgeCollisionChecks_) /
                     static_cast<float>(numProcessedEdges_)),
      countNumVerticesInForwardValidPortion(),
      countNumVerticesInForwardLazyPortion(),
      countNumVerticesInReverseValidPortion(),
      countNumVerticesInReverseLazyPortion(), forwardValidQueue_.size(),
      forwardLazyQueue_.size(), reverseValidQueue_.size(),
      reverseLazyQueue_.size());
}

void BiAIT::informNewSolution() const {
  OMPL_INFORM(
      "%s (%u iterations): Found a new exact solution of cost %.4f. Sampled a "
      "total of %u states, %u "
      "of which were valid samples (%.1f \%). Processed %u edges, %u of which "
      "were collision checked "
      "(%.1f \%). "
      "The [forward valid/forward lazy/reverse valid/reverse lazy] portion "
      "contains [%u/%u/%u/%u] vertices. "
      "The [meet queue] have [%u] edges. The [start/goal] set have [%u/%u] "
      "vertices.",
      name_.c_str(), numIterations_, solutionCost_.value(),
      implicitGraph_.getNumberOfSampledStates(),
      implicitGraph_.getNumberOfValidSamples(),
      implicitGraph_.getNumberOfSampledStates() == 0u
          ? 0.0
          : 100.0 *
                (static_cast<double>(implicitGraph_.getNumberOfValidSamples()) /
                 static_cast<double>(
                     implicitGraph_.getNumberOfSampledStates())),
      numProcessedEdges_, numEdgeCollisionChecks_,
      numProcessedEdges_ == 0u
          ? 0.0
          : 100.0 * (static_cast<float>(numEdgeCollisionChecks_) /
                     static_cast<float>(numProcessedEdges_)),
      countNumVerticesInForwardValidPortion(),
      countNumVerticesInForwardLazyPortion(),
      countNumVerticesInReverseValidPortion(),
      countNumVerticesInReverseLazyPortion(), meetValidEdgeQueue_.size(),
      implicitGraph_.getStartVertices().size(),
      implicitGraph_.getGoalVertices().size());
}

std::size_t BiAIT::countNumVerticesInForwardValidPortion() const {
  std::size_t outputNumber{0u};
  auto vertices = implicitGraph_.getVertices();
  for (const auto &elem : vertices) {
    if (implicitGraph_.isStart(elem) || elem->getCategory()[4]) {
      ++outputNumber;
    }
  }
  return outputNumber;
}

std::size_t BiAIT::countNumVerticesInForwardLazyPortion() const {
  std::size_t outputNumber{0u};
  auto vertices = implicitGraph_.getVertices();
  for (const auto &elem : vertices) {
    if (elem->getCategory()[3]) {
      ++outputNumber;
    }
  }
  return outputNumber;
}

std::size_t BiAIT::countNumVerticesInReverseValidPortion() const {
  std::size_t outputNumber{0u};
  auto vertices = implicitGraph_.getVertices();
  for (const auto &elem : vertices) {
    if (implicitGraph_.isGoal(elem) || elem->getCategory()[1]) {
      ++outputNumber;
    }
  }
  return outputNumber;
}

std::size_t BiAIT::countNumVerticesInReverseLazyPortion() const {
  std::size_t outputNumber{0u};
  auto vertices = implicitGraph_.getVertices();
  for (const auto &elem : vertices) {
    //            assert(!implicitGraph_.isGoal(elem) &&
    //            elem->getCategory().count() == 1);
    if (elem->getCategory()[0]) {
      ++outputNumber;
    }
  }
  return outputNumber;
}

std::shared_ptr<geometric::PathGeometric> BiAIT::getPathToGoal() const {
  auto bestMeetEdge = meetValidEdgeQueue_.top();
  if (bestMeetEdge->data.second.getParent()->getForwardValidParent() ==
          nullptr ||
      bestMeetEdge->data.second.getChild()->getReverseValidParent() ==
          nullptr) {
    Utility::removeFromVectorByValue(
        bestMeetEdge->data.second.getParent()->meetValidEdgeQueuePointer_,
        meetValidEdgeQueue_.top());
    Utility::removeFromVectorByValue(
        bestMeetEdge->data.second.getChild()->meetValidEdgeQueuePointer_,
        meetValidEdgeQueue_.top());
    meetValidEdgeQueue_.pop();
    return nullptr;
  }
  if (static_cast<bool>(bestMeetEdge)) {
    std::shared_ptr<geometric::PathGeometric> path =
        std::make_shared<geometric::PathGeometric>(spaceInformationPtr_);
    assert(bestMeetEdge->data.second
               .getCategory()[2]);  // Its category should be 0b00100;
    // Query to start;
    std::vector<std::shared_ptr<Vertex>>
        invertedPathToStart{};  // Retrieve the vertices vector and store
                                // reversely;
    auto currentVertexToStart = bestMeetEdge->data.second.getParent();
    do {
      invertedPathToStart.emplace_back(currentVertexToStart);
      currentVertexToStart = currentVertexToStart->getForwardValidParent();
    } while (!implicitGraph_.isStart(currentVertexToStart));
    // Emplace the reverse vector to the path with the reverse iterator;
    path->append(implicitGraph_.getStartVertices().at(0)->getState());
    for (auto crIter = invertedPathToStart.crbegin();
         crIter != invertedPathToStart.crend(); ++crIter) {
      path->append((*crIter)->getState());  // (*crIter) is shared_ptr;
    }

    // Query to goal;
    auto currentVertexToGoal = bestMeetEdge->data.second.getChild();
    do {
      path->append(currentVertexToGoal->getState());
      currentVertexToGoal = currentVertexToGoal->getReverseValidParent();
    } while (!implicitGraph_.isGoal(currentVertexToGoal));
    path->append(implicitGraph_.getGoalVertices().at(0)->getState());
    return path;
  } else {
    return nullptr;
  }
}

bool BiAIT::isEdgeBetter(const Edge &lhs, const Edge &rhs) const {
  return std::lexicographical_compare(
      lhs.getEdgeKey().cbegin(), lhs.getEdgeKey().cend(),
      rhs.getEdgeKey().cbegin(), rhs.getEdgeKey().cend(),
      [this](const auto &edge1, const auto &edge2) {
        return optObjPtr_->isCostBetterThan(edge1, edge2);
      });
}

bool BiAIT::isVertexBetter(const KeyVertexPair &lhs,
                           const KeyVertexPair &rhs) const {
  // Vertex queuing method is directly transferred from the one in AIT_star, but
  // modified to root; If the costs of two vertices are equal then we prioritize
  // inconsistent vertices that are targets of edges in the forward queue;
  if (optObjPtr_->isCostEquivalentTo(lhs.first[0u], rhs.first[0u]) &&
      optObjPtr_->isCostEquivalentTo(lhs.first[1u], rhs.first[1u])) {
    const std::vector<ompl::BinaryHeap<
        ompl::geometric::biait::Edge,
        std::function<bool(const Edge &, const Edge &)>>::Element *>
        *queueTypePtr{nullptr};
    bool isConsistent{false};
    if (lhs.second->getCategory().to_ulong() > 7) {
      queueTypePtr = &lhs.second->validQueueIncomingLookupFromStart_;
      isConsistent = lhs.second->isConsistentInForwardLazySearch();
    } else {
      queueTypePtr = &lhs.second->validQueueIncomingLookupFromGoal_;
      isConsistent = lhs.second->isConsistentInReverseLazySearch();
    }
    return !queueTypePtr->empty() && !isConsistent;
  } else {
    // Otherwise it's a regular lexicographical comparison of the keys.
    return std::lexicographical_compare(
        lhs.first.cbegin(), lhs.first.cend(), rhs.first.cbegin(),
        rhs.first.cend(), [this](const auto &a, const auto &b) {
          return optObjPtr_->isCostBetterThan(a, b);
        });
  }
}

base::Cost BiAIT::computeBestCostHeuristic(
    const std::shared_ptr<Vertex> &fromVertex,
    const std::vector<std::shared_ptr<Vertex>> &toVectorOfVertex) const {
  base::Cost outputCost = optObjPtr_->infiniteCost();
  for (const auto &elem : toVectorOfVertex) {
    outputCost = optObjPtr_->betterCost(
        outputCost, optObjPtr_->motionCostHeuristic(fromVertex->getState(),
                                                    elem->getState()));
  }
  return outputCost;
}

base::Cost BiAIT::computeBestCostHeuristic(
    const std::vector<std::shared_ptr<Vertex>> &fromVectorOfVertex,
    const std::shared_ptr<Vertex> &toVertex) const {
  base::Cost outputCost = optObjPtr_->infiniteCost();
  for (const auto &elem : fromVectorOfVertex) {
    outputCost = optObjPtr_->betterCost(
        outputCost, optObjPtr_->motionCostHeuristic(elem->getState(),
                                                    toVertex->getState()));
  }
  return outputCost;
}

void BiAIT::updateCostToStartOfForwardValidDescendant(
    const std::shared_ptr<Vertex> &vertex) const {
  for (const auto &elem : vertex->getForwardValidChildren()) {
    elem->costToStart_ = optObjPtr_->combineCosts(vertex->costToStart_,
                                                  elem->edgeCostFromStart_);
    if (elem->getCategory()[2]) {
      insertOrUpdateMeetValidEdge(elem);
    }
    updateCostToStartOfForwardValidDescendant(elem);
  }
}

void BiAIT::updateCostToGoalOfReverseValidDescendant(
    const std::shared_ptr<Vertex> &vertex) const {
  for (const auto &elem : vertex->getReverseValidChildren()) {
    elem->costToGoal_ =
        optObjPtr_->combineCosts(vertex->costToGoal_, elem->edgeCostFromGoal_);
    if (elem->getCategory()[2]) {
      insertOrUpdateMeetValidEdge(elem);
    }
    updateCostToGoalOfReverseValidDescendant(elem);
  }
}

void BiAIT::updateCostToStartOfRVPredecessor(
    const std::shared_ptr<Vertex> &vertex) const {
  if (vertex->hasReverseValidParent()) {
    auto RVParent = vertex->getReverseValidParent();
    if (optObjPtr_->isCostBetterThan(
            optObjPtr_->combineCosts(vertex->costToStart_,
                                     vertex->edgeCostFromGoal_),
            RVParent->costToStart_)) {
      RVParent->costToStart_ = optObjPtr_->combineCosts(
          vertex->costToStart_, vertex->edgeCostFromGoal_);
      updateCostToStartOfRVPredecessor(RVParent);
    }
  }
}

void BiAIT::insertOrUpdateMeetValidEdge(Edge &selectedValidEdge,
                                        const base::Cost &edgeCost) {
  // Forcing the front-end pass the edgeCost, because calculating the edgeCost
  // is costly;
  selectedValidEdge.setMeetValidEdgeKey(
      computeMeetValidKey(selectedValidEdge, edgeCost));
  if (auto queuePointer = findInMeetValidQueue(selectedValidEdge)) {
    // Update meet valid queue;
    if (optObjPtr_->isCostBetterThan(
            selectedValidEdge.getMeetValidEdgeKey(),
            queuePointer->data.second.getMeetValidEdgeKey())) {
      queuePointer->data.first = edgeCost;
      meetValidEdgeQueue_.update(queuePointer);
    }
  } else {
    // Insert it to the meet valid queue;
    // Using the push_back instead of the emplace_back since the
    // pointerToNewElem will be emplaced multiple times;
    auto pointerToNewElem =
        meetValidEdgeQueue_.insert(std::make_pair(edgeCost, selectedValidEdge));
    pointerToNewElem->data.first = edgeCost;
    pointerToNewElem->data.second.getParent()
        ->meetValidEdgeQueuePointer_.push_back(pointerToNewElem);
    pointerToNewElem->data.second.getChild()
        ->meetValidEdgeQueuePointer_.push_back(pointerToNewElem);
    pointerToNewElem->data.second.getParent()->setCategory(2, true);
    pointerToNewElem->data.second.getChild()->setCategory(2, true);
  }
  // Check whether the solution is improved;
  updateExactSolution();
}

void BiAIT::insertOrUpdateMeetValidEdge(
    const std::shared_ptr<Vertex> &vertex) const {
  // Find the `vertex` related meetValidEdge(s);
  for (const auto &elem : vertex->meetValidEdgeQueuePointer_) {
    if (elem == nullptr) continue;
    assert(vertex->getId() == elem->data.second.getParent()->getId() ||
           vertex->getId() == elem->data.second.getChild()->getId());
    base::Cost newMeetValidKey =
        computeMeetValidKey(elem->data.second, elem->data.first);
    if (optObjPtr_->isCostBetterThan(newMeetValidKey,
                                     elem->data.second.getMeetValidEdgeKey())) {
      // Update the related meetValidEdge(s);
      elem->data.second.setMeetValidEdgeKey(newMeetValidKey);
      meetValidEdgeQueue_.update(elem);
      // Propagate the updated information to start and goal precedents,
      // respectively;
      elem->data.second.getParent()->costToGoal_ = optObjPtr_->combineCosts(
          elem->data.second.getChild()->costToGoal_, elem->data.first);
      elem->data.second.getChild()->costToStart_ = optObjPtr_->combineCosts(
          elem->data.second.getParent()->costToStart_, elem->data.first);
      updateCostToStartOfReverseValidPrecedent(elem->data.second.getChild());
      updateCostToGoalOfForwardValidPrecedent(elem->data.second.getParent());
    }
  }
}

void BiAIT::insertOrUpdateMeetValidEdge(const std::shared_ptr<Vertex> &parent,
                                        const std::shared_ptr<Vertex> &child) {
  if (parent->hasWhitelistedChild(child) ||
      motionValidatorPtr_->checkMotion(parent->getState(), child->getState())) {
    // Collision free!
    if (!parent->hasWhitelistedChild(child)) {
      parent->whitelistAsChild(child);
      ++numEdgeCollisionChecks_;
    }
    Edge meetValidEdge(parent, child, 0b00100,
                       {{optObjPtr_->infiniteCost(), optObjPtr_->infiniteCost(),
                         optObjPtr_->infiniteCost()}});
    const auto edgeCost =
        optObjPtr_->motionCost(parent->getState(), child->getState());
    insertOrUpdateMeetValidEdge(meetValidEdge, edgeCost);
  }
}

void BiAIT::updateCostToStartOfReverseValidPrecedent(
    const std::shared_ptr<Vertex> &vertex) const {
  if (vertex->hasReverseValidParent()) {
    if (optObjPtr_->isCostBetterThan(
            optObjPtr_->combineCosts(vertex->costToStart_,
                                     vertex->edgeCostFromGoal_),
            vertex->getReverseValidParent()->costToStart_)) {
      vertex->getReverseValidParent()->costToStart_ = optObjPtr_->combineCosts(
          vertex->costToStart_, vertex->edgeCostFromGoal_);
      updateCostToStartOfReverseValidPrecedent(vertex->getReverseValidParent());
    }
  }
}

void BiAIT::updateCostToGoalOfForwardValidPrecedent(
    const std::shared_ptr<Vertex> &vertex) const {
  if (vertex->hasForwardValidParent()) {
    if (optObjPtr_->isCostBetterThan(
            optObjPtr_->combineCosts(vertex->costToGoal_,
                                     vertex->edgeCostFromStart_),
            vertex->getForwardValidParent()->costToGoal_)) {
      vertex->getForwardValidParent()->costToGoal_ = optObjPtr_->combineCosts(
          vertex->costToGoal_, vertex->edgeCostFromStart_);
      updateCostToGoalOfForwardValidPrecedent(vertex->getForwardValidParent());
    }
  }
}

/* ##########  ##########  ##########  ##########  ########## */
// Functions called in function `iterate()`;
/* ##########  ##########  ##########  ##########  ########## */

bool BiAIT::performForwardLazySearch() {
  if (forwardLazyQueue_.empty() || forwardValidQueue_.empty() ||
      reverseValidQueue_.empty())
    return false;
  const auto &bestEdge = optObjPtr_->isCostBetterThan(
                             forwardValidQueue_.top()->data.getEdgeKey()[0u],
                             reverseValidQueue_.top()->data.getEdgeKey()[0u])
                             ? forwardValidQueue_.top()->data
                             : reverseValidQueue_.top()->data;
  const auto &bestVertex = forwardLazyQueue_.top()->data;
  return !((bestEdge.getChild()->isConsistentInForwardLazySearch() &&
            optObjPtr_->isCostBetterThan(bestEdge.getEdgeKey()[0u],
                                         bestVertex.first[0u])) ||
           numInconsistentOrUnconnectedTargetsInForward_ == 0u);
}

bool BiAIT::performReverseLazySearch() {
  if (reverseLazyQueue_.empty() || forwardValidQueue_.empty() ||
      reverseValidQueue_.empty())
    return false;
  const auto &bestEdge = optObjPtr_->isCostBetterThan(
                             forwardValidQueue_.top()->data.getEdgeKey()[0u],
                             reverseValidQueue_.top()->data.getEdgeKey()[0u])
                             ? forwardValidQueue_.top()->data
                             : reverseValidQueue_.top()->data;
  const auto &bestVertex = reverseLazyQueue_.top()->data;
  return !((bestEdge.getChild()->isConsistentInReverseLazySearch() &&
            optObjPtr_->isCostBetterThan(bestEdge.getEdgeKey()[0u],
                                         bestVertex.first[0u])) ||
           numInconsistentOrUnconnectedTargetsInReverse_ == 0u);
}

bool BiAIT::performForwardValidSearch() {
  if (forwardValidQueue_.empty()) return false;
  // If the best edge in the forward valid queue has a potential total solution
  // cost of infinity, the forward valid search does not need to be continued.
  // This can happen if the lazy search did not reach each other;
  base::Cost bestEdgeCost = optObjPtr_->infiniteCost();
  bestEdgeCost = forwardValidQueue_.top()->data.getEdgeKey()[0u];
  if (!optObjPtr_->isFinite(bestEdgeCost)) return false;
  return optObjPtr_->isCostBetterThan(bestEdgeCost, solutionCost_);
}

bool BiAIT::performReverseValidSearch() {
  if (reverseValidQueue_.empty()) return false;
  // If the best edge in the reverse valid queue has a potential total solution
  // cost of infinity, the reverse valid search does not need to be continued.
  // This can happen if the lazy search did not reach each other;
  base::Cost bestEdgeCost = optObjPtr_->infiniteCost();
  bestEdgeCost = reverseValidQueue_.top()->data.getEdgeKey()[0u];
  if (!optObjPtr_->isFinite(bestEdgeCost)) return false;
  return optObjPtr_->isCostBetterThan(bestEdgeCost, solutionCost_);
}

void BiAIT::iterateForwardLazySearch() {
  assert(!forwardLazyQueue_.empty());
  // Retrieve the most promising vertex and remove it from the queue;
  auto vertex = forwardLazyQueue_.top()->data.second;
  forwardLazyQueue_.pop();
  vertex->resetForwardLazyQueuePointer();

  if (vertex->getCategory()[1] || vertex->getCategory()[0]) {
    return;
  }
  // If this vertex is also contained in the reverseValidQueue_, eliminate this
  // vertex in RVQueue;
  if (!vertex->validQueueIncomingLookupFromGoal_.empty()) {
    for (const auto &elem : vertex->validQueueIncomingLookupFromGoal_) {
      elem->data.getParent()->removeFromValidQueueOutgoingLookupFromGoal(elem);
      reverseValidQueue_.remove(elem);
    }
    vertex->validQueueIncomingLookupFromGoal_.clear();
  }

  if (vertex->isConsistentInForwardLazySearch()) return;
  // Check if the vertex is under consistent (rhs[s] < g[s]);
  if (optObjPtr_->isCostBetterThan(vertex->heuristicCost_rhs_ForwardLazy_,
                                   vertex->heuristicCost_g_ForwardLazy_) ||
      optObjPtr_->isCostEquivalentTo(vertex->heuristicCost_rhs_ForwardLazy_,
                                     vertex->heuristicCost_g_ForwardLazy_)) {
    // Make it consistent and update the vertex;
    vertex->heuristicCost_g_ForwardLazy_ =
        vertex->heuristicCost_rhs_ForwardLazy_;
    // Update the number of inconsistent targets in the reverse valid queue;
    numInconsistentOrUnconnectedTargetsInForward_ -=
        vertex->validQueueIncomingLookupFromGoal_.size();
  } else {
    // Make the vertex over-consistent;
    vertex->heuristicCost_g_ForwardLazy_ = optObjPtr_->infiniteCost();
    // Update the vertex and its neighbors;
    updateVertexInFLSearch(vertex);
  }
  updateForwardLazyNeighbors(vertex);
}

void BiAIT::iterateReverseLazySearch() {
  assert(!reverseLazyQueue_.empty());

  // Retrieve the most promising vertex  and remove it from the queue;
  auto vertex = reverseLazyQueue_.top()->data.second;
  reverseLazyQueue_.pop();
  vertex->resetReverseLazyQueuePointer();

  if (vertex->getCategory()[4] || vertex->getCategory()[3]) {
    return;
  }
  // If this vertex is also contained in the forwardValidQueue_, eliminate this
  // vertex in FVQueue;
  if (!vertex->validQueueIncomingLookupFromStart_.empty()) {
    for (const auto &elem : vertex->validQueueIncomingLookupFromStart_) {
      elem->data.getParent()->removeFromValidQueueOutgoingLookupFromStart(elem);
      forwardValidQueue_.remove(elem);
    }
    vertex->validQueueIncomingLookupFromStart_.clear();
  }

  if (vertex->isConsistentInReverseLazySearch()) return;
  // Check if the vertex is under consistent (g[s] < v[s])
  if (optObjPtr_->isCostBetterThan(vertex->heuristicCost_rhs_ReverseLazy_,
                                   vertex->heuristicCost_g_ReverseLazy_) ||
      optObjPtr_->isCostEquivalentTo(vertex->heuristicCost_rhs_ReverseLazy_,
                                     vertex->heuristicCost_g_ReverseLazy_)) {
    // Make it consistent and update the vertex;
    vertex->heuristicCost_g_ReverseLazy_ =
        vertex->heuristicCost_rhs_ReverseLazy_;
    // Update the number of inconsistent targets in the reverse valid queue;
    numInconsistentOrUnconnectedTargetsInReverse_ -=
        vertex->validQueueIncomingLookupFromStart_.size();
  } else {
    // Make the vertex over-consistent;
    vertex->heuristicCost_g_ReverseLazy_ = optObjPtr_->infiniteCost();
    // Update the vertex and its neighbors;
    updateVertexInRLSearch(vertex);
  }
  updateReverseLazyNeighbors(vertex);
}

void BiAIT::iterateForwardValidSearch() {
  assert(!forwardValidQueue_.empty());
  // Retrieve the most promising edge;
  auto parent = forwardValidQueue_.top()->data.getParent();
  auto child = forwardValidQueue_.top()->data.getChild();
  child->removeFromValidQueueIncomingLookupFromStart(forwardValidQueue_.top());
  parent->removeFromValidQueueOutgoingLookupFromStart(forwardValidQueue_.top());
  forwardValidQueue_.pop();

  if (!(child->isConsistentInReverseLazySearch() &&
        !implicitGraph_.isGoal(parent) && !child->getCategory()[1] &&
        !child->getCategory()[0])) {
    return;
  }
  ++numProcessedEdges_;

  // If this edge is already in the forward valid tree, it is a free-by:
  if (child->hasForwardValidParent() &&
      child->getForwardValidParent()->getId() == parent->getId()) {
    insertOrUpdateForwardValidQueue(getOutgoingEdges(child));
    return;
  } else if (optObjPtr_->isCostBetterThan(
                 optObjPtr_->combineCosts(
                     parent->costToStart_,
                     optObjPtr_->motionCostHeuristic(parent->getState(),
                                                     child->getState())),
                 child->costToStart_)) {
    // Check if the edge can possibly improve the current solution;
    // If the edge can, then perform collision checking here;
    if (parent->hasWhitelistedChild(child) ||
        motionValidatorPtr_->checkMotion(parent->getState(),
                                         child->getState())) {
      // Collision free!
      if (!parent->hasWhitelistedChild(child)) {
        parent->whitelistAsChild(child);
        ++numEdgeCollisionChecks_;
      }
      const auto edgeCost =
          optObjPtr_->motionCost(parent->getState(), child->getState());
      // Check if the edge can improve the cost-to-come of the child;
      if (optObjPtr_->isCostBetterThan(
              optObjPtr_->combineCosts(parent->costToStart_, edgeCost),
              child->costToStart_)) {
        child->setForwardValidParent(parent, edgeCost);  // Rewire;
        parent->addToForwardValidChildren(child);
        child->setCategory(4, true);
        updateCostToStartOfForwardValidDescendant(child);
        updateSolution();
        insertOrUpdateForwardValidQueue(getOutgoingEdges(child));

        if (!child->meetLazyEdgeQueuePointer_
                 .empty()) {  // Child must be the parent of the meetLazyEdge;
          auto tempVector = child->meetLazyEdgeQueuePointer_;
          for (const auto &elem : tempVector) {
            checkMeetLazyEdgeToValid(elem);
          }
        }
        for (const auto &elem : implicitGraph_.getNeighbors(child)) {
          // Avoid meaningless checking;
          if (!elem->getCategory()[1]) continue;
          if (child->hasBlacklistedChild(elem) ||
              elem->hasBlacklistedChild(child))
            continue;
          insertOrUpdateMeetValidEdge(child, elem);
        }
      }
    } else {
      // In collision;
      parent->blacklistAsChild(child);
      child->blacklistAsChild(parent);
      // Repair both the forward lazy search and the reverse lazy search;
      if (child->hasForwardLazyParent() &&
          child->getForwardLazyParent()->getId() == parent->getId()) {
        // Forward lazy branch;
        invalidateForwardLazyBranch(child);
        updateVertexInFLSearch(child);
      }
      // Propagate the change of h_g_RL and h_rhs_RL to start;
      invalidateFLToStartWhenInvalidatingFL(parent, child);
    }
  }
}

void BiAIT::iterateReverseValidSearch() {
  assert(!reverseValidQueue_.empty());
  // Retrieve the most promising edge;
  auto parent = reverseValidQueue_.top()->data.getParent();
  auto child = reverseValidQueue_.top()->data.getChild();
  child->removeFromValidQueueIncomingLookupFromGoal(reverseValidQueue_.top());
  parent->removeFromValidQueueOutgoingLookupFromGoal(reverseValidQueue_.top());
  reverseValidQueue_.pop();

  if (!(child->isConsistentInForwardLazySearch() &&
        !implicitGraph_.isStart(parent) && !child->getCategory()[4] &&
        !child->getCategory()[3])) {
    return;
  }
  ++numProcessedEdges_;

  // If this edge is already in the reverse valid tree, it is a free-by:
  if (child->hasReverseValidParent() &&
      child->getReverseValidParent()->getId() == parent->getId()) {
    insertOrUpdateReverseValidQueue(getOutgoingEdges(child));
    return;
  } else if (optObjPtr_->isCostBetterThan(
                 optObjPtr_->combineCosts(
                     parent->costToGoal_,
                     optObjPtr_->motionCostHeuristic(parent->getState(),
                                                     child->getState())),
                 child->costToGoal_)) {
    // Check if the edge can possibly improve the current solution;
    // If the edge can, then perform collision checking here;
    if (parent->hasWhitelistedChild(child) ||
        motionValidatorPtr_->checkMotion(parent->getState(),
                                         child->getState())) {
      // Collision free!
      if (!parent->hasWhitelistedChild(child)) {
        parent->whitelistAsChild(child);
        ++numEdgeCollisionChecks_;
      }
      const auto edgeCost =
          optObjPtr_->motionCost(parent->getState(), child->getState());
      // Check if the edge can improve the cost-to-go of the child;
      if (optObjPtr_->isCostBetterThan(
              optObjPtr_->combineCosts(parent->costToGoal_, edgeCost),
              child->costToGoal_)) {
        child->setReverseValidParent(parent, edgeCost);  // Rewire;
        parent->addToReverseValidChildren(child);
        child->setCategory(1, true);
        updateCostToGoalOfReverseValidDescendant(child);
        updateSolution();
        insertOrUpdateReverseValidQueue(getOutgoingEdges(child));

        if (!child->meetLazyEdgeQueuePointer_
                 .empty()) {  // Child must be the parent of the meetLazyEdge;
          auto tempVector = child->meetLazyEdgeQueuePointer_;
          for (const auto &elem : tempVector) {
            checkMeetLazyEdgeToValid(elem);
          }
        }
        for (const auto &elem : implicitGraph_.getNeighbors(child)) {
          // Avoid meaningless checking;
          if (!elem->getCategory()[4]) continue;
          if (child->hasBlacklistedChild(elem) ||
              elem->hasBlacklistedChild(child))
            continue;
          insertOrUpdateMeetValidEdge(elem, child);
        }
      }
    } else {
      // In collision;
      parent->blacklistAsChild(child);
      child->blacklistAsChild(parent);
      // Repair both the forward lazy search and the reverse lazy search;
      if (child->hasReverseLazyParent() &&
          child->getReverseLazyParent()->getId() == parent->getId()) {
        // Reverse lazy branch;
        invalidateReverseLazyBranch(child);
        updateVertexInRLSearch(child);
      }
      invalidateRLToGoalWhenInvalidatingRL(parent, child);
    }
  }
}

void BiAIT::clearLazyQueue() {
  clearForwardLazyQueue();
  clearReverseLazyQueue();
}

void BiAIT::clearForwardLazyQueue() {
  std::vector<KeyVertexPair> forwardLazyQueue;
  forwardLazyQueue_.getContent(forwardLazyQueue);
  for (const auto &elem : forwardLazyQueue) {
    elem.second->resetForwardLazyQueuePointer();
  }
  forwardLazyQueue_.clear();
}

void BiAIT::clearReverseLazyQueue() {
  std::vector<KeyVertexPair> reverseLazyQueue;
  reverseLazyQueue_.getContent(reverseLazyQueue);
  for (const auto &elem : reverseLazyQueue) {
    elem.second->resetReverseLazyQueuePointer();
  }
  reverseLazyQueue_.clear();
}

void BiAIT::clearValidQueue() {
  clearForwardValidQueue();
  clearReverseValidQueue();
}

void BiAIT::clearForwardValidQueue() {
  std::vector<Edge> forwardValidQueue;
  forwardValidQueue_.getContent(forwardValidQueue);
  for (const auto &elem : forwardValidQueue) {
    elem.getChild()->validQueueIncomingLookupFromStart_.clear();
    elem.getParent()->validQueueOutgoingLookupFromStart_.clear();
  }
  forwardValidQueue_.clear();
  numInconsistentOrUnconnectedTargetsInForward_ = 0u;
}

void BiAIT::clearReverseValidQueue() {
  std::vector<Edge> reverseValidQueue;
  reverseValidQueue_.getContent(reverseValidQueue);
  for (const auto &elem : reverseValidQueue) {
    elem.getChild()->validQueueIncomingLookupFromGoal_.clear();
    elem.getParent()->validQueueOutgoingLookupFromGoal_.clear();
  }
  reverseValidQueue_.clear();
  numInconsistentOrUnconnectedTargetsInReverse_ = 0u;
}

void BiAIT::clearMeetLazyQueue() {
  std::vector<WeakEdge> meetEdges;
  meetLazyEdgeQueue_.getContent(meetEdges);
  for (const auto &elem : meetEdges) {
    elem.getParent().lock()->meetLazyEdgeQueuePointer_.clear();
    elem.getChild().lock()->meetLazyEdgeQueuePointer_.clear();
  }
  meetLazyEdgeQueue_.clear();
}

void BiAIT::invalidateForwardLazyBranch(const std::shared_ptr<Vertex> &vertex) {
  if (!vertex->meetLazyEdgeQueuePointer_.empty()) {
    for (const auto &meetEdge : vertex->meetLazyEdgeQueuePointer_) {
      if (meetEdge == nullptr || !meetEdge->data.getParent().lock() ||
          !meetEdge->data.getChild().lock()) {
        if (meetEdge != nullptr && meetEdge->data.getChild().lock()) {
          Utility::removeFromVectorByValue(
              meetEdge->data.getChild().lock()->meetLazyEdgeQueuePointer_,
              meetEdge);
        }
        meetLazyEdgeQueue_.remove(meetEdge);
        continue;
      }
      Utility::removeFromVectorByValue(
          meetEdge->data.getChild().lock()->meetLazyEdgeQueuePointer_,
          meetEdge);
      if (optObjPtr_->isCostEquivalentTo(
              meetEdge->data.getChild().lock()->heuristicCost_g_ForwardLazy_,
              optObjPtr_->combineCosts(
                  meetEdge->data.getParent()
                      .lock()
                      ->heuristicCost_g_ForwardLazy_,
                  optObjPtr_->motionCostHeuristic(
                      meetEdge->data.getParent().lock()->getState(),
                      meetEdge->data.getChild().lock()->getState())))) {
        invalidateRLWhenInvalidatingFL(meetEdge->data.getChild().lock(),
                                       nullptr);
      }
      meetLazyEdgeQueue_.remove(meetEdge);
    }
    vertex->meetLazyEdgeQueuePointer_.clear();
  }
  // If this vertex is consistent before invalidation, then all incoming edges
  // now have targets that are inconsistent;
  if (vertex->isConsistentInForwardLazySearch()) {
    numInconsistentOrUnconnectedTargetsInForward_ +=
        vertex->validQueueIncomingLookupFromGoal_.size();
  }
  // Reset the cost to come from the start and the forward lazy children unless
  // the vertex is itself a goal;
  if (!implicitGraph_.isStart(vertex)) {
    // Reset the parent;
    vertex->heuristicCost_rhs_ForwardLazy_ = optObjPtr_->infiniteCost();
  }
  vertex->heuristicCost_g_ForwardLazy_ = optObjPtr_->infiniteCost();
  vertex->heuristicCost_g_ReverseLazy_ = optObjPtr_->infiniteCost();
  vertex->heuristicCost_rhs_ReverseLazy_ = optObjPtr_->infiniteCost();
  // Update all related edges in the forward valid queue;
  for (const auto &elem : vertex->validQueueIncomingLookupFromStart_) {
    elem->data.setEdgeKey(computeForwardEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    forwardValidQueue_.update(elem);
  }
  // Update all related edges in the reverse valid queue;
  for (const auto &elem : vertex->validQueueIncomingLookupFromGoal_) {
    elem->data.setEdgeKey(computeReverseEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    reverseValidQueue_.update(elem);
  }
  // Remove the `vertex` from the forward lazy queue if it is in it;
  if (vertex->forwardLazyQueuePointer_) {
    forwardLazyQueue_.remove(vertex->forwardLazyQueuePointer_);
    vertex->resetForwardLazyQueuePointer();
  }

  if (vertex->hasForwardLazyParent()) {
    vertex->getForwardLazyParent()->removeFromForwardLazyChildren(
        vertex->getId());
    vertex->resetForwardLazyParent();
  }
  if (vertex->getCategory()[4]) vertex->setCategory(3, true);

  // Update the cost of all forward lazy children;
  for (const auto &elem : vertex->getForwardLazyChildren()) {
    invalidateForwardLazyBranch(elem);
  }

  // Update the forward lazy search vertex to ensure that this vertex is placed
  // in V_open if necessary;
  updateValidQueueIfVertexInside(vertex);
}

void BiAIT::invalidateRLWhenInvalidatingFL(
    const std::shared_ptr<Vertex> &vertex,
    const std::shared_ptr<Vertex> &propagateFrom) {
  base::Cost bestHeuristic = optObjPtr_->infiniteCost();
  for (const auto &child : vertex->getReverseLazyChildren()) {
    if (propagateFrom != nullptr && child->getId() == propagateFrom->getId())
      continue;
    base::Cost candidateHeuristic = optObjPtr_->combineCosts(
        child->heuristicCost_g_ForwardLazy_,
        optObjPtr_->motionCostHeuristic(child->getState(), vertex->getState()));
    if (optObjPtr_->isCostBetterThan(candidateHeuristic, bestHeuristic))
      bestHeuristic = candidateHeuristic;
  }

  vertex->heuristicCost_rhs_ForwardLazy_ = bestHeuristic;
  vertex->heuristicCost_g_ForwardLazy_ = bestHeuristic;

  // Update all related edges in the reverse valid queue;
  for (const auto &elem : vertex->validQueueIncomingLookupFromGoal_) {
    elem->data.setEdgeKey(computeReverseEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    reverseValidQueue_.update(elem);
  }

  if (!optObjPtr_->isFinite(vertex->heuristicCost_g_ForwardLazy_) &&
      vertex->hasReverseLazyParent()) {
    invalidateRLToGoalWhenInvalidatingRL(vertex->getReverseLazyParent(),
                                         vertex);
  }
}

void BiAIT::invalidateFLToStartWhenInvalidatingFL(
    const std::shared_ptr<Vertex> &vertex,
    const std::shared_ptr<Vertex> &propagateFrom) {
  // This function is same as `invalidateFLWhenInvalidatingRL`;
  base::Cost bestHeuristic = optObjPtr_->infiniteCost();
  for (const auto &child : vertex->getForwardLazyChildren()) {
    if (propagateFrom != nullptr && child->getId() == propagateFrom->getId())
      continue;
    base::Cost candidateHeuristic = optObjPtr_->combineCosts(
        child->heuristicCost_g_ReverseLazy_,
        optObjPtr_->motionCostHeuristic(child->getState(), vertex->getState()));
    if (optObjPtr_->isCostBetterThan(candidateHeuristic, bestHeuristic))
      bestHeuristic = candidateHeuristic;
  }

  vertex->heuristicCost_rhs_ReverseLazy_ = bestHeuristic;
  vertex->heuristicCost_g_ReverseLazy_ = bestHeuristic;

  // Update all the related edges in the forward valid queue;
  for (const auto &elem : vertex->validQueueIncomingLookupFromStart_) {
    elem->data.setEdgeKey(computeForwardEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    forwardValidQueue_.update(elem);
  }

  if (!optObjPtr_->isFinite(vertex->heuristicCost_g_ReverseLazy_) &&
      vertex->hasForwardLazyParent()) {
    invalidateFLToStartWhenInvalidatingFL(vertex->getForwardLazyParent(),
                                          vertex);
  }
}

void BiAIT::invalidateReverseLazyBranch(const std::shared_ptr<Vertex> &vertex) {
  if (!vertex->meetLazyEdgeQueuePointer_.empty()) {
    for (const auto &meetEdge : vertex->meetLazyEdgeQueuePointer_) {
      if (meetEdge == nullptr || !meetEdge->data.getParent().lock() ||
          !meetEdge->data.getChild().lock()) {
        if (meetEdge != nullptr && meetEdge->data.getParent().lock()) {
          Utility::removeFromVectorByValue(
              meetEdge->data.getParent().lock()->meetLazyEdgeQueuePointer_,
              meetEdge);
        }
        meetLazyEdgeQueue_.remove(meetEdge);
        continue;
      }
      Utility::removeFromVectorByValue(
          meetEdge->data.getParent().lock()->meetLazyEdgeQueuePointer_,
          meetEdge);
      if (optObjPtr_->isCostEquivalentTo(
              meetEdge->data.getParent().lock()->heuristicCost_g_ReverseLazy_,
              optObjPtr_->combineCosts(
                  meetEdge->data.getChild()
                      .lock()
                      ->heuristicCost_g_ReverseLazy_,
                  optObjPtr_->motionCostHeuristic(
                      meetEdge->data.getChild().lock()->getState(),
                      meetEdge->data.getParent().lock()->getState())))) {
        invalidateFLWhenInvalidatingRL(meetEdge->data.getParent().lock(),
                                       nullptr);
      }
      meetLazyEdgeQueue_.remove(meetEdge);
    }
    vertex->meetLazyEdgeQueuePointer_.clear();
  }
  // If this vertex is consistent before invalidation, then all incoming edges
  // now have targets that are inconsistent;
  if (vertex->isConsistentInReverseLazySearch()) {
    numInconsistentOrUnconnectedTargetsInReverse_ +=
        vertex->validQueueIncomingLookupFromStart_.size();
  }
  // Reset the cost to come from the goal and the reverse lazy children unless
  // the vertex is itself a start;
  if (!implicitGraph_.isGoal(vertex)) {
    // Reset the lazy children;
    vertex->heuristicCost_rhs_ReverseLazy_ = optObjPtr_->infiniteCost();
  }
  vertex->heuristicCost_g_ReverseLazy_ = optObjPtr_->infiniteCost();
  vertex->heuristicCost_g_ForwardLazy_ = optObjPtr_->infiniteCost();
  vertex->heuristicCost_rhs_ForwardLazy_ = optObjPtr_->infiniteCost();

  // Update all related edges in the reverse valid queue;
  for (const auto &elem : vertex->validQueueIncomingLookupFromGoal_) {
    elem->data.setEdgeKey(computeReverseEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    reverseValidQueue_.update(elem);
  }
  // Update all related edges in the forward valid queue;
  for (const auto &elem : vertex->validQueueIncomingLookupFromStart_) {
    elem->data.setEdgeKey(computeForwardEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    forwardValidQueue_.update(elem);
    //            debugPrintDataInEdgeQueue(forwardValidQueue_, "INV");
  }
  // Remove the `vertex` from the reverse lazy queue if it is in it;
  if (vertex->reverseLazyQueuePointer_) {
    reverseLazyQueue_.remove(vertex->reverseLazyQueuePointer_);
    vertex->resetReverseLazyQueuePointer();
  }

  if (vertex->hasReverseLazyParent()) {
    vertex->getReverseLazyParent()->removeFromReverseLazyChildren(
        vertex->getId());
    vertex->resetReverseLazyParent();
  }
  if (vertex->getCategory()[1]) vertex->setCategory(0, true);

  // Update the cost of all reverse lazy children;
  for (const auto &elem : vertex->getReverseLazyChildren()) {
    invalidateReverseLazyBranch(elem);
  }
  // Update the forward lazy search vertex to ensure that this vertex is placed
  // in V_open if necessary;
  updateValidQueueIfVertexInside(vertex);
}

void BiAIT::invalidateFLWhenInvalidatingRL(
    const std::shared_ptr<Vertex> &vertex,
    const std::shared_ptr<Vertex> &propagateFrom) {
  base::Cost bestHeuristic = optObjPtr_->infiniteCost();
  for (const auto &child : vertex->getForwardLazyChildren()) {
    if (propagateFrom != nullptr && child->getId() == propagateFrom->getId())
      continue;
    base::Cost candidateHeuristic = optObjPtr_->combineCosts(
        child->heuristicCost_g_ReverseLazy_,
        optObjPtr_->motionCostHeuristic(child->getState(), vertex->getState()));
    if (optObjPtr_->isCostBetterThan(candidateHeuristic, bestHeuristic))
      bestHeuristic = candidateHeuristic;
  }

  vertex->heuristicCost_rhs_ReverseLazy_ = bestHeuristic;
  vertex->heuristicCost_g_ReverseLazy_ = bestHeuristic;

  // Update all the related edges in the forward valid queue;
  for (const auto &elem : vertex->validQueueIncomingLookupFromStart_) {
    elem->data.setEdgeKey(computeForwardEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    forwardValidQueue_.update(elem);
  }

  if (!optObjPtr_->isFinite(vertex->heuristicCost_g_ReverseLazy_) &&
      vertex->hasForwardLazyParent()) {
    invalidateFLToStartWhenInvalidatingFL(vertex->getForwardLazyParent(),
                                          vertex);
  }
}

void BiAIT::invalidateRLToGoalWhenInvalidatingRL(
    const std::shared_ptr<Vertex> &vertex,
    const std::shared_ptr<Vertex> &propagateFrom) {
  // This function is same as `invalidateRLWhenInvalidatingFL`;
  base::Cost bestHeuristic = optObjPtr_->infiniteCost();
  for (const auto &child : vertex->getReverseLazyChildren()) {
    if (propagateFrom != nullptr && child->getId() == propagateFrom->getId())
      continue;
    base::Cost candidateHeuristic = optObjPtr_->combineCosts(
        child->heuristicCost_g_ForwardLazy_,
        optObjPtr_->motionCostHeuristic(child->getState(), vertex->getState()));
    if (optObjPtr_->isCostBetterThan(candidateHeuristic, bestHeuristic))
      bestHeuristic = candidateHeuristic;
  }

  vertex->heuristicCost_rhs_ForwardLazy_ = bestHeuristic;
  vertex->heuristicCost_g_ForwardLazy_ = bestHeuristic;

  // Update all related edges in the reverse valid queue;
  for (const auto &elem : vertex->validQueueIncomingLookupFromGoal_) {
    elem->data.setEdgeKey(computeReverseEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    reverseValidQueue_.update(elem);
  }

  if (!optObjPtr_->isFinite(vertex->heuristicCost_g_ForwardLazy_) &&
      vertex->hasReverseLazyParent()) {
    invalidateRLToGoalWhenInvalidatingRL(vertex->getReverseLazyParent(),
                                         vertex);
  }
}

void BiAIT::invalidateAllLazyComponent() {
  auto vectorOfVerticesPtr = implicitGraph_.getVertices();
  for (const auto &vertex : vectorOfVerticesPtr) {
    vertex->setHeuristicCostsToInfinity();
    vertex->resetForwardLazyParent();
    vertex->resetForwardLazyChildren();
    vertex->resetReverseLazyParent();
    vertex->resetReverseLazyChildren();
    vertex->resetForwardLazyQueuePointer();
    vertex->resetReverseLazyQueuePointer();
    vertex->meetLazyEdgeQueuePointer_.clear();
  }
  meetLazyEdgeQueue_.clear();
}

void BiAIT::updateVertexInFLSearch(const std::shared_ptr<Vertex> &vertex) {
  // If the vertex is a start, there's no updating to do.
  if (implicitGraph_.isStart(vertex) ||
      !vertex->validQueueIncomingLookupFromGoal_.empty()) {
    return;
  } else if (!vertex->getCategory()[0] && !vertex->getCategory()[1]) {
    // We process this vertex and try to connect or update it to the current
    // associated tree;
    updateVertexInFLSearchToFLTree(vertex);
  } else if (vertex->getCategory()[0]) {
    updateVertexInFLSearchToMeetQueue(vertex);
  }
}

void BiAIT::updateVertexInFLSearchToFLTree(
    const std::shared_ptr<Vertex> &vertex) {
  // Get the best parent for this vertex;
  auto bestParent = vertex->getForwardLazyParent();
  auto bestCost = optObjPtr_->infiniteCost();

  updateVertexInFLSearch_bestParent(vertex, bestParent, bestCost);
  // Set the best cost as the cost-to-come from start;
  vertex->heuristicCost_rhs_ForwardLazy_ = bestCost;

  if (optObjPtr_->isFinite(bestCost)) {
    // The vertex is connected; then update the forward lazy parent;
    vertex->setForwardLazyParent(bestParent);
    bestParent->addToForwardLazyChildren(vertex);
  } else {
    // The vertex is orphaned, currently; reset the forward lazy parent if there
    // is one;
    if (vertex->hasForwardLazyParent()) {
      vertex->getForwardLazyParent()->removeFromForwardLazyChildren(
          vertex->getId());
      vertex->resetForwardLazyParent();
    }
  }
  // If this vertex is inconsistent now, insert or update it in the open queue;
  if (!vertex->isConsistentInForwardLazySearch()) {
    insertOrUpdateForwardLazyQueue(vertex);
  } else {
    if (auto elemPtr = vertex->forwardLazyQueuePointer_) {
      // Remove this vertex from the open queue if it is in the queue and is
      // consistent;
      forwardLazyQueue_.remove(elemPtr);
      vertex->resetForwardLazyQueuePointer();
    }
  }
  updateValidQueueIfVertexInside(vertex);
}

void BiAIT::updateVertexInFLSearchToMeetQueue(
    const std::shared_ptr<Vertex> &vertex) {
  // `vertex` is in the RL tree, we find the best parent in FL tree for RL, if
  // both vertex parent is consistent in RL and parent is consistent in FL, we
  // connect vertex and the found parent with `meetLazyEdge`; Get the best
  // parent for this vertex;
  if (!vertex->isConsistentInReverseLazySearch() ||
      !optObjPtr_->isFinite(vertex->heuristicCost_g_ReverseLazy_))
    return;
  std::weak_ptr<Vertex> bestParent;
  base::Cost bestCost = optObjPtr_->infiniteCost();

  bool foundBestParent =
      updateVertexInFLSearch_bestWeakParent(vertex, bestParent, bestCost);
  if (!foundBestParent || !bestParent.lock()->getCategory()[3] ||
      !bestParent.lock()->isConsistentInForwardLazySearch())
    return;
  // lazyTreesMeet, `vertex`: RL, `bestParent`: FL;
  insertOrUpdateMeetLazyQueue(bestParent, vertex);

  // Propagate the g-value and rhs-value in both lazy trees;
  auto meetEdgeCost = optObjPtr_->motionCostHeuristic(
      bestParent.lock()->getState(), vertex->getState());
  if (optObjPtr_->isCostBetterThan(
          optObjPtr_->combineCosts(vertex->heuristicCost_g_ReverseLazy_,
                                   meetEdgeCost),
          bestParent.lock()->heuristicCost_rhs_ReverseLazy_)) {
    bestParent.lock()->heuristicCost_rhs_ReverseLazy_ =
        optObjPtr_->combineCosts(vertex->heuristicCost_g_ReverseLazy_,
                                 meetEdgeCost);
    bestParent.lock()->heuristicCost_g_ReverseLazy_ =
        bestParent.lock()->heuristicCost_rhs_ReverseLazy_;
    if (bestParent.lock()->reverseLazyQueuePointer_) {
      reverseLazyQueue_.remove(bestParent.lock()->reverseLazyQueuePointer_);
      bestParent.lock()->resetReverseLazyQueuePointer();
    }
    propagateCostToGoalInFLTree(bestParent.lock());
    updateValidQueueIfVertexInside(bestParent.lock());
  }
  if (optObjPtr_->isCostBetterThan(
          optObjPtr_->combineCosts(
              bestParent.lock()->heuristicCost_g_ForwardLazy_, meetEdgeCost),
          vertex->heuristicCost_g_ForwardLazy_)) {
    vertex->heuristicCost_rhs_ForwardLazy_ = optObjPtr_->combineCosts(
        bestParent.lock()->heuristicCost_g_ForwardLazy_, meetEdgeCost);
    vertex->heuristicCost_g_ForwardLazy_ =
        vertex->heuristicCost_rhs_ForwardLazy_;
    if (vertex->forwardLazyQueuePointer_) {
      forwardLazyQueue_.remove(vertex->forwardLazyQueuePointer_);
      vertex->resetForwardLazyQueuePointer();
    }
    propagateCostFromStartInRLTree(vertex);
    updateValidQueueIfVertexInside(vertex);
  }
}

void BiAIT::updateVertexInFLSearch_bestParent(
    const std::shared_ptr<Vertex> &vertex, std::shared_ptr<Vertex> &bestParent,
    base::Cost &bestCost) {
  // Check all neighbors in RGG, and record the best lazy parent and the cost to
  // root through this parent;
  for (const auto &elem : implicitGraph_.getNeighbors(vertex)) {
    if (elem->getId() != vertex->getId() &&
        !elem->hasBlacklistedChild(vertex) &&
        !vertex->hasBlacklistedChild(elem)) {
      auto edgeCost =
          optObjPtr_->motionCostHeuristic(elem->getState(), vertex->getState());
      auto iterateCost = optObjPtr_->combineCosts(
          elem->heuristicCost_g_ForwardLazy_, edgeCost);
      if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
        bestParent = elem;
        bestCost = iterateCost;
      }
    }
  }
  // Check all children this vertex holds in the reverse valid search;
  for (const auto &reverseValidChild : vertex->getReverseValidChildren()) {
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        reverseValidChild->getState(), vertex->getState());
    auto iterateCost = optObjPtr_->combineCosts(
        reverseValidChild->heuristicCost_g_ForwardLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
      bestParent = reverseValidChild;
      bestCost = iterateCost;
    }
  }
  // Check all children this vertex holds in the forward valid search;
  for (const auto &forwardValidChild : vertex->getForwardValidChildren()) {
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        forwardValidChild->getState(), vertex->getState());
    auto iterateCost = optObjPtr_->combineCosts(
        forwardValidChild->heuristicCost_g_ForwardLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
      bestParent = forwardValidChild;
      bestCost = iterateCost;
    }
  }
  // Check the parent of this vertex in the valid search;
  if (vertex->hasForwardValidParent()) {
    auto forwardValidParent = vertex->getForwardValidParent();
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        forwardValidParent->getState(), vertex->getState());
    auto candidateCost = optObjPtr_->combineCosts(
        forwardValidParent->heuristicCost_g_ForwardLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(candidateCost, bestCost)) {
      bestParent = forwardValidParent;
      bestCost = candidateCost;
    }
  }
  if (vertex->hasReverseValidParent()) {
    auto reverseValidParent = vertex->getReverseValidParent();
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        reverseValidParent->getState(), vertex->getState());
    auto candidateCost = optObjPtr_->combineCosts(
        reverseValidParent->heuristicCost_g_ForwardLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(candidateCost, bestCost)) {
      bestParent = reverseValidParent;
      bestCost = candidateCost;
    }
  }
}

bool BiAIT::updateVertexInFLSearch_bestWeakParent(
    const std::shared_ptr<Vertex> &vertex, std::weak_ptr<Vertex> &bestParent,
    base::Cost &bestCost) {
  bool foundBestParent{false};
  // Check all neighbors in RGG, and record the best lazy parent and the cost to
  // root through this parent;
  for (const auto &elem : implicitGraph_.getNeighbors(vertex)) {
    if (elem->getId() != vertex->getId() &&
        !elem->hasBlacklistedChild(vertex) &&
        !vertex->hasBlacklistedChild(elem)) {
      auto edgeCost =
          optObjPtr_->motionCostHeuristic(elem->getState(), vertex->getState());
      auto iterateCost = optObjPtr_->combineCosts(
          elem->heuristicCost_g_ForwardLazy_, edgeCost);
      if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
        bestParent = elem;
        bestCost = iterateCost;
        foundBestParent = true;
      }
    }
  }
  // Check all children this vertex holds in the reverse valid search;
  for (const auto &reverseValidChild : vertex->getReverseValidWeakChildren()) {
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        reverseValidChild.lock()->getState(), vertex->getState());
    auto iterateCost = optObjPtr_->combineCosts(
        reverseValidChild.lock()->heuristicCost_g_ForwardLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
      bestParent = reverseValidChild;
      bestCost = iterateCost;
      foundBestParent = true;
    }
  }
  // Check all children this vertex holds in the forward valid search;
  for (const auto &forwardValidChild : vertex->getForwardValidWeakChildren()) {
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        forwardValidChild.lock()->getState(), vertex->getState());
    auto iterateCost = optObjPtr_->combineCosts(
        forwardValidChild.lock()->heuristicCost_g_ForwardLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
      bestParent = forwardValidChild;
      bestCost = iterateCost;
      foundBestParent = true;
    }
  }
  // Check the parent of this vertex in the valid search;
  if (vertex->hasForwardValidParent()) {
    auto forwardValidParent = vertex->getForwardValidWeakParent();
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        forwardValidParent.lock()->getState(), vertex->getState());
    auto candidateCost = optObjPtr_->combineCosts(
        forwardValidParent.lock()->heuristicCost_g_ForwardLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(candidateCost, bestCost)) {
      bestParent = forwardValidParent;
      bestCost = candidateCost;
      foundBestParent = true;
    }
  }
  if (vertex->hasReverseValidParent()) {
    auto reverseValidParent = vertex->getReverseValidWeakParent();
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        reverseValidParent.lock()->getState(), vertex->getState());
    auto candidateCost = optObjPtr_->combineCosts(
        reverseValidParent.lock()->heuristicCost_g_ForwardLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(candidateCost, bestCost)) {
      bestParent = reverseValidParent;
      bestCost = candidateCost;
      foundBestParent = true;
    }
  }
  return foundBestParent;
}

void BiAIT::updateVertexInRLSearch(const std::shared_ptr<Vertex> &vertex) {
  // If the vertex is a goal, there's no updating to do.
  if (implicitGraph_.isGoal(vertex) ||
      !vertex->validQueueIncomingLookupFromStart_.empty()) {
    return;
  } else if (!vertex->getCategory()[3] && !vertex->getCategory()[4]) {
    // We process this vertex and try to connect or update it to the current
    // associated tree;
    updateVertexInRLSearchToRLTree(vertex);
  } else if (vertex->getCategory()[3]) {
    updateVertexInRLSearchToMeetQueue(vertex);
  }
}

void BiAIT::updateVertexInRLSearchToRLTree(
    const std::shared_ptr<Vertex> &vertex) {
  // Get the best parent for this vertex;
  auto bestParent = vertex->getReverseLazyParent();
  auto bestCost = optObjPtr_->infiniteCost();
  updateVertexInRLSearch_bestParent(vertex, bestParent, bestCost);
  // Set the best cost as the cost-to-come from start;
  vertex->heuristicCost_rhs_ReverseLazy_ = bestCost;

  if (optObjPtr_->isFinite(bestCost)) {
    // The vertex is connected; then update the reverse lazy parent;
    vertex->setReverseLazyParent(bestParent);
    bestParent->addToReverseLazyChildren(vertex);
  } else {
    // The vertex is orphaned, currently; reset the reverse lazy parent if there
    // is one;
    if (vertex->hasReverseLazyParent()) {
      vertex->getReverseLazyParent()->removeFromReverseLazyChildren(
          vertex->getId());
      vertex->resetReverseLazyParent();
    }
  }
  // If this vertex is inconsistent now, insert or update it in the open queue;
  if (!vertex->isConsistentInReverseLazySearch()) {
    insertOrUpdateReverseLazyQueue(vertex);
  } else {
    if (auto elemPtr = vertex->reverseLazyQueuePointer_) {
      // Remove this vertex from the open queue if it is in the queue and is
      // consistent;
      reverseLazyQueue_.remove(elemPtr);
      vertex->resetReverseLazyQueuePointer();
    }
  }
  updateValidQueueIfVertexInside(vertex);
}

void BiAIT::updateVertexInRLSearchToMeetQueue(
    const std::shared_ptr<Vertex> &vertex) {
  // `vertex` is in the FL tree, we find the best parent in RL tree for FL, if
  // both vertex is consistent in FL and parent is consistent in RL, we connect
  // vertex and the found parent with `meetLazyEdge`; Get the best parent for
  // this vertex;
  if (!vertex->isConsistentInForwardLazySearch() ||
      !optObjPtr_->isFinite(vertex->heuristicCost_g_ForwardLazy_))
    return;
  std::weak_ptr<Vertex> bestParent;
  base::Cost bestCost = optObjPtr_->infiniteCost();

  bool foundBestParent =
      updateVertexInRLSearch_bestWeakParent(vertex, bestParent, bestCost);
  if (!foundBestParent || !bestParent.lock()->getCategory()[0] ||
      !bestParent.lock()->isConsistentInReverseLazySearch())
    return;
  // lazyTreesMeet, `vertex`: FL, `bestParent`: RL;
  insertOrUpdateMeetLazyQueue(vertex, bestParent);

  // Propagate the g-value and rhs-value in both lazy trees;
  auto meetEdgeCost = optObjPtr_->motionCostHeuristic(
      vertex->getState(), bestParent.lock()->getState());
  if (optObjPtr_->isCostBetterThan(
          optObjPtr_->combineCosts(vertex->heuristicCost_g_ForwardLazy_,
                                   meetEdgeCost),
          bestParent.lock()->heuristicCost_g_ForwardLazy_)) {
    bestParent.lock()->heuristicCost_rhs_ForwardLazy_ =
        optObjPtr_->combineCosts(vertex->heuristicCost_g_ForwardLazy_,
                                 meetEdgeCost);
    bestParent.lock()->heuristicCost_g_ForwardLazy_ =
        bestParent.lock()->heuristicCost_rhs_ForwardLazy_;
    if (bestParent.lock()->forwardLazyQueuePointer_) {
      forwardLazyQueue_.remove(bestParent.lock()->forwardLazyQueuePointer_);
      bestParent.lock()->resetForwardLazyQueuePointer();
    }
    propagateCostFromStartInRLTree(bestParent.lock());
    updateValidQueueIfVertexInside(bestParent.lock());
  }
  if (optObjPtr_->isCostBetterThan(
          optObjPtr_->combineCosts(
              bestParent.lock()->heuristicCost_g_ReverseLazy_, meetEdgeCost),
          vertex->heuristicCost_g_ReverseLazy_)) {
    vertex->heuristicCost_rhs_ReverseLazy_ = optObjPtr_->combineCosts(
        bestParent.lock()->heuristicCost_g_ReverseLazy_, meetEdgeCost);
    vertex->heuristicCost_g_ReverseLazy_ =
        vertex->heuristicCost_rhs_ReverseLazy_;
    if (vertex->reverseLazyQueuePointer_) {
      reverseLazyQueue_.remove(vertex->reverseLazyQueuePointer_);
      vertex->resetReverseLazyQueuePointer();
    }
    propagateCostToGoalInFLTree(vertex);
    updateValidQueueIfVertexInside(vertex);
  }
}

void BiAIT::updateVertexInRLSearch_bestParent(
    const std::shared_ptr<Vertex> &vertex, std::shared_ptr<Vertex> &bestParent,
    base::Cost &bestCost) {
  // Check all neighbors in RGG, and record the best lazy parent and the cost to
  // root through this parent;
  for (const auto &elem : implicitGraph_.getNeighbors(vertex)) {
    if (elem->getId() != vertex->getId() &&
        !elem->hasBlacklistedChild(vertex) &&
        !vertex->hasBlacklistedChild(elem)) {
      auto edgeCost =
          optObjPtr_->motionCostHeuristic(elem->getState(), vertex->getState());
      auto iterateCost = optObjPtr_->combineCosts(
          elem->heuristicCost_g_ReverseLazy_, edgeCost);
      if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
        bestParent = elem;
        bestCost = iterateCost;
      }
    }
  }
  // Check all children this vertex holds in the forward valid search;
  for (const auto &forwardValidChild : vertex->getForwardValidChildren()) {
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        forwardValidChild->getState(), vertex->getState());
    auto iterateCost = optObjPtr_->combineCosts(
        forwardValidChild->heuristicCost_g_ReverseLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
      bestParent = forwardValidChild;
      bestCost = iterateCost;
    }
  }
  // Check all children this vertex holds in the forward valid search;
  for (const auto &reverseValidChild : vertex->getReverseValidChildren()) {
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        reverseValidChild->getState(), vertex->getState());
    auto iterateCost = optObjPtr_->combineCosts(
        reverseValidChild->heuristicCost_g_ReverseLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
      bestParent = reverseValidChild;
      bestCost = iterateCost;
    }
  }
  // Check the parent of this vertex in the forward valid search;
  if (vertex->hasForwardValidParent()) {
    auto forwardValidParent = vertex->getForwardValidParent();
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        forwardValidParent->getState(), vertex->getState());
    auto candidateCost = optObjPtr_->combineCosts(
        forwardValidParent->heuristicCost_g_ReverseLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(candidateCost, bestCost)) {
      bestParent = forwardValidParent;
      bestCost = candidateCost;
    }
  }
  if (vertex->hasReverseValidParent()) {
    auto reverseValidParent = vertex->getReverseValidParent();
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        reverseValidParent->getState(), vertex->getState());
    auto candidateCost = optObjPtr_->combineCosts(
        reverseValidParent->heuristicCost_g_ReverseLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(candidateCost, bestCost)) {
      bestParent = reverseValidParent;
      bestCost = candidateCost;
    }
  }
}

bool BiAIT::updateVertexInRLSearch_bestWeakParent(
    const std::shared_ptr<Vertex> &vertex, std::weak_ptr<Vertex> &bestParent,
    base::Cost &bestCost) {
  bool foundBestParent{false};
  // Check all neighbors in RGG, and record the best lazy parent and the cost to
  // root through this parent;
  for (const auto &elem : implicitGraph_.getNeighbors(vertex)) {
    if (elem->getId() != vertex->getId() &&
        !elem->hasBlacklistedChild(vertex) &&
        !vertex->hasBlacklistedChild(elem)) {
      auto edgeCost =
          optObjPtr_->motionCostHeuristic(elem->getState(), vertex->getState());
      auto iterateCost = optObjPtr_->combineCosts(
          elem->heuristicCost_g_ReverseLazy_, edgeCost);
      if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
        bestParent = elem;
        bestCost = iterateCost;
        foundBestParent = true;
      }
    }
  }
  // Check all children this vertex holds in the forward valid search;
  for (const auto &forwardValidChild : vertex->getForwardValidWeakChildren()) {
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        forwardValidChild.lock()->getState(), vertex->getState());
    auto iterateCost = optObjPtr_->combineCosts(
        forwardValidChild.lock()->heuristicCost_g_ReverseLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
      bestParent = forwardValidChild;
      bestCost = iterateCost;
      foundBestParent = true;
    }
  }
  // Check all children this vertex holds in the forward valid search;
  for (const auto &reverseValidChild : vertex->getReverseValidWeakChildren()) {
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        reverseValidChild.lock()->getState(), vertex->getState());
    auto iterateCost = optObjPtr_->combineCosts(
        reverseValidChild.lock()->heuristicCost_g_ReverseLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(iterateCost, bestCost)) {
      bestParent = reverseValidChild;
      bestCost = iterateCost;
      foundBestParent = true;
    }
  }
  // Check the parent of this vertex in the forward valid search;
  if (vertex->hasForwardValidParent()) {
    auto forwardValidParent = vertex->getForwardValidWeakParent();
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        forwardValidParent.lock()->getState(), vertex->getState());
    auto candidateCost = optObjPtr_->combineCosts(
        forwardValidParent.lock()->heuristicCost_g_ReverseLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(candidateCost, bestCost)) {
      bestParent = forwardValidParent;
      bestCost = candidateCost;
      foundBestParent = true;
    }
  }
  if (vertex->hasReverseValidParent()) {
    auto reverseValidParent = vertex->getReverseValidWeakParent();
    auto edgeCost = optObjPtr_->motionCostHeuristic(
        reverseValidParent.lock()->getState(), vertex->getState());
    auto candidateCost = optObjPtr_->combineCosts(
        reverseValidParent.lock()->heuristicCost_g_ReverseLazy_, edgeCost);
    if (optObjPtr_->isCostBetterThan(candidateCost, bestCost)) {
      bestParent = reverseValidParent;
      bestCost = candidateCost;
      foundBestParent = true;
    }
  }
  return foundBestParent;
}

void BiAIT::updateForwardLazyNeighbors(const std::shared_ptr<Vertex> &vertex) {
  // Start with forward lazy children, because if this vertex becomes the parent
  // of a neighbor, that neighbor would be updated again as part of the forward
  // lazy children;
  for (const auto &elem : vertex->getForwardLazyChildren()) {
    updateVertexInFLSearch(elem);
  }
  // AIT: we can now process the neighbors;
  for (const auto &elem : implicitGraph_.getNeighbors(vertex)) {
    if (elem->getId() != vertex->getId() &&
        !elem->hasBlacklistedChild(vertex) &&
        !vertex->hasBlacklistedChild(elem)) {
      updateVertexInFLSearch(elem);
    }
  }
  // AIT: we also need to process the forward search children; (BiAIT: we also
  // need to process the reverse valid search child)
  for (const auto &elem : vertex->getReverseValidChildren()) {
    updateVertexInFLSearch(elem);
  }
  // AIT: we also need to update the forward search parent if it exists; (BiAIT:
  // update the valid parent if it exists)
  if (vertex->hasForwardValidParent()) {
    updateVertexInFLSearch(vertex->getForwardValidParent());
  }
  if (vertex->hasReverseValidParent()) {
    updateVertexInFLSearch(vertex->getReverseValidParent());
  }
}

void BiAIT::updateReverseLazyNeighbors(const std::shared_ptr<Vertex> &vertex) {
  // Start with reverse lazy children, because if this vertex becomes the parent
  // of a neighbor, that neighbor would be updated again as part of the reverse
  // lazy children; AIT: we can now process the neighbors;
  for (const auto &elem : vertex->getReverseLazyChildren()) {
    updateVertexInRLSearch(elem);
  }
  for (const auto &elem : implicitGraph_.getNeighbors(vertex)) {
    if (elem->getId() != vertex->getId() &&
        !elem->hasBlacklistedChild(vertex) &&
        !vertex->hasBlacklistedChild(elem)) {
      updateVertexInRLSearch(elem);
    }
  }
  // BiAIT: we also need to process the forward valid search child;
  for (const auto &elem : vertex->getForwardValidChildren()) {
    updateVertexInRLSearch(elem);
  }
  // BiAIT: update the valid parent if it exists;
  if (vertex->hasForwardValidParent()) {
    updateVertexInRLSearch(vertex->getForwardValidParent());
  }
  if (vertex->hasReverseValidParent()) {
    updateVertexInRLSearch(vertex->getReverseValidParent());
  }
}

void BiAIT::propagateCostFromStartInRLTree(
    const std::shared_ptr<Vertex> &vertex) {
  if (implicitGraph_.isGoal(vertex)) return;
  if (const auto parent = vertex->getReverseLazyParent()) {
    auto candidateParentCostFromStart =
        optObjPtr_->combineCosts(vertex->heuristicCost_g_ForwardLazy_,
                                 optObjPtr_->motionCostHeuristic(
                                     vertex->getState(), parent->getState()));
    if (optObjPtr_->isCostBetterThan(candidateParentCostFromStart,
                                     parent->heuristicCost_g_ForwardLazy_)) {
      parent->heuristicCost_rhs_ForwardLazy_ = candidateParentCostFromStart;
      parent->heuristicCost_g_ForwardLazy_ =
          parent->heuristicCost_rhs_ForwardLazy_;
      if (parent->forwardLazyQueuePointer_) {
        forwardLazyQueue_.remove(parent->forwardLazyQueuePointer_);
        parent->resetForwardLazyQueuePointer();
      }
      propagateCostFromStartInRLTree(parent);
      updateValidQueueIfVertexInside(parent);
    }
  }
}

void BiAIT::propagateCostToGoalInFLTree(const std::shared_ptr<Vertex> &vertex) {
  if (implicitGraph_.isStart(vertex)) return;
  if (const auto parent = vertex->getForwardLazyParent()) {
    auto candidateParentCostToGoal =
        optObjPtr_->combineCosts(vertex->heuristicCost_g_ReverseLazy_,
                                 optObjPtr_->motionCostHeuristic(
                                     parent->getState(), vertex->getState()));
    if (optObjPtr_->isCostBetterThan(candidateParentCostToGoal,
                                     parent->heuristicCost_g_ReverseLazy_)) {
      parent->heuristicCost_rhs_ReverseLazy_ = candidateParentCostToGoal;
      parent->heuristicCost_g_ReverseLazy_ =
          parent->heuristicCost_rhs_ReverseLazy_;
      if (parent->reverseLazyQueuePointer_) {
        reverseLazyQueue_.remove(parent->reverseLazyQueuePointer_);
        parent->resetReverseLazyQueuePointer();
      }
      propagateCostToGoalInFLTree(parent);
      updateValidQueueIfVertexInside(parent);
    }
  }
}

void BiAIT::updateValidQueueIfVertexInside(
    const std::shared_ptr<Vertex> &vertex) {
  // This vertex now has a changed heuristic. All edges in the F&R valid queue
  // that have this vertex as a child must be updated. This can not be delayed,
  // as whether the lazy search can be suspended depends on the best edge in the
  // valid queue;
  updateFVValidQueueIfVertexInside(vertex);
  updateRVValidQueueIfVertexInside(vertex);
}

void BiAIT::updateFVValidQueueIfVertexInside(
    const std::shared_ptr<Vertex> &vertex) {
  for (const auto &elem : vertex->validQueueIncomingLookupFromStart_) {
    elem->data.setEdgeKey(computeForwardEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    forwardValidQueue_.update(elem);
  }
}

void BiAIT::updateRVValidQueueIfVertexInside(
    const std::shared_ptr<Vertex> &vertex) {
  for (const auto &elem : vertex->validQueueIncomingLookupFromGoal_) {
    elem->data.setEdgeKey(computeReverseEdgeKey(
        optObjPtr_, elem->data.getParent(), elem->data.getChild()));
    reverseValidQueue_.update(elem);
  }
}

MeetLazyEdgeQueue::Element *BiAIT::findInMeetLazyQueue(const Edge &edge) {
  // We encourage making the parent close to the start and child close to the
  // goal; But this function can also work otherwise;
  auto parentId = edge.getParent()->getId();
  auto childId = edge.getChild()->getId();
  std::vector<WeakEdge> meetLazyEdges{};
  meetLazyEdges.reserve(meetLazyEdgeQueue_.size());
  meetLazyEdgeQueue_.getContent(meetLazyEdges);
  auto found =
      std::find_if(meetLazyEdges.begin(), meetLazyEdges.end(),
                   [parentId, childId](const auto &elem) {
                     return (elem.getParent().lock()->getId() == parentId &&
                             elem.getChild().lock()->getId() == childId) ||
                            (elem.getParent().lock()->getId() == childId &&
                             elem.getChild().lock()->getId() == parentId);
                   });
  // `edge` is not in the `meetLazyEdgeQueue_`, and return nullptr;
  if (found == meetLazyEdges.end()) return nullptr;
  // Else, we try to figure out the pointer to the meetLazyEdge in
  // `meetLazyEdgeQueue_`;
  else {
    auto parentPointerToElemVector =
        found->getParent().lock()->meetLazyEdgeQueuePointer_;
    auto childPointerToElemVector =
        found->getChild().lock()->meetLazyEdgeQueuePointer_;
    for (const auto parentPointerToElem : parentPointerToElemVector) {
      if (std::find_if(childPointerToElemVector.begin(),
                       childPointerToElemVector.end(),
                       [parentPointerToElem](const auto elem) {
                         return parentPointerToElem == elem;
                       }) != childPointerToElemVector.end()) {
        return parentPointerToElem;
      }
    }
    return nullptr;
  }
}

MeetLazyEdgeQueue::Element *BiAIT::findInMeetLazyQueue(const WeakEdge &edge) {
  // We encourage making the parent close to the start and child close to the
  // goal; But this function can also work otherwise;
  auto parentId = edge.getParent().lock()->getId();
  auto childId = edge.getChild().lock()->getId();
  std::vector<WeakEdge> meetLazyEdges{};
  meetLazyEdges.reserve(meetLazyEdgeQueue_.size());
  meetLazyEdgeQueue_.getContent(meetLazyEdges);
  auto found =
      std::find_if(meetLazyEdges.begin(), meetLazyEdges.end(),
                   [parentId, childId](const auto &elem) {
                     return (elem.getParent().lock()->getId() == parentId &&
                             elem.getChild().lock()->getId() == childId) ||
                            (elem.getParent().lock()->getId() == childId &&
                             elem.getChild().lock()->getId() == parentId);
                   });
  // `edge` is not in the `meetLazyEdgeQueue_`, and return nullptr;
  if (found == meetLazyEdges.end()) return nullptr;
  // Else, we try to figure out the pointer to the meetLazyEdge in
  // `meetLazyEdgeQueue_`;
  else {
    auto parentPointerToElemVector =
        found->getParent().lock()->meetLazyEdgeQueuePointer_;
    auto childPointerToElemVector =
        found->getChild().lock()->meetLazyEdgeQueuePointer_;
    for (const auto parentPointerToElem : parentPointerToElemVector) {
      if (std::find_if(childPointerToElemVector.begin(),
                       childPointerToElemVector.end(),
                       [parentPointerToElem](const auto elem) {
                         return parentPointerToElem == elem;
                       }) != childPointerToElemVector.end()) {
        return parentPointerToElem;
      }
    }
    return nullptr;
  }
}

MeetValidEdgeQueue::Element *BiAIT::findInMeetValidQueue(const Edge &edge) {
  // We encourage making the parent close to the start and child close to the
  // goal; But this function can also work otherwise;
  auto parentId = edge.getParent()->getId();
  auto childId = edge.getChild()->getId();
  std::vector<CostEdgePair> meetValidEdges{};
  meetValidEdgeQueue_.getContent(meetValidEdges);
  auto found =
      std::find_if(meetValidEdges.begin(), meetValidEdges.end(),
                   [parentId, childId](const auto &elem) {
                     return (elem.second.getParent()->getId() == parentId &&
                             elem.second.getChild()->getId() == childId) ||
                            (elem.second.getParent()->getId() == childId &&
                             elem.second.getChild()->getId() == parentId);
                   });
  // `edge` is not in the `meetValidEdgeQueue_`, and return nullptr;
  if (found == meetValidEdges.end()) return nullptr;
  // Else, we try to figure out the pointer to the meetValidEdge in
  // `meetValidEdgeQueue_`;
  else {
    auto parentPointerToElemVector =
        found->second.getParent()->meetValidEdgeQueuePointer_;
    auto childPointerToElemVector =
        found->second.getChild()->meetValidEdgeQueuePointer_;
    for (const auto parentPointerToElem : parentPointerToElemVector) {
      if (std::find_if(childPointerToElemVector.begin(),
                       childPointerToElemVector.end(),
                       [parentPointerToElem](const auto elem) {
                         return parentPointerToElem == elem;
                       }) != childPointerToElemVector.end()) {
        return parentPointerToElem;
      }
    }
    throw(
        std::logic_error("Found `edge` in `meetValidEdgeQueue_`, but the found "
                         "edge's parent and child do not correct pointer to "
                         "the `meetValidEdgeQueue_::Element*`;"));
  }
}

void BiAIT::removeFromMeetValidQueue(
    const std::vector<MeetValidEdgeQueue::Element *>
        &vectorOfMeetValidEdgesToBePruned) {
  for (const auto &elem : vectorOfMeetValidEdgesToBePruned) {
    if (elem->data.second.getParent()->meetValidEdgeQueuePointer_.size() == 1) {
      // TODO: propagate the change of cost-to-Goal to inf via FV branch;
      elem->data.second.getParent()->costToGoal_ = optObjPtr_->infiniteCost();
    }
    if (elem->data.second.getChild()->meetValidEdgeQueuePointer_.size() == 1) {
      // TODO: propagate the change of cost-to-Start to inf via RV branch;
      elem->data.second.getChild()->costToStart_ = optObjPtr_->infiniteCost();
    }
    Utility::removeFromVectorByValue(
        elem->data.second.getParent()->meetValidEdgeQueuePointer_, elem);
    Utility::removeFromVectorByValue(
        elem->data.second.getChild()->meetValidEdgeQueuePointer_, elem);
    meetValidEdgeQueue_.remove(elem);
  }
}

void BiAIT::checkMeetLazyEdgeToValid(
    MeetLazyEdgeQueue::Element *meetLazyEdgePointer) {
  if (!meetLazyEdgePointer->data.getParent().lock() ||
      !meetLazyEdgePointer->data.getChild().lock()) {
    return;
  }
  const auto &candidateParent = meetLazyEdgePointer->data.getParent().lock();
  const auto &candidateChild = meetLazyEdgePointer->data.getChild().lock();
  if (candidateParent->getCategory()[4] && candidateChild->getCategory()[1]) {
    // The meetLazyEdge has potential to be a meetValidEdge;
    if (candidateParent->hasWhitelistedChild(candidateChild) ||
        motionValidatorPtr_->checkMotion(candidateParent->getState(),
                                         candidateChild->getState())) {
      // Collision free!
      if (!candidateParent->hasWhitelistedChild(candidateChild)) {
        candidateParent->whitelistAsChild(candidateChild);
        ++numEdgeCollisionChecks_;
      }
      candidateParent->setCategory(2, true);
      candidateChild->setCategory(2, true);
      const auto edgeCost = optObjPtr_->motionCost(candidateParent->getState(),
                                                   candidateChild->getState());
      if (optObjPtr_->isCostBetterThan(
              optObjPtr_->combineCosts(candidateParent->costToGoal_, edgeCost),
              candidateParent->costToGoal_)) {
        candidateParent->costToGoal_ =
            optObjPtr_->combineCosts(candidateChild->costToGoal_, edgeCost);
      }
      if (optObjPtr_->isCostBetterThan(
              optObjPtr_->combineCosts(candidateChild->costToStart_, edgeCost),
              candidateChild->costToStart_)) {
        candidateChild->costToStart_ =
            optObjPtr_->combineCosts(candidateParent->costToStart_, edgeCost);
      }

      Edge meetValidEdge(
          candidateParent, candidateChild, 0b00100,
          {{optObjPtr_->infiniteCost(), optObjPtr_->infiniteCost(),
            optObjPtr_->infiniteCost()}});
      insertOrUpdateMeetValidEdge(meetValidEdge, edgeCost);
    } else {
      candidateParent->blacklistAsChild(candidateChild);
      candidateChild->blacklistAsChild(candidateParent);
      // the meetLazyEdge has collision with obstacles;
      Utility::removeFromVectorByValue(
          candidateParent->meetLazyEdgeQueuePointer_, meetLazyEdgePointer);
      if (optObjPtr_->isCostEquivalentTo(
              candidateParent->heuristicCost_g_ReverseLazy_,
              optObjPtr_->combineCosts(
                  candidateChild->heuristicCost_g_ReverseLazy_,
                  optObjPtr_->motionCostHeuristic(
                      candidateChild->getState(),
                      candidateParent->getState())))) {
        invalidateFLWhenInvalidatingRL(candidateParent, nullptr);
      }
      Utility::removeFromVectorByValue(
          candidateChild->meetLazyEdgeQueuePointer_, meetLazyEdgePointer);
      if (optObjPtr_->isCostEquivalentTo(
              candidateChild->heuristicCost_g_ForwardLazy_,
              optObjPtr_->combineCosts(
                  candidateParent->heuristicCost_g_ForwardLazy_,
                  optObjPtr_->motionCostHeuristic(
                      candidateParent->getState(),
                      candidateChild->getState())))) {
        invalidateRLWhenInvalidatingFL(candidateChild, nullptr);
      }
      meetLazyEdgeQueue_.remove(meetLazyEdgePointer);
      updateVertexInRLSearchToMeetQueue(candidateParent);
      updateVertexInFLSearchToMeetQueue(candidateChild);
    }
  }
}
}  // namespace ompl::geometric