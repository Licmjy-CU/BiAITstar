//
// Created by Chenming LI on 13/3/2022.
//

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>

#include "BiAITstar/BiAIT.h"

using namespace ompl;

const double stateValidityCheckingResolution{0.0002};

#define OMPLAPP_CUSTOM_RESOURCE_DIR "/Users/licm/Development/BiAIT/EnvAndModel"

int main(int argc, char **argv) {
  app::SE2RigidBodyPlanning setup;
  std::string benchmark_name;

  setup.getSpaceInformation()->setStateValidityCheckingResolution(0.0001);

  benchmark_name = std::string("BugTrap_planar_env");
  std::string env_fname = std::string(OMPLAPP_CUSTOM_RESOURCE_DIR) + "/2D/" +
                          "BugTrap_planar_old_env.dae";
  std::string robot_fname = std::string(OMPLAPP_CUSTOM_RESOURCE_DIR) + "/2D/" +
                            "car1_planar_robot.dae";
  setup.setRobotMesh(robot_fname);
  setup.setEnvironmentMesh(env_fname);

  base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
  start->setX(0.0);
  start->setY(0.0);
  start->setYaw(0.0);

  base::ScopedState<base::SE2StateSpace> goal(start);
  goal->setX(-30.0);
  goal->setY(0.0);
  goal->setYaw(1.57);

  setup.setStartAndGoalStates(start, goal);
  setup.getSpaceInformation()->setStateValidityCheckingResolution(
      stateValidityCheckingResolution);

  ompl::base::OptimizationObjectivePtr optimizationObjective_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(
          setup.getSpaceInformation());
  //    optimizationObjective_->setCostThreshold(ompl::base::Cost(144.0));
  optimizationObjective_->setCostThreshold(
      optimizationObjective_->infiniteCost());
  setup.setOptimizationObjective(optimizationObjective_);

  setup.setup();

  // If using the ompl::base::PlannerPtr, we can not use the visualPtr since it
  // is non-virtual;
  auto BiAITPtr =
      std::make_shared<ompl::geometric::BiAIT>(setup.getSpaceInformation());
  BiAITPtr->params().setParam("batch_size", std::to_string(1000));
  BiAITPtr->params().setParam("rewire_factor", std::to_string(1.0));
  BiAITPtr->params().setParam("enable_meet_in_the_middle", std::to_string(1));
  BiAITPtr->params().setParam("enable_brute_force", std::to_string(0));
  BiAITPtr->params().setParam("enable_DFS_only", std::to_string(0));
  setup.setPlanner(BiAITPtr);
  setup.print();

  ompl::base::PlannerStatus isSolved = setup.solve(60.0);
  if (isSolved) {
    // get the goal representation from the problem definition (not the same as
    // the goal state) and inquire about the found path
    ompl::base::PathPtr path = setup.getProblemDefinition()->getSolutionPath();
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    path->print(std::cout);
  } else
    std::cout << "No solution found" << std::endl;
  return 0;
}