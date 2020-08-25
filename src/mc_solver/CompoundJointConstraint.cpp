/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CompoundJointConstraint.h>

#include <mc_solver/ConstraintLoader.h>

namespace mc_solver
{

CompoundJointConstraint::CompoundJointConstraint(mc_rbdyn::Robot & robot, double dt)
{
  functions_.reserve(robot.module().compoundJoints().size());
  constraints_.reserve(functions_.size());
  for(auto & cstr : robot.module().compoundJoints())
  {
    functions_.emplace_back(std::make_shared<mc_tvm::CompoundJointFunction>(robot, cstr, dt));
  }
}

void CompoundJointConstraint::addToSolver(QPSolver & solver)
{
  if(!constraints_.size())
  {
    for(const auto & f : functions_)
    {
      constraints_.push_back(solver.problem().add(f <= 0.));
    }
  }
}

void CompoundJointConstraint::removeFromSolver(QPSolver & solver)
{
  for(const auto & c : constraints_)
  {
    solver.problem().remove(c.get());
  }
  constraints_.clear();
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintLoader::register_load_function(
    "compoundJoint",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      return std::make_shared<mc_solver::CompoundJointConstraint>(
          solver.robots().fromConfig(config, "CompoundJointConstraint"), solver.dt());
    });
}
