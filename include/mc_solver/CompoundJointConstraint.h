/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/Constraint.h>

#include <mc_tvm/CompoundJointFunction.h>

#include <tvm/ControlProblem.h>

namespace mc_solver
{

using CompoundJointConstraintDescription = mc_rbdyn::CompoundJointConstraintDescription;
using CompoundJointConstraintDescriptionVector = mc_rbdyn::CompoundJointConstraintDescriptionVector;

/** \class CompoundJointConstraint
 *
 * For a robot implement a compound joint constraint for every description
 * provided in the associated RobotModule
 */
struct MC_SOLVER_DLLAPI CompoundJointConstraint : public Constraint
{
  CompoundJointConstraint(mc_rbdyn::Robot & robot, double dt);

  void addToSolver(QPSolver & solver) override;

  void removeFromSolver(QPSolver & solver) override;

private:
  std::vector<mc_tvm::CompoundJointFunctionPtr> functions_;
  std::vector<tvm::TaskWithRequirementsPtr> constraints_;
};

} // namespace mc_solver
