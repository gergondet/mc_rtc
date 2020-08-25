/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/Constraint.h>

#include <mc_tvm/CoMInConvexFunction.h>

#include <tvm/ControlProblem.h>
#include <tvm/task_dynamics/VelocityDamper.h>

namespace mc_solver
{

/** \class CoMInConvexConstraint
 *
 *  For every plane provided to this constraint, this ensures that the robot's
 *  CoM remains on the positive side of this plane.
 *
 *  If this constraint is given a consistent set of planes this can ensure the
 *  CoM remains in a convex.
 *
 */
struct MC_SOLVER_DLLAPI CoMInConvexConstraint : public Constraint
{
public:
  CoMInConvexConstraint(mc_rbdyn::Robot & robot);

  void addToSolver(QPSolver & solver) override;

  void removeFromSolver(QPSolver & solver) override;

  /** Set the planes that the CoM will remain above */
  void setPlanes(QPSolver & solver,
                 const std::vector<tvm::geometry::PlanePtr> & planes,
                 const std::optional<tvm::task_dynamics::VelocityDamper::Config> & config = std::nullopt);

private:
  mc_tvm::CoMInConvexFunctionPtr function_;
  tvm::task_dynamics::VelocityDamper::Config config_{0.05, 0.01, 0.1, 0.0};
  tvm::TaskWithRequirementsPtr constraint_;
};

} // namespace mc_solver
