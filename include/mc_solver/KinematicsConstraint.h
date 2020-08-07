/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/Constraint.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/ControlProblem.h>

namespace mc_solver
{

/** \class KinematicsConstraint
 *
 * Takes care of kinematics constraint for a robot, that is:
 * - joint limits
 * - joint velocity limits
 *
 */
struct MC_SOLVER_DLLAPI KinematicsConstraint : public Constraint
{
public:
  /** Constructor
   *
   * \param robot The robot affected by this constraint
   *
   * \param robotIndex The index of the robot affected by this constraint
   *
   * \param damper Value of the damper {interaction distance, safety distance, offset}
   *
   * \param velocityPercent Maximum joint velocity percentage, 0.5 is advised
   */
  KinematicsConstraint(mc_rbdyn::Robot & robot, const std::array<double, 3> & damper, double velocityPercent = 0.5);

  void addToSolver(QPSolver & solver) override;

  void removeFromSolver(QPSolver & solver) override;

private:
  mc_rbdyn::RobotPtr robot_;
  std::array<double, 3> damper_;
  double velocityPercent_;

  std::vector<tvm::TaskWithRequirementsPtr> constraints_;
};

} // namespace mc_solver
