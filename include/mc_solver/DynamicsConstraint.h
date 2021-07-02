/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/KinematicsConstraint.h>

#include <mc_tvm/DynamicFunction.h>

namespace mc_solver
{

/** \class DynamicsConstraint
 *
 * Takes care of kinematic and dynamic constraints for a robot. In addition to
 * the constraints imposed by KinematicConstraint, this also ensure torque
 * limits via the equation of motion.
 *
 * \see KinematicsConstraint
 */
struct MC_SOLVER_DLLAPI DynamicsConstraint : public KinematicsConstraint
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
  DynamicsConstraint(mc_rbdyn::Robot & robot,
                     const std::array<double, 3> & damper = {0.1, 0.01, 0.5},
                     double velocityPercent = 0.5);

  /** Implementation of mc_solver::ConstraintSet::addToSolver */
  void addToSolver(QPSolver & solver) override;

  /** Implementation of mc_solver::ConstraintSet::removeFromSolver */
  void removeFromSolver(QPSolver & solver) override;

  inline mc_tvm::DynamicFunction & dynamic() noexcept
  {
    return *dynamic_;
  }

protected:
  mc_tvm::DynamicFunctionPtr dynamic_;
};

using DynamicsConstraintPtr = std::shared_ptr<DynamicsConstraint>;

} // namespace mc_solver
