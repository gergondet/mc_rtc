/* Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <RBDyn/Jacobian.h>

namespace mc_tvm
{

/** Represents the error between two frame positions.
 *
 * Given two frames (f1, f2) belonging to (r1, r2) this is the difference
 * between their current relative position.
 * This is a function of r1.q and r2.q.
 *
 * If r1 or r2 is not actuated, then it is not taken into account in
 * computations. If neither are actuated this is a constant function.
 *
 * The degrees of freedom computed by this function can be specified through a
 * 6x6 dof matrix.
 *
 * Outputs:
 *
 * - Value: dof*transformError(X_f1, X_f2)
 * - Velocity
 * - NormalAcceleration
 * - Jacobian
 *
 */
class MC_TVM_DLLAPI FrameErrorFunction : public tvm::function::abstract::Function
{
public:
  using Output = tvm::function::abstract::Function::Output;
  DISABLE_OUTPUTS(Output::JDot)
  SET_UPDATES(FrameErrorFunction, Value, Jacobian, Velocity, NormalAcceleration)

  /** Constructor
   *
   * \param f1 First contact frame
   *
   * \param f2 Second contact frame
   *
   * \param dof Contact dof
   *
   */
  FrameErrorFunction(mc_rbdyn::FramePtr f1,
                     mc_rbdyn::FramePtr f2,
                     const Eigen::Vector6d & dof = Eigen::Vector6d::Ones());

  /** Access the contact dof vector */
  inline const Eigen::Vector6d & dof() const noexcept
  {
    return dof_;
  }

  /** Set the contact dof vector */
  inline void dof(const Eigen::Vector6d & dof) noexcept
  {
    dof_ = dof;
  }

private:
  mc_rbdyn::FramePtr f1_;
  mc_rbdyn::FramePtr f2_;
  Eigen::Vector6d dof_;

  bool use_f1_ = false;
  rbd::Jacobian f1Jacobian_;
  bool use_f2_ = false;
  rbd::Jacobian f2Jacobian_;
  bool sameVariable_ = false;

  Eigen::Matrix3d dLog1_;
  Eigen::Matrix3d dLog2_;

  void updateValue();
  void updateJacobian();
  void updateVelocity();
  void updateNormalAcceleration();
};

using ContactFunctionPtr = std::shared_ptr<mc_tvm::FrameErrorFunction>;

} // namespace mc_tvm
