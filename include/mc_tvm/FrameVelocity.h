/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <RBDyn/Jacobian.h>

namespace mc_tvm
{

// FIXME This actually ignores the frame orientation and works with body orientation

/** This class implements a function to compute the local velocity of a given frame */
struct MC_TVM_DLLAPI FrameVelocity : public tvm::function::abstract::Function
{
  SET_UPDATES(FrameVelocity, Value, Velocity, Jacobian)

  /** Constructor
   *
   * Creates a FrameVelocity function, values are correctly initialized
   */
  FrameVelocity(mc_rbdyn::Frame & frame, const Eigen::Vector6d & dof);

  /** Empty reset function in case we want to wrap this as a task later */
  inline void reset() {}

  /** Access the related frame */
  inline const mc_rbdyn::Frame & frame() const noexcept
  {
    return *frame_;
  }

  /** Get the dof vector */
  inline const Eigen::Vector6d & dof() const noexcept
  {
    return dof_;
  }
  /** Get the dof vector */
  inline Eigen::Vector6d & dof() noexcept
  {
    return dof_;
  }

private:
  mc_rbdyn::FramePtr frame_;
  Eigen::Vector6d dof_;
  rbd::Jacobian jac_;
  Eigen::MatrixXd jacobian_;

  void updateValue();
  void updateVelocity();
  void updateJacobian();
};

using FrameVelocityPtr = std::shared_ptr<FrameVelocity>;

} // namespace mc_tvm
