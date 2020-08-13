/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_tvm
{

/** This class implements a position function for a given frame */
class MC_TVM_DLLAPI TransformFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(TransformFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * Set the objective to the current frame pose
   *
   */
  TransformFunction(mc_rbdyn::Frame & frame);

  /** Set the target pose to the current frame pose */
  void reset();

  /** Get the current objective */
  inline const sva::PTransformd & pose() const noexcept
  {
    return pose_;
  }

  /** Set the objective */
  inline void pose(const sva::PTransformd & pose) noexcept
  {
    pose_ = pose;
  }

  /** Get the current objective */
  inline const Eigen::Vector6d & refVel() const noexcept
  {
    return refVel_;
  }

  /** Set the objective */
  inline void refVel(const Eigen::Vector6d & refVel) noexcept
  {
    refVel_ = refVel;
  }

  /** Get the current objective */
  inline const Eigen::Vector6d & refAccel() const noexcept
  {
    return refAccel_;
  }

  /** Set the objective */
  inline void refAccel(const Eigen::Vector6d & refAccel) noexcept
  {
    refAccel_ = refAccel;
  }

  /** Get the frame */
  inline const mc_rbdyn::Frame & frame() const noexcept
  {
    return *frame_;
  }

protected:
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  mc_rbdyn::FramePtr frame_;

  /** Target */
  sva::PTransformd pose_;
  Eigen::Vector6d refVel_;
  Eigen::Vector6d refAccel_;
};

} // namespace mc_tvm
