/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

namespace mc_tvm
{

/** This class implements a position function for a given frame */
class MC_TVM_DLLAPI PositionFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(PositionFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * Set the objective to the current frame position
   *
   */
  PositionFunction(const mc_rbdyn::Frame & frame);

  /** Set the target position to the current frame position */
  void reset();

  /** Get the current objective */
  inline const Eigen::Vector3d & position() const noexcept
  {
    return pos_;
  }

  /** Set the objective */
  inline void position(const Eigen::Vector3d & pos) noexcept
  {
    pos_ = pos;
  }

  /** Get the current objective */
  inline const Eigen::Vector3d & refVel() const noexcept
  {
    return refVel_;
  }

  /** Set the objective */
  inline void refVel(const Eigen::Vector3d & refVel) noexcept
  {
    refVel_ = refVel;
  }

  /** Get the current objective */
  inline const Eigen::Vector3d & refAccel() const noexcept
  {
    return refAccel_;
  }

  /** Set the objective */
  inline void refAccel(const Eigen::Vector3d & refAccel) noexcept
  {
    refAccel_ = refAccel;
  }

protected:
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  mc_rbdyn::ConstFramePtr frame_;

  /** Target */
  Eigen::Vector3d pos_;
  Eigen::Vector3d refVel_;
  Eigen::Vector3d refAccel_;
};

} // namespace mc_tvm
