/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

namespace mc_tvm
{

/** This class implements an orientation function for a given frame */
class MC_TVM_DLLAPI OrientationFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(OrientationFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * Set the objective to the current frame orientation
   *
   */
  OrientationFunction(const mc_rbdyn::Frame & frame);

  /** Set the target orientation to the current frame orientation */
  void reset();

  /** Get the current objective */
  inline const Eigen::Matrix3d & orientation() const noexcept
  {
    return ori_;
  }

  /** Set the objective */
  inline void orientation(const Eigen::Matrix3d & ori) noexcept
  {
    ori_ = ori;
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
  Eigen::Matrix3d ori_;
  Eigen::Vector3d refVel_;
  Eigen::Vector3d refAccel_;
};

} // namespace mc_tvm
