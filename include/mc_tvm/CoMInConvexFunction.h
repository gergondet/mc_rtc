/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>
#include <mc_rbdyn/polygon_utils.h>

#include <tvm/function/abstract/Function.h>
#include <tvm/geometry/Plane.h>

namespace mc_tvm
{

/** This function computes the distance of the CoM to a set of planes.
 *
 * By providing a consistent set of planes, the function can be used to keep
 * the CoM in a convex region of space.
 */
struct MC_TVM_DLLAPI CoMInConvexFunction : public tvm::function::abstract::Function
{
public:
  using Output = tvm::function::abstract::Function::Output;
  DISABLE_OUTPUTS(Output::JDot)
  SET_UPDATES(CoMInConvexFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * By default, this function computes nothing
   *
   */
  CoMInConvexFunction(mc_rbdyn::RobotPtr robot);

  /** Add a plane.
   *
   * This will add one dimension to the function output. This new value is
   * the distance to that plane.
   *
   * This function does not check whether this is consistent with planes that
   * were added previously.
   */
  void addPlane(tvm::geometry::PlanePtr plane);

  /** Remove all planes */
  void reset();

protected:
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  mc_rbdyn::CoMPtr com_;

  /** Set of planes */
  std::vector<tvm::geometry::PlanePtr> planes_;
};

} // namespace mc_tvm
