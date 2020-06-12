/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <tvm/graph/abstract/Node.h>

namespace mc_rbdyn
{

/** Center of mass (CoM) of a Robot and related quantities
 *
 * Provides the frame position, jacobian, velocity and normal acceleration.
 * These signals are correctly initialized on the object's creation.
 *
 * Outputs:
 * - Position: position of the CoM in world coordinates
 * - Jacobian: jacobian of the CoM in world coordinates
 * - Velocity: velocity of the CoM in world coordinates
 * - NormalAcceleration: normal acceleration of the CoM in world coordinates
 * - JDot: derivative of the jacobian of the CoM in world coordinates
 *
 */
struct MC_RBDYN_DLLAPI CoM : public tvm::graph::abstract::Node<CoM>
{
  SET_OUTPUTS(CoM, Position, Jacobian, Velocity, NormalAcceleration, JDot)
  SET_UPDATES(CoM, Position, Jacobian, Velocity, NormalAcceleration, JDot)

  friend struct Robot;

private:
  struct ctor_token
  {
  };

public:
  /** Constructor
   *
   * Creates a frame belonging to a robot
   *
   * \param name Name of the frame
   *
   * \param robot Robot to which the frame is attached
   *
   * \param body Parent body of the frame
   *
   * \param X_b_f Static transformation from the body to the frame
   */
  CoM(ctor_token, std::string_view name, RobotPtr robot, std::string_view body, sva::PTransformd X_b_f);
};

} // namespace mc_rbdyn

