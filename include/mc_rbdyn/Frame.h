/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <mc_rtc/shared.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <RBDyn/Jacobian.h>

#include <tvm/graph/abstract/Node.h>
#include <tvm/internal/MatrixWithProperties.h>

namespace mc_rbdyn
{

/** A frame belongs to a Robot instance.
 *
 * Provides the frame position, jacobian, velocity and normal acceleration.
 * These signals are correctly initialized on the object's creation.
 *
 * Outputs:
 * - Position: position of the frame in world coordinates
 * - Jacobian: jacobian of the frame in world coordinates
 * - Velocity: velocity of the frame in world coordinates
 * - NormalAcceleration: normal acceleration of the frame in world coordinates
 * - JDot: derivative of the jacobian of the frame in world coordinate
 *
 */
struct MC_RBDYN_DLLAPI Frame : public tvm::graph::abstract::Node<Frame>, mc_rtc::shared<Frame>
{
  SET_OUTPUTS(Frame, Position, Jacobian, Velocity, NormalAcceleration, JDot)
  SET_UPDATES(Frame, Position, Jacobian, Velocity, NormalAcceleration, JDot)

  friend struct Robot;

protected:
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
  Frame(ctor_token, std::string_view name, RobotPtr robot, std::string_view body, sva::PTransformd X_b_f);

  /** Constructor
   *
   * Creates a frame from an existing frame
   *
   * \param name Name of the new frame
   *
   * \param frame Reference frame
   *
   * \param X_f1_f2 Static transformation from the original frame to the new one
   */
  Frame(ctor_token, std::string_view name, const Frame & frame, sva::PTransformd X_f1_f2);

  /** Get a copy of the internal RBDyn Jacobian object
   *
   * This is useful if your entity requires:
   * - the body jacobian
   * - the jacobian translated at varying position
   */
  inline rbd::Jacobian rbdJacobian() const noexcept
  {
    return jac_;
  }

  /** The frame's name */
  inline std::string_view name() const noexcept
  {
    return name_;
  }

  /** Frame static transformation from its parent body to the frame */
  inline const sva::PTransformd & X_b_f() const noexcept
  {
    return X_b_f_;
  }

  /** The robot to which this frame belongs */
  inline const Robot & robot() const noexcept
  {
    return *robot_;
  }

  /** The robot to which this frame belongs */
  inline Robot & robot() noexcept
  {
    return *robot_;
  }

  /** The frame's position in world coordinates */
  inline const sva::PTransformd & position() const noexcept
  {
    return position_;
  }

  inline const tvm::internal::MatrixWithProperties & jacobian() const noexcept
  {
    return jacobian_;
  }

  inline const sva::MotionVecd & velocity() const noexcept
  {
    return velocity_;
  }

  inline const sva::MotionVecd & normalAcceleration() const noexcept
  {
    return normalAcceleration_;
  }

  inline const tvm::internal::MatrixWithProperties & JDot() const noexcept
  {
    return jacDot_;
  }

  const std::string & body() const noexcept;

private:
  std::string name_;
  RobotPtr robot_;
  unsigned int bodyId_;
  rbd::Jacobian jac_;
  sva::PTransformd X_b_f_;

  Eigen::Matrix3d h_;
  Eigen::MatrixXd jacTmp_;

  sva::PTransformd position_;
  void updatePosition();

  tvm::internal::MatrixWithProperties jacobian_;
  void updateJacobian();

  sva::MotionVecd velocity_;
  void updateVelocity();

  sva::MotionVecd normalAcceleration_;
  void updateNormalAcceleration();

  tvm::internal::MatrixWithProperties jacDot_;
  void updateJDot();

  void updateAll();
};

} // namespace mc_rbdyn
