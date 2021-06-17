/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Frame.h>

#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/Robot.h>

namespace mc_rbdyn
{

/** A frame that belongs to a Robot instance.
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
struct MC_RBDYN_DLLAPI RobotFrame : public Frame, mc_rtc::shared<RobotFrame, false>
{
  SET_OUTPUTS(RobotFrame, Jacobian, NormalAcceleration, JDot)
  SET_UPDATES(RobotFrame, Jacobian, NormalAcceleration, JDot)

  friend struct Robot;

protected:
  struct ctor_token
  {
  };

public:
  /** Constructor
   *
   * Creates a frame belonging to a robot and directly associated to a body
   *
   * \param name Name of the frame
   *
   * \param robot Robot to which the frame is attached
   *
   * \param body Parent body of the frame
   */
  RobotFrame(ctor_token, std::string_view name, Robot & robot, std::string_view body);

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
  RobotFrame(ctor_token, std::string_view name, RobotFrame & frame, sva::PTransformd X_f1_f2);

  RobotFrame(const RobotFrame &) = delete;
  RobotFrame & operator=(const RobotFrame &) = delete;

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
  inline const std::string & name() const noexcept
  {
    return name_;
  }

  /** The robot to which this frame belongs */
  inline const Robot & robot() const noexcept
  {
    return robot_;
  }

  /** The robot to which this frame belongs */
  inline Robot & robot() noexcept
  {
    return robot_;
  }

  /** Transformation from the frame to its parent body */
  inline sva::PTransformd X_b_f() const noexcept
  {
    return X_r_f_;
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

  inline bool operator==(const RobotFrame & rhs) const noexcept
  {
    return rhs.name_ == name_ && &rhs.robot_ == &robot_;
  }

  /** True if the frame has a force sensor (direct or indirect) attached */
  inline bool hasForceSensor() const noexcept
  {
    return robot_.frameHasForceSensor(name_) || robot_.frameHasIndirectForceSensor(name_);
  }

  /** Returns the force sensor attached to the frame
   *
   * \throws if \ref hasForceSensor() is false
   */
  const ForceSensor & forceSensor() const
  {
    if(robot_.frameHasForceSensor(name_))
    {
      return robot_.frameForceSensor(name_);
    }
    return robot_.findFrameForceSensor(name_);
  }

  /** Returns the force sensor gravity-free wrench in this frame
   *
   * \throws if \ref hasForceSensor() is false
   */
  inline sva::ForceVecd wrench() const
  {
    return robot_.frameWrench(name_);
  }

  /** Create a frame whose parent is this frame
   *
   * \param name Name of the new frame
   *
   * \param X_p_f Transformation from this frame to the newly created frame
   *
   * \param baked Attach the newly created frame to the parent body of this frame rather than the frame
   */
  FramePtr makeFrame(std::string_view name, const sva::PTransformd & X_p_f, bool baked = false) override
  {
    return robot_.makeFrame(name, *this, X_p_f, baked).shared_from_this();
  }

  /** Return true only if this frame is a RobotFrame.*/
  virtual bool isRobotFrame() const noexcept override
  {
    return true;
  }

private:
  Robot & robot_;
  unsigned int bodyId_;
  rbd::Jacobian jac_;

  Eigen::MatrixXd jacTmp_;

  void updatePosition();

  tvm::internal::MatrixWithProperties jacobian_;
  void updateJacobian();

  void updateVelocity();

  sva::MotionVecd normalAcceleration_;
  void updateNormalAcceleration();

  tvm::internal::MatrixWithProperties jacDot_;
  void updateJDot();
};

} // namespace mc_rbdyn
