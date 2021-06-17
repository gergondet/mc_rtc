/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/shared.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <RBDyn/Jacobian.h>

#include <tvm/graph/abstract/Node.h>
#include <tvm/internal/MatrixWithProperties.h>

namespace mc_rbdyn
{

/** Represent a frame
 *
 * This frame is not associated to any variable. It's up to the user to update
 * the frame position/velocity/acceleration
 *
 * Outputs:
 * - Position: position of the frame in world coordinates
 * - Velocity: velocity of the frame in world coordinates
 *
 */

struct MC_RBDYN_DLLAPI Frame : public tvm::graph::abstract::Node<Frame>, mc_rtc::shared<Frame>
{
  SET_OUTPUTS(Frame, Position, Velocity)
  SET_UPDATES(Frame, Position, Velocity)

public:
  /** Constructor
   *
   * Creates a frame
   *
   * \param name Name of the frame
   *
   * \param X_0_f Position of the frame
   *
   * \param vel_0_f Velocity of the frame
   */
  Frame(std::string_view name,
        const sva::PTransformd & X_0_f = sva::PTransformd::Identity(),
        const sva::MotionVecd & vel_0_f = sva::MotionVecd::Zero());

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
  Frame(std::string_view name, Frame & frame, const sva::PTransformd & X_f1_f2);

  Frame(const Frame &) = delete;
  Frame & operator=(const Frame &) = delete;

  /** The frame's name */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

  /** The frame's position in world coordinates */
  inline const sva::PTransformd & position() const noexcept
  {
    return position_;
  }

  /** Update the frame's position, throws if the frame is defined relative to another frame */
  inline void position(const sva::PTransformd & pos)
  {
    if(parent_)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Frame {} is defined in reference to {}, its absolute position cannot be set!", name(), parent_->name());
    }
    position_ = pos;
  }

  /** The frame's velocity in world coordinates */
  inline const sva::MotionVecd & velocity() const noexcept
  {
    return velocity_;
  }

  /** Update the frame's velocity, throws if the frame is defined relative to another frame */
  inline void velocity(const sva::MotionVecd & velocity)
  {
    if(parent_)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Frame {} is defined in reference to {}, its velocity cannot be set!", name(), parent_->name());
    }
    velocity_ = velocity;
  }

  /** The frame's parent frame, can be nullptr */
  inline const FramePtr & parent() const noexcept
  {
    return parent_;
  }

  /** Transformation from the parent's frame to this frame */
  inline const sva::PTransformd & X_p_f() const noexcept
  {
    return X_p_f_;
  }

  /** Set the transformation from the parent's frame to this frame
   *
   * \throws If this frame has no parent frame
   */
  inline void X_p_f(const sva::PTransformd & X_p_f)
  {
    if(!parent_)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Frame {} has no parent, its relative position cannot be set.",
                                                       name());
    }
    X_p_f_ = X_p_f;
  }

  /** Create a frame whose parent is this frame
   *
   * \param name Name of the new frame
   *
   * \param X_p_f Transformation from this frame to the newly created frame
   *
   * \param baked Attach the newly created frame to the parent body of this frame rather than the frame
   */
  virtual FramePtr makeFrame(std::string_view name, const sva::PTransformd & X_p_f, bool baked = false)
  {
    if(baked && parent_)
    {
      return std::make_shared<Frame>(name, *parent_, X_p_f * parent_->X_p_f_);
    }
    else
    {
      return std::make_shared<Frame>(name, *this, X_p_f);
    }
  }

  /** Return true only if this frame is a RobotFrame.*/
  virtual bool isRobotFrame() const noexcept
  {
    return false;
  }

protected:
  Frame(FramePtr parent,
        std::string_view name,
        const sva::PTransformd & X_p_f,
        const sva::PTransformd & pos = sva::PTransformd::Identity(),
        const sva::MotionVecd & vel = sva::MotionVecd::Zero());

  FramePtr parent_;
  FramePtr root_;

  sva::PTransformd X_p_f_;
  sva::PTransformd X_r_f_ = sva::PTransformd::Identity();

  std::string name_;

  Eigen::Matrix3d h_ = Eigen::Matrix3d::Zero();

  sva::PTransformd position_;

  sva::MotionVecd velocity_;

protected:
  void updatePosition();
  void updateVelocity();
};

} // namespace mc_rbdyn
