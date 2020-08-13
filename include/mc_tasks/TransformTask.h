/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/TransformFunction.h>

namespace mc_tasks
{

/*! \brief Control the (6D) position of a frame */
struct MC_TASKS_DLLAPI TransformTask : public TrajectoryTaskGeneric<mc_tvm::TransformFunction>
{
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::TransformFunction>;

public:
  /*! \brief Constructor
   *
   * \param frame Frame to control
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  TransformTask(mc_rbdyn::Frame & frame, double stiffness = 2.0, double weight = 500.0);

  /*! \brief Get the frame's target */
  inline const sva::PTransformd & target() const noexcept
  {
    return errorT_->pose();
  }

  /*! \brief Set the frame's target
   *
   * \param pos Target position in inertial frame
   *
   */
  inline void target(const sva::PTransformd & pos) noexcept
  {
    errorT_->pose(pos);
  }

  /*! \brief Target another frame with a given offset
   *
   * \param frame Target frame
   *
   * \param offset Offset relative to the target frame
   */
  void target(const mc_rbdyn::Frame & frame, const sva::PTransformd & offset);

  void addToLogger(mc_rtc::Logger & logger) override;

  using TrajectoryBase::damping;
  using TrajectoryBase::setGains;
  using TrajectoryBase::stiffness;

  /*! \brief Set dimensional stiffness and damping
   *
   * \param stiffness Dimensional stiffness
   *
   * \param damping Dimensional damping
   *
   */
  inline void setGains(const sva::MotionVecd & stiffness, const sva::MotionVecd & damping)
  {
    return TrajectoryBase::setGains(stiffness.vector(), damping.vector());
  }

  /*! \brief Set dimensional stiffness
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Dimensional stiffness as a motion vector
   *
   */
  void stiffness(const sva::MotionVecd & stiffness)
  {
    return TrajectoryBase::stiffness(stiffness.vector());
  }

  /*! \brief Get dimensional stiffness as a motion vector */
  inline sva::MotionVecd mvStiffness() noexcept
  {
    return sva::MotionVecd(dimStiffness());
  }

  /*! \brief Set dimensional damping
   *
   * \param damping Dimensional damping as a motion vector
   *
   */
  inline void damping(const sva::MotionVecd & damping)
  {
    return TrajectoryBase::damping(damping.vector());
  }

  /*! \brief Get dimensional damping as a motion vector */
  sva::MotionVecd mvDamping()
  {
    return sva::MotionVecd(dimDamping());
  }

  /*! \brief Set trajectory task's reference velocity from motion vector in
   * body coordinates.
   *
   * \param velB Reference velocity in body coordinates, i.e. velocity of the frame in the frame.
   *
   */
  void refVelB(const sva::MotionVecd & velB)
  {
    TrajectoryBase::refVel(velB.vector());
  }

  /*! \brief Get reference velocity in body coordinates as a motion vector */
  sva::MotionVecd refVelB() const
  {
    return sva::MotionVecd(TrajectoryBase::refVel());
  }

  /*! \brief Set trajectory task's reference acceleration from motion vector.
   *
   * \param acc Reference acceleration.
   *
   */
  void refAccel(const sva::MotionVecd & accel)
  {
    return TrajectoryBase::refAccel(accel.vector());
  }

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  /* Don't use parent's refVel() as the velocity frame (spatial or body) is ambiguous. */
  using TrajectoryBase::refVel;
};

} // namespace mc_tasks
