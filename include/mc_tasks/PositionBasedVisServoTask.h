/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/PositionBasedVisServoFunction.h>

namespace mc_tasks
{
/*! \brief Servo an end-effector depending on position error */
struct MC_TASKS_DLLAPI PositionBasedVisServoTask : public TrajectoryTaskGeneric<mc_tvm::PositionBasedVisServoFunction>
{
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::PositionBasedVisServoFunction>;

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
  PositionBasedVisServoTask(mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 500);

  /*! \brief Set the current error
   *
   * \param X_t_f Transformation from the target to the control frame
   *
   */
  inline void error(const sva::PTransformd & X_t_f) noexcept
  {
    errorT_->error(X_t_f);
  }

  /*! \brief Access the current error */
  inline const sva::PTransformd & error() const noexcept
  {
    return errorT_->error();
  }

  /*! \brief Access the controlled frame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return errorT_->frame();
  }

  void addToLogger(mc_rtc::Logger & logger) override;
};
} // namespace mc_tasks
