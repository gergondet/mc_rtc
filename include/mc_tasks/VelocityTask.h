/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/FrameVelocity.h>

namespace mc_tasks
{

/* \brief Control the velocity (6D) of a frame */
struct MC_TASKS_DLLAPI VelocityTask : public TrajectoryTaskGeneric<mc_tvm::FrameVelocity>
{
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::FrameVelocity>;

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
  VelocityTask(mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 500.0);

  /** Get the frame controlled by this task */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return errorT_->frame();
  }

  /** Get the frame controlled by this task (non-const) */
  inline mc_rbdyn::RobotFrame & frame() noexcept
  {
    return errorT_->frame();
  }
};

using VelocityTaskPtr = std::shared_ptr<VelocityTask>;

} // namespace mc_tasks
