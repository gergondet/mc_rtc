/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/PositionFunction.h>

namespace mc_tasks
{

/*! \brief Control the position of a frame */
struct MC_TASKS_DLLAPI PositionTask : public TrajectoryTaskGeneric<mc_tvm::PositionFunction>
{
public:
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::PositionFunction>;

  /*! \brief Constructor
   *
   * \param frame Frame to control
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  PositionTask(mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 500);

  virtual ~PositionTask() = default;

  /*! \brief Get the body position target */
  inline const Eigen::Vector3d & position() noexcept
  {
    return errorT_->position();
  }

  /*! \brief Set the body position target
   *
   * \param pos Body position in world frame
   *
   */
  inline void position(const Eigen::Vector3d & pos) noexcept
  {
    errorT_->position(pos);
  }

  /** Get the frame controlled by this task */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return errorT_->frame();
  }

protected:
  void addToLogger(mc_rtc::Logger & logger) override;
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
};

} // namespace mc_tasks
