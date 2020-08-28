/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/VectorOrientationTask.h>

namespace mc_tasks
{

/*! \brief Orient a "gaze" vector defined on a body to look towards a world
 * position. This task is a convenience wrapper around VectorOrientationTask
 */
struct MC_TASKS_DLLAPI LookAtTask : public VectorOrientationTask
{
  /*! \brief Constructor with user-specified target initialization
   *
   * \param frame Frame controlled by this task
   *
   * \param frameVector Gaze vector for the frame. For instance [1., 0, 0] will
   * try to align the x axis of the frame with the target direction.
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  LookAtTask(mc_rbdyn::Frame & frame, const Eigen::Vector3d & frameVector, double stiffness = 2.0, double weight = 500);

  /** Reset the task */
  void reset() override;

  /**
   * \brief Look towards the target frame position
   *
   * \param pos Target position in world frame
   */
  void target(const Eigen::Vector3d & target_);

  /**
   * @brief Gets the target frame position
   * See targetVector() to obtain the gaze vector
   *
   * @return Target vector in world frame
   */
  inline const Eigen::Vector3d & target() const
  {
    return target_;
  }

private:
  void addToLogger(mc_rtc::Logger & logger) override;
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

private:
  /*! Target position in world frame */
  Eigen::Vector3d target_;
};

using LookAtTaskPtr = std::shared_ptr<LookAtTask>;

} // namespace mc_tasks
