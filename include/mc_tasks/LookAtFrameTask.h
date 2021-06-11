/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/LookAtTask.h>

namespace mc_tasks
{

/*! \brief Track a frame position with a "gaze" vector.
 * This task is a convenience wrapper for LookAtTask that takes care of
 * automatically updating the gaze target.
 */
struct MC_TASKS_DLLAPI LookAtFrameTask : public LookAtTask
{
public:
  /*! \brief Constructor
   *
   * \param frame Frame to be controlled
   *
   * \param frameVector Gaze vector for the frame. For instance [1., 0, 0] will try to align the x axis of the frame
   * with the target direction
   *
   * \param targetFrame Frame that will be tracked
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  LookAtFrameTask(mc_rbdyn::RobotFrame & frame,
                  const Eigen::Vector3d & frameVector,
                  mc_rbdyn::FreeFrame & targetFrame,
                  double stiffness = 0.5,
                  double weight = 200);

  /*! \brief Update the gaze target from the frame position */
  void update(mc_solver::QPSolver &) override;

  /*! \brief Access offset relative to the surface */
  inline const sva::PTransformd & offset() const noexcept
  {
    return offset_;
  }

  /*! \brief Set the offset relative to the surface */
  inline void offset(const sva::PTransformd & off) noexcept
  {
    offset_ = off;
  }

private:
  /** Target frame */
  mc_rbdyn::ConstFreeFramePtr targetFrame_;
  /** Offset to the surface in surface frame */
  sva::PTransformd offset_;
};

using LookAtFrameTaskPtr = std::shared_ptr<LookAtFrameTask>;

} // namespace mc_tasks
