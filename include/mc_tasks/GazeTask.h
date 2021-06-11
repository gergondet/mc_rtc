/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/GazeFunction.h>

namespace mc_tasks
{

/*! \brief Control the gaze of a body
 *
 *
 */
struct MC_TASKS_DLLAPI GazeTask : public TrajectoryTaskGeneric<mc_tvm::GazeFunction>
{
public:
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::GazeFunction>;

  /*! \brief Constructor
   *
   * \param frame Camera frame used by this gaze task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  GazeTask(mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 500);

  /*! \brief Set the current error
   *
   * \param point2d Target position in camera frame
   *
   * \param point2d_ref Desired position of the point in image frame
   *
   */
  inline void error(const Eigen::Vector2d & point2d,
                    const Eigen::Vector2d & point2d_ref = Eigen::Vector2d::Zero()) noexcept
  {
    errorT_->estimate(point2d);
    errorT_->target(point2d_ref);
  }

  /*! \brief Set the current error
   *
   * \param point3d Target position in camera frame
   *
   * \param point2d_ref Desired position of the point in image frame
   *
   */
  inline void error(const Eigen::Vector3d & point3d,
                    const Eigen::Vector2d & point2d_ref = Eigen::Vector2d::Zero()) noexcept
  {
    errorT_->estimate(point3d);
    errorT_->target(point2d_ref);
  }

  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return errorT_->frame();
  }
};

} // namespace mc_tasks
