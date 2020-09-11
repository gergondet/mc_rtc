/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/SplineTrajectoryTask.h>

#include <mc_trajectory/ExactCubic.h>

namespace mc_tasks
{

/*! \brief Track an exact cubic spline, that is a curve passing exactly through
 * waypoints in position, with optional initial and final velocity and
 * acceleration.
 *
 * SplineTrajectoryTask takes care of much of the logic (task target updates,
 * orientation waypoint handling, etc.), and brings in all the functionalities
 * from TrajectoryTaskGeneric. This ExactCubicTrajectoryTask only handles specific
 * aspect of the ExactCubic curve.
 */
struct MC_TASKS_DLLAPI ExactCubicTrajectoryTask : public SplineTrajectoryTask<ExactCubicTrajectoryTask>
{
  friend struct SplineTrajectoryTask<ExactCubicTrajectoryTask>;

  /**
   * \brief Trajectory following an exact cubic spline with given initial and
   * final acceleration/velocity. The curve will pass exactly through the
   * specified position waypoints (see posWaypoints) and will interpolate
   * between orientation waypoints. Initial and final acceleration/velocity will
   * also be enforced.
   *
   * \param frame Frame controlled by this task
   *
   * \param duration Duration of motion
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   * \param target Final world pose to reach
   *
   * \param posWp Waypoints in position specified as pairs of [time, position]
   *
   * \param initVel Initial velocity of the curve (default: Zero)
   *
   * \param initAcc Initial acceleration of the curve (default: Zero)
   *
   * \param finalVel Final velocity of the curve (default: Zero)
   *
   * \param finalAcc Final acceleration of the curve (default: Zero)
   *
   * \param oriWp Waypoints in orientation, specified as pairs of [time, orientation]
   */
  ExactCubicTrajectoryTask(mc_rbdyn::Frame & frame,
                           double duration,
                           double stiffness,
                           double weight,
                           const sva::PTransformd & target,
                           const std::vector<std::pair<double, Eigen::Vector3d>> & posWp = {},
                           const Eigen::Vector3d & initVel = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & initAcc = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & finalVel = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & finalAcc = Eigen::Vector3d::Zero(),
                           const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

  /*! \brief const accessor to the underlying spline (used by SplineTrajectoryTask)
   *
   * \returns The spline
   */
  inline const mc_trajectory::ExactCubic & spline() const noexcept
  {
    return bspline_;
  };
  /*! \brief accessor to the underlying spline (used by SplineTrajectoryTask)
   *
   * \returns The spline
   */
  inline mc_trajectory::ExactCubic & spline() noexcept
  {
    return bspline_;
  };

  /*! \brief Add interactive GUI elements to control the curve waypoints
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  /** \brief Waypoints in position. The curve will pass exactly through these waypoints.
   */
  inline void posWaypoints(const std::vector<std::pair<double, Eigen::Vector3d>> & posWp)
  {
    bspline_.waypoints(posWp);
  }

  /** \brief Initial and final velocity and acceleration constraints for the
   * curve
   *
   * \param initVel Initial velocity of the curve (default: Zero)
   *
   * \param initAcc Initial acceleration of the curve (default: Zero)
   *
   * \param finalVel Final velocity of the curve (default: Zero)
   *
   * \param finalAcc Final acceleration of the curve (default: Zero)
   */
  inline void constraints(const Eigen::Vector3d & initVel,
                          const Eigen::Vector3d & initAcc,
                          const Eigen::Vector3d & finalVel,
                          const Eigen::Vector3d & finalAcc)
  {
    bspline_.constraints(initVel, initAcc, finalVel, finalAcc);
  }

protected:
  /*! \brief Sets the curve target pose
   * \param target Target pose for the curve
   */
  inline void targetPos(const Eigen::Vector3d & target)
  {
    bspline_.target(target);
  }

  /** \brief Returns the curve's target position */
  inline const Eigen::Vector3d & targetPos() const noexcept
  {
    return bspline_.target();
  }

protected:
  mc_trajectory::ExactCubic bspline_;
  sva::PTransformd initialPose_;
};

} // namespace mc_tasks
