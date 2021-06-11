/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/SplineTrajectoryTask.h>

#include <mc_trajectory/BSpline.h>

namespace mc_tasks
{

/*! \brief Track a bezier curve with a robot surface.
 *
 * SplineTrajectoryTask takes care of much of the logic (task target updates,
 * orientation waypoint handling, etc.), and brings in all the functionalities
 * from TrajectoryTaskGeneric. This BSplineTrajectoryTask only handles specific
 * aspect of the Bezier curve.
 */
struct MC_TASKS_DLLAPI BSplineTrajectoryTask : public SplineTrajectoryTask<BSplineTrajectoryTask>
{
  friend struct SplineTrajectoryTask<BSplineTrajectoryTask>;
  using SplineTrajectoryBase = SplineTrajectoryTask<BSplineTrajectoryTask>;
  using waypoints_t = mc_trajectory::BSpline::waypoints_t;

public:
  using SplineTrajectoryBase::target;
  /**
   * \brief Creates a trajectory that follows a bspline curve
   *
   * \param frame Frame to control
   *
   * \param duration Duration of the motion
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   * \param target Final position to reach in the world frame
   *
   * \param posWp Waypoints in position
   *
   * \param oriWp Waypoints in orientation specified as pairs of (time, orientation).
   *
   */
  BSplineTrajectoryTask(mc_rbdyn::RobotFrame & frame,
                        double duration,
                        double stiffness,
                        double weight,
                        const sva::PTransformd & target,
                        const waypoints_t & posWp = {},
                        const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

  /*! \brief const accessor to the underlying spline (used by SplineTrajectoryTask)
   *
   * \returns The spline
   */
  inline const mc_trajectory::BSpline & spline() const noexcept
  {
    return bspline_;
  }

  /*! \brief accessor to the underlying spline (used by SplineTrajectoryTask)
   *
   * \returns The spline
   */
  inline mc_trajectory::BSpline & spline() noexcept
  {
    return bspline_;
  }

  /*! \brief Add interactive GUI elements to control the curve waypoints
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  /** \brief Control points for the bezier curve (position)
   *
   * \param posWp Vector of position control points for the bezier curve.
   * Shouldn't include the starting and target position (use target() instead).
   */
  inline void posWaypoints(const waypoints_t & posWp)
  {
    bspline_.waypoints(posWp);
  }

protected:
  /*! \brief Sets the curve target pose
   * \param target Target pose for the curve
   */
  inline void targetPos(const Eigen::Vector3d & target)
  {
    bspline_.target(target);
  }

  /** \brief Returns the curves target position */
  inline const Eigen::Vector3d & targetPos() const noexcept
  {
    return bspline_.target();
  }

protected:
  mc_trajectory::BSpline bspline_;
};

} // namespace mc_tasks
