/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/LookAtTask.h>
#include <mc_tasks/ros_api.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mc_tasks
{
/*! \brief Control the gaze vector of a body to look towards a world position
 * updated at each iteration from a ROS TF Frame.
 */
struct MC_TASKS_ROS_DLLAPI LookAtTFTask : public LookAtTask
{
  /*! \brief Constructor
   *
   * \param frame Frame controlled by this task
   *
   * \param frameVector Gaze vector for the frame. For instance [1., 0, 0] will
   * try to align the x axis of the frame with the target direction.
   *
   * \param sourceFrame name of the target's source tf
   *
   * \param targetFrame name of the target's tf
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  LookAtTFTask(mc_rbdyn::RobotFrame & frame,
               const Eigen::Vector3d & frameVector,
               const std::string & sourceFrame,
               const std::string & targetFrame,
               double stiffness = 0.5,
               double weight = 200);

  /*! \brief Update the gaze target from TF position */
  void update(mc_solver::QPSolver &) override;

private:
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  std::string sourceFrame;
  std::string targetFrame;
  Eigen::Vector3d target_ori;
};

} // namespace mc_tasks
