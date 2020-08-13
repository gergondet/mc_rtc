/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/OrientationFunction.h>

namespace mc_tasks
{

/*! \brief Control the orientation of a frame */
struct MC_TASKS_DLLAPI OrientationTask : public TrajectoryTaskGeneric<mc_tvm::OrientationFunction>
{
public:
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::OrientationFunction>;

  /*! \brief Constructor
   *
   * \param frame Frame to control
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  OrientationTask(mc_rbdyn::Frame & frame, double stiffness = 2.0, double weight = 500);

  /*! \brief Set the body orientation target
   *
   * \param ori Body orientation in world frame
   *
   */
  inline void orientation(const Eigen::Matrix3d & ori) noexcept
  {
    errorT_->orientation(ori);
  }

  /*! \brief Get the current body orientation target
   *
   * \returns The body orientation target in world frame
   *
   */
  inline const Eigen::Matrix3d & orientation() noexcept
  {
    return errorT_->orientation();
  }

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;
};

} // namespace mc_tasks
