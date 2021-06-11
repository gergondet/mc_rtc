/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/VectorOrientationFunction.h>

namespace mc_tasks
{

/*! \brief Control the orientation of a body specified by vector orientation */
struct MC_TASKS_DLLAPI VectorOrientationTask : public TrajectoryTaskGeneric<mc_tvm::VectorOrientationFunction>
{
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::VectorOrientationFunction>;

  /*! \brief Constructor with user-specified target
   *
   * \param frame Frame that will be controlled
   *
   * \param frameVector Vector to be controlled, expressed in the frame's frame
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  VectorOrientationTask(mc_rbdyn::RobotFrame & frame,
                        const Eigen::Vector3d & frameVector,
                        double stiffness = 2.0,
                        double weight = 500);

  /*! \brief Set the frame vector to be controlled
   *
   * \param vector Vector to be controlled in the body frame
   *
   */
  inline void frameVector(const Eigen::Vector3d & vector) noexcept
  {
    errorT_->frameVector(vector);
  }

  /*! \brief Get the current controlled vector in the reference frame
   *
   * \returns The body orientation target in world frame
   *
   */
  inline const Eigen::Vector3d & frameVector() const noexcept
  {
    return errorT_->frameVector();
  }

  /*! \brief Set world target for the controlled vector
   *
   * \param vector Target vector in the world frame
   *
   */
  inline void targetVector(const Eigen::Vector3d & vector) noexcept
  {
    errorT_->target(vector);
  }

  /**
   * @brief Get the target orientation
   *
   * @return The target orientation in world frame
   */
  inline const Eigen::Vector3d & targetVector() const noexcept
  {
    return errorT_->target();
  }

  /**
   * @brief Get the current body orientation
   *
   * @return The current body orientation vector in world frame
   */
  inline const Eigen::Vector3d & actual() const noexcept
  {
    return errorT_->actual();
  }

  /*! \brief Return the controlled frame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return errorT_->frame();
  }

  /*! \brief Return the controlled robot */
  inline const mc_rbdyn::Robot & robot() const noexcept
  {
    return errorT_->frame().robot();
  }

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;

  size_t bIndex_;
  inline const Eigen::Vector3d & bodyPos() const noexcept
  {
    return robot().mbc().bodyPosW[bIndex_].translation();
  }
};

} // namespace mc_tasks
