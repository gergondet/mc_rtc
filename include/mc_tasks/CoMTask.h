/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/CoMFunction.h>

namespace mc_tasks
{

/*! \brief Control a robot's CoM */
struct MC_TASKS_DLLAPI CoMTask : public TrajectoryTaskGeneric<mc_tvm::CoMFunction>
{
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::CoMFunction>;

public:
  /*! \brief Constructor
   *
   * \param robot Robot whose CoM is controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  CoMTask(mc_rbdyn::Robot & robot, double stiffness = 5.0, double weight = 100.);

  /*! \brief Set the CoM target to a given position
   *
   * \param com New CoM target
   *
   */
  inline void com(const Eigen::Vector3d & com) noexcept
  {
    errorT_->com(com);
  }

  /*! \brief Return the current CoM target
   *
   * \returns The current CoM target
   */
  inline const Eigen::Vector3d & com() const
  {
    return errorT_->com();
  }

  /** \brief Actual CoM position (computed at the previous iteration)
   *
   * \return Return the current CoM position
   */
  const Eigen::Vector3d & actual() const noexcept
  {
    return robot_->com().com();
  }

  /*! \brief Load from configuration */
  void load(mc_solver::QPSolver &, const mc_rtc::Configuration & config) override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder &) override;
  void addToLogger(mc_rtc::Logger & logger) override;
};

using CoMTaskPtr = std::shared_ptr<CoMTask>;

} // namespace mc_tasks
