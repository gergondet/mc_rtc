/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/MomentumFunction.h>

namespace mc_tasks
{

/*! \brief Control the momentum of a robot */
struct MC_TASKS_DLLAPI MomentumTask : public TrajectoryTaskGeneric<mc_tvm::MomentumFunction>
{
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::MomentumFunction>;

public:
  /*! \brief Constructor
   *
   * \param robot Robot whose momentum is controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  MomentumTask(mc_rbdyn::Robot & robot, double stiffness = 2.0, double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task target to the current momentum and reset desired velocity and acceleration to zero.
   *
   */
  void reset() override;

  /*! \brief Get the current target momentum */
  inline const sva::ForceVecd & momentum() const noexcept
  {
    return errorT_->momentum();
  }

  /*! \brief Set the target momentum
   *
   * \param target Target momentum
   *
   */
  inline void momentum(const sva::ForceVecd & mom) noexcept
  {
    errorT_->momentum(mom);
  }

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;
};

} // namespace mc_tasks
