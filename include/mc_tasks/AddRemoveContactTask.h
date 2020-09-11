/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/MetaTask.h>
#include <mc_tasks/VelocityTask.h>

#include <mc_solver/BoundedSpeedConstr.h>

namespace mc_tasks
{

/** Add or remove a contact
 *
 * This task combines a bounded speed constraint and velocity task to drive a frame in its normal direction. The
 * velocity sign informs the direction.
 */
struct MC_TASKS_DLLAPI AddRemoveContactTask : public MetaTask
{
public:
  /** Constructor
   *
   * \param frame Frame controlled by this task
   *
   * \param speed Maximum speed of the displacement
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   */
  AddRemoveContactTask(mc_rbdyn::Frame & frame, double speed = 0.01, double stiffness = 2, double weight = 1000);

  /*! \brief Set the desired dislacement speed */
  void speed(double s);

  inline void update(mc_solver::QPSolver &) override {}

  /** This is managed by this task */
  inline void dimWeight(const Eigen::VectorXd &) override {}

  inline const Eigen::VectorXd & dimWeight() const noexcept override
  {
    return task_->dimWeight();
  }

  inline Eigen::VectorXd eval() const override
  {
    return task_->eval();
  }

  inline Eigen::VectorXd speed() const override
  {
    return task_->speed();
  }

  /* Hide these virtual functions */
  inline void selectActiveJoints(mc_solver::QPSolver & solver,
                                 const std::vector<std::string> & joints,
                                 const std::map<std::string, std::vector<std::array<int, 2>>> & dofs = {}) override
  {
    task_->selectActiveJoints(solver, joints, dofs);
  }

  inline void selectInactiveJoints(mc_solver::QPSolver & solver,
                                   const std::vector<std::string> & joints,
                                   const std::map<std::string, std::vector<std::array<int, 2>>> & dofs = {}) override
  {
    task_->selectInactiveJoints(solver, joints, dofs);
  }

  inline void resetJointsSelector(mc_solver::QPSolver & solver) override
  {
    task_->resetJointsSelector(solver);
  }

  inline void reset() override
  {
    task_->reset();
  }

  inline const mc_rbdyn::Frame & frame() const noexcept
  {
    return task_->frame();
  }

private:
  mc_solver::BoundedSpeedConstrPtr constraint_;
  mc_tasks::VelocityTaskPtr task_;
  double speed_;

  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  inline void addToLogger(mc_rtc::Logger & logger) override
  {
    MetaTask::addToLogger(*task_, logger);
  }

  inline void removeFromLogger(mc_rtc::Logger & logger) override
  {
    MetaTask::removeFromLogger(*task_, logger);
  }

  void addToGUI(mc_rtc::GUI & gui) override;
};

using AddRemoveContactTaskPtr = std::shared_ptr<mc_tasks::AddRemoveContactTask>;

} // namespace mc_tasks
