/*
* Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/PostureFunction.h>

namespace mc_tasks
{

// FIXME Rewrite an equivalent to tasks::qp::JointGains/tasks::qp::JointStiffness which interacts with dimensional
// stiffness

/** A posture task for a given robot */
struct MC_TASKS_DLLAPI PostureTask : public TrajectoryTaskGeneric<mc_tvm::PostureFunction>
{
  using TrajectoryBase = TrajectoryTaskGeneric<mc_tvm::PostureFunction>;

  PostureTask(mc_rbdyn::Robot & robot, double stiffness = 1, double weight = 10);

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

  void reset() override;

  /** Change posture objective */
  inline void posture(const std::vector<std::vector<double>> & p) noexcept
  {
    errorT_->posture(p);
  }

  /** Get current posture objective */
  inline const std::vector<std::vector<double>> & posture() const noexcept
  {
    return errorT_->posture();
  }

  /** Set specific joint targets
   *
   * \param joints Map of joint's name to joint's configuration
   *
   */
  void target(const mc_rtc::map<std::string, std::vector<double>> & joints);

protected:
  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  void addToLogger(mc_rtc::Logger & logger) override;

private:
  /** Store mimic information */
  mc_rtc::map<std::string, std::vector<int>> mimics_;
};

using PostureTaskPtr = std::shared_ptr<PostureTask>;

} // namespace mc_tasks
