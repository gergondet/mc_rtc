/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/PostureFunction.h>

namespace mc_tasks
{

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

  /** Set a specific stiffness for a joint
   *
   * Automatically set critical damping.
   *
   * \note This is a helper above the dimensional stiffness setter, changing
   * the gains of the task after this function is called will clear its effect
   * (except when calling setJointGains or setJointStiffness)
   *
   * \param name Name of the joint
   *
   * \param stiffness Desired stiffness for this joint
   *
   * \throws If the robot has no joint named \p name
   */
  inline void setJointStiffness(const std::string & name, double stiffness)
  {
    setJointGains(name, stiffness, 2 * std::sqrt(stiffness));
  }

  /** Set specific gains for a joint
   *
   * \note This is a helper above the dimensional stiffness/damping setter,
   * changing the gains of the task after this function is called will clear
   * its effect (except when calling setJointGains or setJointStiffness)
   *
   * \param name Name of the joint
   *
   * \param stiffness Desired stiffness for this joint
   *
   * \param damping Desired damping for this joint
   *
   * \throws If the robot has no joint named \p name
   */
  void setJointGains(const std::string & name, double stiffness, double damping);

protected:
  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  void addToLogger(mc_rtc::Logger & logger) override;

private:
  /** Store mimic information */
  mc_rtc::map<std::string, std::vector<int>> mimics_;
};

using PostureTaskPtr = std::shared_ptr<PostureTask>;

} // namespace mc_tasks
