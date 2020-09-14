/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TransformTask.h>

namespace mc_tasks
{

namespace force
{

namespace
{
static const std::pair<double, double> defaultFGain = {0.02, 0.005};
static const std::pair<double, double> defaultTGain = {0.2, 0.05};
} // namespace

/*! \brief Add a contact in a compliant manner
 *
 * Uses an mc_tasks::TransformTask to drive the selected frame until certain
 * force and torque thresholds are reached.
 *
 * This is a force-compliant variant of mc_tasks::AddContactTask that should be
 * used when a force sensor is available.
 */
struct MC_TASKS_DLLAPI ComplianceTask : MetaTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Constructor with dof restrictions
   *
   * The force control is only active when the difference between the current sensor reading and the objective is above
   * \p forceThresh and/or \p torqueThresh
   *
   * \param frame Frame controlled by this task
   *
   * \param dof Allows to enable/disable some axis in the control/wrench monitoring
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   * \param forceThresh Force threshold to reach
   *
   * \param Torque threshold to reach
   *
   * \param forceGain PD gains on the force part
   *
   * \param torqueGain PD gains on the torque part
   *
   * \throws If the frame the task is attempting to control does not have a
   * sensor attached to it
   *
   */
  ComplianceTask(mc_rbdyn::Frame & frame,
                 const Eigen::Vector6d & dof = Eigen::Vector6d::Ones(),
                 double stiffness = 5.0,
                 double weight = 1000.0,
                 double forceThresh = 3.,
                 double torqueThresh = 1.,
                 std::pair<double, double> forceGain = defaultFGain,
                 std::pair<double, double> torqueGain = defaultTGain);

  /*! \brief Reset the task
   *
   * Set the frame's position objective to the frame's current position
   *
   */
  inline void reset() override
  {
    task_->reset();
  }

  /*! \brief Modify the target wrench */
  inline void setTargetWrench(const sva::ForceVecd & wrench) noexcept
  {
    obj_ = wrench;
  }

  /*! \brief Get the current target wrench */
  inline const sva::ForceVecd & getTargetWrench() const noexcept
  {
    return obj_;
  }

  /*! \brief Set the task stiffness */
  inline void stiffness(double s)
  {
    task_->stiffness(s);
  }

  /*! \brief Get the task stiffness */
  inline double stiffness() const noexcept
  {
    return task_->stiffness();
  }

  /*! \brief Set the task weight */
  inline void weight(double w)
  {
    task_->weight(w);
  }

  /*! \brief Get the task weight */
  inline double weight() const noexcept
  {
    return task_->weight();
  }

  /*! \brief Set the force threshold */
  inline void forceThreshold(double t) noexcept
  {
    forceThresh_ = t;
  }

  /*! \brief Get the force threshold */
  inline double forceThreshold() const noexcept
  {
    return forceThresh_;
  }

  /*! \brief Set the torque threshold */
  inline void torqueThreshold(double t) noexcept
  {
    torqueThresh_ = t;
  }

  /*! \brief Get the torque threshold */
  inline double torqueThreshold() const noexcept
  {
    return torqueThresh_;
  }

  /*! \brief Set the force gain */
  inline void forceGain(std::pair<double, double> t) noexcept
  {
    forceGain_ = t;
  }

  /*! \brief Get the force gain */
  inline const std::pair<double, double> & forceGain() const noexcept
  {
    return forceGain_;
  }

  /*! \brief Set the torque gain */
  inline void torqueGain(std::pair<double, double> t) noexcept
  {
    torqueGain_ = t;
  }

  /*! \brief Get the torque gain */
  inline const std::pair<double, double> & torqueGain() const noexcept
  {
    return torqueGain_;
  }

  /*! \brief Set the current dof vector */
  inline void dof(const Eigen::Vector6d & dof) noexcept
  {
    dof_ = dof;
  }

  /*! \brief Get the current dof vector */
  inline const Eigen::Vector6d & dof() const noexcept
  {
    return dof_;
  }

  inline void dimWeight(const Eigen::VectorXd & dimW) override
  {
    task_->dimWeight(dimW);
  }

  inline const Eigen::VectorXd & dimWeight() const noexcept override
  {
    return task_->dimWeight();
  }

  inline void selectActiveJoints(mc_solver::QPSolver & solver,
                                 const std::vector<std::string> & activeJointsName,
                                 const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override
  {
    task_->selectActiveJoints(solver, activeJointsName, activeDofs);
  }

  inline void selectInactiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & inactiveJointsName,
      const std::map<std::string, std::vector<std::array<int, 2>>> & inactiveDofs = {}) override
  {
    task_->selectInactiveJoints(solver, inactiveJointsName, inactiveDofs);
  }

  inline void resetJointsSelector(mc_solver::QPSolver & solver) override
  {
    task_->resetJointsSelector(solver);
  }

  inline Eigen::VectorXd eval() const override
  {
    return error_.vector();
  }

  inline Eigen::VectorXd speed() const override
  {
    return errorD_.vector();
  }

private:
  sva::PTransformd computePose();

  /** Task used by this task */
  TransformTaskPtr task_;
  /** Current wrench error */
  sva::ForceVecd error_ = sva::ForceVecd::Zero();
  /** Wrench objective */
  sva::ForceVecd obj_ = sva::ForceVecd::Zero();
  /** Previous error measurement */
  sva::ForceVecd prevError_ = sva::ForceVecd::Zero();
  /** Error derivative */
  sva::ForceVecd errorD_ = sva::ForceVecd::Zero();
  /** Force threshold */
  double forceThresh_;
  /** Torque threshold */
  double torqueThresh_;
  /** Force gains */
  std::pair<double, double> forceGain_;
  /** Torque gains */
  std::pair<double, double> torqueGain_;
  /** Active degrees of freedom (force/torque wise) */
  Eigen::Vector6d dof_;
  /** True on the first update */
  bool firstUpdate_ = true;

  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver &) override;
};

} // namespace force

} // namespace mc_tasks
