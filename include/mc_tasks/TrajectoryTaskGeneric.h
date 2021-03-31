/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/MetaTask.h>

#include <mc_tasks/details/details.h>

#include <mc_tvm/JointsSelectorFunction.h>

#include <mc_rbdyn/Robots.h>

#include <tvm/ControlProblem.h>
#include <tvm/task_dynamics/Proportional.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>

namespace mc_tasks
{

// FIXME It may be better to create a TaskWithRequirementsPtr directly rather than having intermediate values for
// stiffness, damping, weight and dimWeight but this would require tvm::Task to allow swapping functions

/*! \brief Generic wrapper for a TVM function put in the cost function
 *
 * This task is meant to be derived to build actual tasks
 *
 */
template<typename T>
struct TrajectoryTaskGeneric : public MetaTask
{
  using TrajectoryBase = TrajectoryTaskGeneric<T>;

  static constexpr bool hasRefVel = details::has_refVel_v<T>;
  static constexpr bool hasRefAccel = details::has_refAccel_v<T>;
  // Return type of the refVel() getter (if any)
  using refVel_return_t = details::refVel_return_t<T>;
  // For use in setter function and prevent forming reference to void
  using refVel_t = std::conditional_t<std::is_void_v<refVel_return_t>, int, std::decay_t<refVel_return_t>>;
  // Return type of the refAccel() getter (if any)
  using refAccel_return_t = details::refAccel_return_t<T>;
  // For use in setter function and prevent forming reference to void
  using refAccel_t = std::conditional_t<std::is_void_v<refVel_return_t>, int, std::decay_t<refVel_return_t>>;

  /*! \brief Constructor (auto damping)
   *
   * This is a simple constructor alternative. Damping is set to
   * 2*sqrt(stiffness). This is the most appropriate constructor to use
   * TrajectoryTask as a SetPointTask
   *
   * \param robot Robot used in the task
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   */
  TrajectoryTaskGeneric(mc_rbdyn::Robot & robot, double stiffness, double weight);

  ~TrajectoryTaskGeneric() override;

  /*! \brief Reset task target velocity and acceleration to zero
   *
   */
  void reset() override;

  /*! \brief Set the trajectory reference velocity
   *
   * \param vel New reference velocity
   *
   */
  void refVel(const refVel_t & vel)
  {
    if constexpr(hasRefVel)
    {
      errorT_->refVel(vel);
    }
    else
    {
      static_assert(details::always_false_v<T>, "refVel is not defined for this function");
    }
  }

  /*! \brief Get the trajectory reference velocity
   *
   */
  refVel_return_t refVel() const noexcept
  {
    if constexpr(hasRefVel)
    {
      return errorT_->refVel();
    }
    else
    {
      static_assert(details::always_false_v<T>, "refVel is not defined for this function");
    }
  }

  /*! \brief Set the trajectory reference acceleration
   *
   * \param accel New reference acceleration
   *
   */
  void refAccel(const refAccel_t & accel)
  {
    if constexpr(hasRefAccel)
    {
      errorT_->refAccel(accel);
    }
    else
    {
      static_assert(details::always_false_v<T>, "refAccel is not defined for this function");
    }
  }

  /*! \brief Get the trajectory reference acceleration
   *
   */
  refAccel_return_t refAccel() const noexcept
  {
    if constexpr(hasRefAccel)
    {
      return errorT_->refAccel();
    }
    else
    {
      static_assert(details::always_false_v<T>, "refAccel is not defined for this function");
    }
  }

  /*! \brief Set the task stiffness/damping
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Task stiffness
   *
   */
  void stiffness(double stiffness)
  {
    setGains(stiffness, 2 * std::sqrt(stiffness));
  }

  /*! \brief Set dimensional stiffness
   *
   * The caller should be sure that the dimension of the vector fits the task dimension.
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Dimensional stiffness
   *
   */
  void stiffness(const Eigen::VectorXd & stiffness)
  {
    setGains(stiffness, 2 * stiffness.cwiseSqrt());
  }

  /*! \brief Set the task damping, leaving its stiffness unchanged
   *
   * \param damping Task stiffness
   *
   */
  void damping(double damping)
  {
    damping_.setConstant(damping);
    setGains(stiffness_, damping_);
  }

  /*! \brief Set dimensional damping
   *
   * The caller should be sure that the dimension of the vector fits the task dimension.
   *
   * \param damping Dimensional damping
   *
   */
  void damping(const Eigen::VectorXd & damping)
  {
    setGains(stiffness_, damping);
  }

  /*! \brief Set both stiffness and damping
   *
   * \param stiffness Task stiffness
   *
   * \param damping Task damping
   *
   */
  void setGains(double stiffness, double damping)
  {
    stiffness_.setConstant(stiffness);
    damping_.setConstant(damping);
    setGains(stiffness_, damping_);
  }

  /*! \brief Set dimensional stiffness and damping
   *
   * The caller should be sure that the dimensions of the vectors fit the task dimension.
   *
   * \param stiffness Dimensional stiffness
   *
   * \param damping Dimensional damping
   *
   */
  void setGains(const Eigen::VectorXd & stiffness, const Eigen::VectorXd & damping);

  /*! \brief Get the current task stiffness */
  double stiffness() const noexcept
  {
    return stiffness_(0);
  }

  /*! \brief Get the current task damping */
  double damping() const noexcept
  {
    return damping_(0);
  }

  /*! \brief Get the current task dimensional stiffness */
  const Eigen::VectorXd & dimStiffness() const noexcept
  {
    return stiffness_;
  }

  /*! \brief Get the current task dimensional damping */
  const Eigen::VectorXd & dimDamping() const noexcept
  {
    return damping_;
  }

  /*! \brief Set the task weight
   *
   * \param w Task weight
   *
   */
  void weight(double w);

  /*! \brief Returns the task weight */
  double weight() const noexcept
  {
    return weight_;
  }

  void dimWeight(const Eigen::VectorXd & dimW) override;

  const Eigen::VectorXd & dimWeight() const noexcept override
  {
    return dimWeight_;
  }

  /** \brief Create an active joints selector
   *
   * \warning This function should only be called if the task hasn't yet been
   * added to the solver. If the tasks is already in the solver it does nothing,
   * and warns that it had no effect. Call void selectActiveJoints(mc_solver::QPSolver &, const std::vector<std::string>
   * &, const std::map<std::string, std::vector<std::array<int, 2>>> &) instead.
   *
   * @param activeJointsName Name of the joints activated for this task
   * \param activeDofs Allow to select only part of the dofs of a joint
   * \param checkJoints When true, checks whether the joints exist in the robot
   * \throws If checkJoints is true and a joint name in activeJointsName is not
   * part of the robot
   */
  void selectActiveJoints(const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {},
                          bool checkJoints = true);

  /** \brief Create an active joints selector
   *
   * \note Calling this function is a bit expensive. If the task is already in
   * the solver, it will be removed first, then recreated with the joint
   * selector and added to the solver again. If possible, consider calling void selectActiveJoints(const
   * std::vector<std::string> &, const std::map<std::string, std::vector<std::array<int, 2>>> &) before adding the task
   * to the solver.
   *
   * @param activeJointsName Name of the joints activated for this task
   * \param activeDofs Allow to select only part of the dofs of a joint
   * \throws If a joint name in activeJointsName is not part of the robot
   */
  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  /** \brief Create an inactive joints selector
   *
   * \warning This function should only be called if the task hasn't yet been
   * added to the solver. If the tasks is already in the solver it does nothing,
   * and warns that it had no effect. Call void selectInactiveJoints(mc_solver::QPSolver &, const
   * std::vector<std::string> &, const std::map<std::string, std::vector<std::array<int, 2>>> &) instead.
   *
   * @param unactiveJointsName Name of the joints not activated for this task
   * \param unactiveDofs Allow to select only part of the dofs of a joint
   * \param checkJoints When true, checks whether the joints exist in the robot
   * \throws If checkJoints is true and a joint name in unactiveJointsName is not
   * part of the robot
   */
  void selectInactiveJoints(const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {},
                            bool checkJoints = true);

  /** \brief Create an inactive joints selector
   *
   * \note Calling this function is a bit expensive. If the task is already in
   * the solver, it will be removed first, then recreated with the joint
   * selector and added to the solver again. If possible, consider calling void selectInactiveJoints(const
   * std::vector<std::string> &, const std::map<std::string, std::vector<std::array<int, 2>>> &) before adding the task
   * to the solver.
   *
   * @param unactiveJointsName Name of the joints not activated for this task
   * \param unactiveDofs Allow to select only part of the dofs of a joint
   * \throws If a joint name in unactiveJointsName is not part of the robot
   */
  void selectInactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  virtual void resetJointsSelector();

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd eval() const override;

  Eigen::VectorXd speed() const override;

  Eigen::VectorXd normalAcc() const;

  /** Expose the underlying TVM function to acess (e.g.) the jacobian */
  const tvm::function::abstract::Function & function() const noexcept
  {
    return selectorT_ ? *selectorT_ : *errorT_;
  }

  /** Expose the underlying error function without the selector */
  const T & error() const noexcept
  {
    return *errorT_;
  }

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

  /** Access the robot used by this task */
  inline const mc_rbdyn::Robot & robot() const noexcept
  {
    return *robot_;
  }

  /** True if the task has already been added to a solver */
  inline bool inSolver() const noexcept
  {
    return task_ != nullptr;
  }

protected:
  /*! This function should be called to finalize the task creation, it will
   * create the actual tasks objects */
  template<typename... Args>
  void finalize(Args &&... args);

  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  void addToLogger(mc_rtc::Logger & logger) override;

  mc_rbdyn::RobotPtr robot_;
  std::shared_ptr<T> errorT_ = nullptr;
  tvm::TaskWithRequirementsPtr task_; // null if the task is not in solver

  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

private:
  Eigen::VectorXd stiffness_;
  Eigen::VectorXd damping_;
  double weight_;
  Eigen::VectorXd dimWeight_;
  std::shared_ptr<mc_tvm::JointsSelectorFunction> selectorT_ = nullptr;

  void update(mc_solver::QPSolver &) override;
};

} // namespace mc_tasks

#include <mc_tasks/TrajectoryTaskGeneric.hpp>
