/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/Constraint.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/api.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robots.h>

#include <mc_rtc/pragma.h>
#include <mc_rtc/time.h>

#include <tvm/ControlProblem.h>
#include <tvm/LinearizedControlProblem.h>
#include <tvm/scheme/WeightedLeastSquares.h>

#include <memory>

namespace mc_tasks
{
struct MetaTask;
using MetaTaskPtr = std::shared_ptr<MetaTask>;
} // namespace mc_tasks

namespace mc_rtc
{
struct Logger;
namespace gui
{
struct StateBuilder;
using GUI = StateBuilder;
} // namespace gui
} // namespace mc_rtc

namespace mc_solver
{

// Work around GCC bug see: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=43407
MC_RTC_diagnostic_push
MC_RTC_diagnostic_ignored(GCC, "-Wattributes")

/** Describe the type of feedback used to control the robot */
enum class MC_SOLVER_DLLAPI FeedbackType
{
  /** No feedback, i.e. open-loop control */
  None,
  /** Use encoder values for actuated joints */
  Joints,
  /** Joints + encoder velocity obtained from numerical differentiation */
  JointsWVelocity,
  /** Run in closed loop w.r.t realRobots. The user is responsible for ensuring
   * that the observed state of the real robots is valid */
  ObservedRobots
};

MC_RTC_diagnostic_pop

/** \class QPSolver
 *
 * Wraps a tasks::qp::QPSolver instance
 *
 * Always ensure that the solver is up-to-date
 */

struct MC_SOLVER_DLLAPI QPSolver
{
public:
  /** Constructor
   * \param robots Set of robots managed by this solver
   * \param realRobots Set of robots managed by this solver
   * \param logger Logger associated to this solver
   * \param gui GUI server associated to this solver
   * \param timeStep Timestep of the solver
   *
   * \note The real robots will be created by copying the provided robots
   */
  QPSolver(mc_rbdyn::RobotsPtr robots,
           mc_rbdyn::RobotsPtr realRobots,
           std::shared_ptr<mc_rtc::Logger> logger,
           std::shared_ptr<mc_rtc::gui::StateBuilder> gui,
           double timeStep);

  /** Add a constraint set
   * \param cs Constraint set added to the solver
   */
  void addConstraint(ConstraintPtr constraint);

  /** Remove a constraint set
   * \param cs Constrain set removed from the solver
   */
  void removeConstraint(ConstraintPtr constraint);

  /** Add a task to the solver, its update function will be called every iteration
   *
   * Adding the same task multiple times has no effect.
   *
   * \param task Task that will be added
   *
   */
  void addTask(mc_tasks::MetaTaskPtr task);

  /** Remove a task from the solver
   *
   * Removing a task that is not in the solver has no effect.
   *
   * \param task Task that will beremoved
   *
   */
  void removeTask(mc_tasks::MetaTaskPtr task);

  /** Returns the current set of contacts */
  inline const std::vector<mc_rbdyn::Contact> & contacts() const noexcept
  {
    return contacts_;
  }

  /** Add or update a contact
   *
   * If the contact already exists this updates the friction or dof vector if
   * the specification is different from the existing contact
   *
   * \param contact Contact definition
   *
   * \throws If the contact definition is ill-formed (wrong robot or wrong
   * surface)
   */
  void addContact(const mc_rbdyn::Contact & contact);

  /** Add or update a virtual contact
   *
   * If the contact already exists this updates the dof vector if necessary
   *
   * Contrary to \ref addContact this does not add contact forces and only requires that the frames exist (not surfaces)
   *
   * \param contact Contact definition
   *
   */
  void addVirtualContact(const mc_rbdyn::Contact & contact);

  /** Remove a contact
   *
   * Does nothing if the contact is not currently active
   *
   * \param contact Contact definition
   */
  void removeContact(const mc_rbdyn::Contact & contact);

  /** Returns the MetaTasks currently in the solver */
  inline const std::vector<mc_tasks::MetaTaskPtr> & tasks() const noexcept
  {
    return tasks_;
  }

  /** Desired resultant of contact force in robot surface frame
   * \param contact Contact for which the force is desired.
   * This contact must be one of the active contacts in the solver.
   * \return Contact force in robot surface frame
   */
  const sva::ForceVecd desiredContactForce(const mc_rbdyn::Contact & id) const;

  /** Run one iteration of the QP.
   *
   * If succesful, will update the robots' configurations
   *
   * \param fType Type of feedback used to close the loop on sensory information
   *
   * \return True if successful, false otherwise.
   */
  bool run(FeedbackType fType = FeedbackType::None);

  /** Gives access to the robots controlled by this solver */
  inline const mc_rbdyn::Robots & robots() const noexcept
  {
    return *robots_;
  }

  /** Gives access to the robots controlled by this solver */
  inline mc_rbdyn::Robots & robots() noexcept
  {
    return *robots_;
  }

  /** Gives access to the observed robots */
  inline const mc_rbdyn::Robots & realRobots() const noexcept
  {
    return *realRobots_;
  }
  inline mc_rbdyn::Robots & realRobots() noexcept
  {
    return *realRobots_;
  }

  /** Returns the timestep of the solver */
  inline double dt() const noexcept
  {
    return dt_;
  }

  /** Access to the logger instance */
  inline mc_rtc::Logger & logger() const noexcept
  {
    return *logger_;
  }

  /** Access to the gui instance */
  inline mc_rtc::gui::StateBuilder & gui() const noexcept
  {
    return *gui_;
  }

  /** Access the underlying problem */
  inline tvm::LinearizedControlProblem & problem() noexcept
  {
    return problem_;
  }

  /** Returns the duration of the latest run call */
  const mc_rtc::duration_ms & solveTime() noexcept
  {
    return solve_dt_;
  }

private:
  /** Controlled robots */
  std::shared_ptr<mc_rbdyn::Robots> robots_;
  /** Observed robots */
  std::shared_ptr<mc_rbdyn::Robots> realRobots_;
  /** Solver timestep */
  double dt_;
  /** Control problem */
  tvm::LinearizedControlProblem problem_;
  /** Solver scheme */
  tvm::scheme::WeightedLeastSquares solver_;
  /** Constraints currently active */
  std::vector<ConstraintPtr> constraints_;
  /** Dynamics constraint currently active for robots in the solver */
  mc_rtc::map<std::string, DynamicsConstraintPtr> dynamics_;
  /** Tasks currently active */
  std::vector<mc_tasks::MetaTaskPtr> tasks_;
  /** Current contacts */
  std::vector<mc_rbdyn::Contact> contacts_;
  /** Contact data on the solver side */
  struct ContactData
  {
    /** Contact function in the solver */
    tvm::TaskWithRequirementsPtr contactConstraint_;
    /** Force variables on r1 side (if any) */
    tvm::VariableVector f1_;
    /** Constraints on f1 */
    std::vector<tvm::TaskWithRequirementsPtr> f1Constraints_;
    /** Force variables on r2 side (if any) */
    tvm::VariableVector f2_;
    /** Constraints on f2 */
    std::vector<tvm::TaskWithRequirementsPtr> f2Constraints_;
  };
  /** Related contact functions */
  std::vector<ContactData> contactsData_;
  /** Pointer to the Logger */
  std::shared_ptr<mc_rtc::Logger> logger_ = nullptr;
  /** Pointer to the GUI helper */
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_ = nullptr;
  /** Runtime of the latest run call */
  mc_rtc::duration_ms solve_dt_{0};

  /** Common part of control loop */
  bool runCommon();
  /** Run without feedback (open-loop) */
  bool runOpenLoop();
  /** Run with encoders' feedback */
  bool runJointsFeedback(bool wVelocity);

  /**
   * WARNING EXPERIMENTAL
   *
   * Runs the QP on an estimated robot state.
   *
   * Uses the real robot state (mbc.q and mbc.alpha) from realRobots() instances.
   * It is the users responsibility to ensure that the real robot instance is properly estimated
   * and filled. Typically, this will be done through the Observers pipeline.
   * For example, the following pipeline provides a suitable state:
   *
   * \code{.yaml}
   * RunObservers: [Encoder, KinematicInertial]
   * UpdateObservers: [Encoder, KinematicInertial]
   * \endcode
   *
   * @return True if successful, false otherwise
   */
  bool runClosedLoop();

  /** Update a robot from the optimization result */
  void updateRobot(mc_rbdyn::Robot & robot);

  /** Feedback data */
  std::vector<std::vector<double>> prev_encoders_{};
  std::vector<std::vector<double>> encoders_alpha_{};
  std::vector<Eigen::VectorXd> control_q_{};
  std::vector<Eigen::VectorXd> control_alpha_{};

  std::vector<ConstraintPtr>::iterator getConstraint(ConstraintPtr & c);
  std::vector<mc_tasks::MetaTaskPtr>::iterator getTask(mc_tasks::MetaTaskPtr & t);
  size_t getContactIdx(const mc_rbdyn::Contact & contact);

  bool addDynamicsConstraint(const DynamicsConstraintPtr & constraint);
  void removeDynamicsConstraint(const DynamicsConstraintPtr & constraint);
  std::tuple<size_t, bool> addVirtualContactImpl(const mc_rbdyn::Contact & contact);
  void addContactToDynamics(const std::string & robot,
                            mc_rbdyn::Frame & frame,
                            const std::vector<sva::PTransformd> & points,
                            tvm::VariableVector & forces,
                            std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                            const Eigen::MatrixXd & frictionCone,
                            double dir);
};

using SolverPtr = std::shared_ptr<mc_solver::QPSolver>;

} // namespace mc_solver
