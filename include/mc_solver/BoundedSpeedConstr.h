/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/Constraint.h>

#include <mc_tvm/FrameVelocity.h>

#include <tvm/ControlProblem.h>

#include <SpaceVecAlg/EigenTypedef.h>

namespace mc_solver
{
/** \class BoundedSpeedConstr
 * \brief Wrapper around tasks::qp::BoundedSpeedConstr
 */

/** \class BoundedSpeedConstr
 *
 * This manages a set of velocity bound constraint
 *
 */
struct MC_SOLVER_DLLAPI BoundedSpeedConstr : public Constraint
{
public:
  /** Constructor */
  BoundedSpeedConstr();

  ~BoundedSpeedConstr() override = default;

  /** Add a speed bound on the given frame
   *
   * Given a frame \f$f\f$ whose speed in its own frame is \f$\dot{f_f}\f$, a
   * dof vector \f$d\f$ and a speed limit \f$\overline{s}\f$, this will
   * implement \f$d^{T} \overline{s} \leq d^{T} \dot{f_f} \leq d^{T} \overline{s}\f$
   *
   * \param solver Solver where this constraint will be inserted
   *
   * \param frame Frame that the constraint will be applied to
   *
   * \param dof Dof selection
   *
   * \param speed Speed limit on the given frame
   *
   * Adding the same constraint multiple times will simply update the dof/speed parameters
   */
  void addBoundedSpeed(QPSolver & solver,
                       mc_rbdyn::Frame & frame,
                       const Eigen::Vector6d & dof,
                       const Eigen::Vector6d & speed);

  /** Add a speed bound on the given frame
   *
   * Given a frame \f$f\f$ whose speed in its own frame is \f$\dot{f_f}\f$, a
   * dof vector \f$d\f$, a lower speed limit \f$\underline{s}\f$ and an upper
   * speed limit \\f$\overline{s}\f$, this will implement \f$d^{T} \underline{s}
   * \leq d^{T} \dot{f_f} \leq d^{T} \overline{s}\f$
   *
   * \param solver Solver where this constraint will be inserted
   *
   * \param frame Frame that the constraint will be applied to
   *
   * \param dof Dof selection
   *
   * \param lowerSpeed Lower speed limit on the given frame
   *
   * \param upperSpeed Upper speed limit on the given frame
   *
   * Adding the same constraint multiple times will simply update the dof/speed parameters
   */
  void addBoundedSpeed(QPSolver & solver,
                       mc_rbdyn::Frame & frame,
                       const Eigen::Vector6d & dof,
                       const Eigen::Vector6d & lowerSpeed,
                       const Eigen::Vector6d & upperSpeed);

  /** Remove a speed bound on the given frame
   *
   * \param solver Solver where this constraint will be inserted
   *
   * \param frame Frame that the constraint was applied to
   *
   * \returns True if a constraint was removed, false otherwise
   */
  bool removeBoundedSpeed(QPSolver & solver, mc_rbdyn::Frame & frame);

  /** Remove all bounded speed constraints */
  void reset(QPSolver & solver);

  void addToSolver(QPSolver & solver) override;

  void removeFromSolver(QPSolver & solver) override;

  inline void update(QPSolver &) override {}

private:
  bool inSolver_ = false;

  struct BoundedSpeedData
  {
    BoundedSpeedData(mc_tvm::FrameVelocityPtr fn,
                     const Eigen::Vector6d & lowerSpeed,
                     const Eigen::Vector6d & upperSpeed);
    mc_tvm::FrameVelocityPtr fn;
    Eigen::Vector6d lowerSpeed;
    Eigen::Vector6d upperSpeed;
    tvm::TaskWithRequirementsPtr task;
  };
  std::vector<BoundedSpeedData> data_;

  std::vector<BoundedSpeedData>::iterator getData(const mc_rbdyn::Frame & frame);

  void addBoundedSpeed(QPSolver & solver, BoundedSpeedData & data);

  void updateBoundedSpeed(QPSolver & solver, BoundedSpeedData & data);

  void removeBoundedSpeed(QPSolver & solver, BoundedSpeedData & data);
};

using BoundedSpeedConstrPtr = std::shared_ptr<BoundedSpeedConstr>;

} // namespace mc_solver
