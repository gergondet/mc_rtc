/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/KinematicsConstraint.h>

#include <mc_solver/QPSolver.h>

#include <mc_rbdyn/Robot.h>

#include <tvm/hint/internal/DiagonalCalculator.h>
#include <tvm/task_dynamics/Proportional.h>
#include <tvm/task_dynamics/VelocityDamper.h>

namespace mc_solver
{

KinematicsConstraint::KinematicsConstraint(mc_rbdyn::Robot & robot,
                                           const std::array<double, 3> & damper,
                                           double velocityPercent)
: robot_(robot), damper_(damper), velocityPercent_(velocityPercent)
{
  name_ = fmt::format("KinematicsConstraint::{}", robot.name());
  if(damper_[0] <= damper_[1])
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Wrong damper configuration in {}\nThe interaction distance must be strictly higher than the safety distance",
        name_);
  }
  if(damper_[2] < 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Wrong damper configuration in {}\nThe damping offset must be positive", name_);
  }
}

void KinematicsConstraint::addToSolver(QPSolver & solver)
{
  if(!constraints_.empty())
  {
    return;
  }
  /** Joints limits */
  int startParam = robot_->qFloatingBase()->size();
  int startDof = robot_->qFloatingBase()->space().tSize();
  auto nParams = robot_->qJoints()->size();
  auto ql = robot_->limits().ql.segment(startParam, nParams);
  auto qu = robot_->limits().qu.segment(startParam, nParams);
  auto jl = solver.problem().add(
      ql <= robot_->qJoints() <= qu,
      tvm::task_dynamics::VelocityDamper(solver.dt(), {damper_[0] * (qu - ql), damper_[1] * (qu - ql),
                                                       Eigen::VectorXd::Constant(nParams, 1, 0),
                                                       Eigen::VectorXd::Constant(nParams, 1, damper_[2])}),
      {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(jl);
  /** Velocity limits */
  auto nDof = robot_->qJoints()->space().tSize();
  auto vl = robot_->limits().vl.segment(startDof, nDof) * velocityPercent_ / solver.dt();
  auto vu = robot_->limits().vu.segment(startDof, nDof) * velocityPercent_ / solver.dt();
  auto vL =
      solver.problem().add(vl <= tvm::dot(robot_->qJoints()) <= vu, tvm::task_dynamics::Proportional(1 / solver.dt()),
                           {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(vL);
  /** Acceleration limits */
  auto al = robot_->limits().al.segment(startDof, nDof);
  auto au = robot_->limits().au.segment(startDof, nDof);
  auto aL = solver.problem().add(al <= tvm::dot(robot_->qJoints(), 2) <= au, tvm::task_dynamics::None{},
                                 {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(aL);
  /** Mimic constraints */
  for(const auto & m : robot_->mimics())
  {
    Eigen::DenseIndex startIdx = 0;
    const auto & leader = m.first;
    const auto & followers = m.second.first;
    const auto & mimicMult = m.second.second;
    for(const auto & f : followers)
    {
      Eigen::MatrixXd A = mimicMult.segment(startIdx, f->size());
      auto mimic = solver.problem().add(A * tvm::dot(leader, 2) - tvm::dot(f, 2) == 0., tvm::task_dynamics::None{},
                                        {tvm::requirements::PriorityLevel(0)});
      // solver.problem().add(tvm::hint::Substitution(solver.problem().constraint(mimic.get()), tvm::dot(f, 2),
      //                                             tvm::constant::fullRank,
      //                                             tvm::hint::internal::DiagonalCalculator{}));
      constraints_.push_back(mimic);
      startIdx += f->size();
    }
  }
}

void KinematicsConstraint::removeFromSolver(QPSolver & solver)
{
  if(constraints_.empty())
  {
    return;
  }
  for(auto & c : constraints_)
  {
    solver.problem().remove(c.get());
  }
  constraints_.clear();
}

} // namespace mc_solver
