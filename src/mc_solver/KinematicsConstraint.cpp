/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/KinematicsConstraint.h>

#include <mc_solver/QPSolver.h>

#include <mc_rbdyn/Robot.h>

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
  auto nParams = robot_->qJoints()->size();
  auto ql = robot_->limits().ql.tail(nParams);
  auto qu = robot_->limits().qu.tail(nParams);
  auto jl = solver.problem().add(
      ql <= robot_->qJoints() <= qu,
      tvm::task_dynamics::VelocityDamper(solver.dt(), {damper_[0] * (qu - ql), damper_[1] * (qu - ql),
                                                       Eigen::VectorXd::Constant(nParams, 1, 0),
                                                       Eigen::VectorXd::Constant(nParams, 1, damper_[2])}),
      {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(jl);
  /** Velocity limits */
  auto nDof = robot_->qJoints()->space().tSize();
  auto vl = robot_->limits().vl.tail(nDof) * velocityPercent_ / solver.dt();
  auto vu = robot_->limits().vu.tail(nDof) * velocityPercent_ / solver.dt();
  auto vL = solver.problem().add(vl <= dot(robot_->qJoints()) <= vu, tvm::task_dynamics::Proportional(1 / solver.dt()),
                                 {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(vL);
  auto al = robot_->limits().al.tail(nDof);
  auto au = robot_->limits().au.tail(nDof);
  auto aL = solver.problem().add(al <= dot(robot_->qJoints(), 2) <= au, tvm::task_dynamics::None{}, {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(aL);
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
