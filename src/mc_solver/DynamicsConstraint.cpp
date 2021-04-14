/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/DynamicsConstraint.h>

#include <mc_solver/ConstraintLoader.h>

#include <tvm/task_dynamics/None.h>

namespace mc_solver
{

DynamicsConstraint::DynamicsConstraint(mc_rbdyn::Robot & robot,
                                       const std::array<double, 3> & damper,
                                       double velocityPercent)
: KinematicsConstraint(robot, damper, velocityPercent), dynamic_(std::make_shared<mc_tvm::DynamicFunction>(robot_))
{
  name_ = fmt::format("DynamicsConstraint::{}", robot.name());
}

void DynamicsConstraint::addToSolver(QPSolver & solver)
{
  if(!constraints_.empty())
  {
    return;
  }
  KinematicsConstraint::addToSolver(solver);
  auto dyn = solver.problem().add(dynamic_ == 0., tvm::task_dynamics::None(), {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(dyn);
  auto cstr = solver.problem().constraint(*dyn);
  solver.problem().add(tvm::hint::Substitution(cstr, robot_->tau()));
  auto tL = solver.problem().add(robot_->limits().tl <= robot_->tau() <= robot_->limits().tu,
                                 tvm::task_dynamics::None(), {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(tL);
  // FIXME Add torque derivative limits
}

void DynamicsConstraint::removeFromSolver(QPSolver & solver)
{
  if(constraints_.empty())
  {
    return;
  }
  // FIXME If we don't have to remove the substitution hint then KinematicsConstraint base is enough
  KinematicsConstraint::removeFromSolver(solver);
}

} // namespace mc_solver

namespace
{

template<typename T>
mc_solver::ConstraintPtr load_constr(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  auto & robot = solver.robots().fromConfig(config, "Kinematics|DynamicsConstraint");
  return std::make_shared<T>(robot, config("damper"), config("velocityPercent", 0.5));
}

static auto kin_registered =
    mc_solver::ConstraintLoader::register_load_function("kinematics", &load_constr<mc_solver::KinematicsConstraint>);
static auto dyn_registered =
    mc_solver::ConstraintLoader::register_load_function("dynamics", &load_constr<mc_solver::DynamicsConstraint>);

} // namespace
