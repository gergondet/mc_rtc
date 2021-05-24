/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/CoMFunction.h>

#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

CoMFunction::CoMFunction(mc_rbdyn::RobotPtr robot) : tvm::function::abstract::Function(3), comAlgo_(robot->com())
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &CoMFunction::updateValue,
                  Update::Velocity, &CoMFunction::updateVelocity,
                  Update::Jacobian, &CoMFunction::updateJacobian,
                  Update::NormalAcceleration, &CoMFunction::updateNormalAcceleration,
                  Update::JDot, &CoMFunction::updateJDot);
  // clang-format on
  addOutputDependency<CoMFunction>(Output::Value, Update::Value);
  addOutputDependency<CoMFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<CoMFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<CoMFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  addOutputDependency<CoMFunction>(Output::JDot, Update::JDot);
  addVariable(robot->q(), false);
  addInputDependency<CoMFunction>(Update::Value, comAlgo_, mc_rbdyn::CoM::Output::CoM);
  addInputDependency<CoMFunction>(Update::Velocity, comAlgo_, mc_rbdyn::CoM::Output::Velocity);
  addInputDependency<CoMFunction>(Update::Jacobian, comAlgo_, mc_rbdyn::CoM::Output::Jacobian);
  addInputDependency<CoMFunction>(Update::NormalAcceleration, comAlgo_, mc_rbdyn::CoM::Output::NormalAcceleration);
  addInputDependency<CoMFunction>(Update::JDot, comAlgo_, mc_rbdyn::CoM::Output::JDot);
}

void CoMFunction::reset()
{
  com_ = comAlgo_->com();
  refVel_.setZero();
  refAccel_.setZero();
}

void CoMFunction::updateValue()
{
  value_ = comAlgo_->com() - com_;
  refVel_.setZero();
  refAccel_.setZero();
}

void CoMFunction::updateVelocity()
{
  velocity_ = comAlgo_->velocity() - refVel_;
}

void CoMFunction::updateJacobian()
{
  splitJacobian(comAlgo_->jacobian(), comAlgo_->robot().q());
}

void CoMFunction::updateNormalAcceleration()
{
  normalAcceleration_ = comAlgo_->normalAcceleration();
}

void CoMFunction::updateJDot()
{
  splitJacobian(comAlgo_->JDot(), comAlgo_->robot().q());
}

} // namespace mc_tvm
