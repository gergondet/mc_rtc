/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/PositionFunction.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

PositionFunction::PositionFunction(mc_rbdyn::Frame & frame) : tvm::function::abstract::Function(3), frame_(frame)
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &PositionFunction::updateValue,
                  Update::Velocity, &PositionFunction::updateVelocity,
                  Update::Jacobian, &PositionFunction::updateJacobian,
                  Update::NormalAcceleration, &PositionFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<PositionFunction>(Output::Value, Update::Value);
  addOutputDependency<PositionFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<PositionFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<PositionFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  const auto & robot = frame_->robot();
  addVariable(robot.q(), false);
  addInputDependency<PositionFunction>(Update::Value, frame_, mc_rbdyn::Frame::Output::Position);
  addInputDependency<PositionFunction>(Update::Velocity, frame_, mc_rbdyn::Frame::Output::Velocity);
  addInputDependency<PositionFunction>(Update::Jacobian, frame_, mc_rbdyn::Frame::Output::Jacobian);
  addInputDependency<PositionFunction>(Update::NormalAcceleration, frame_, mc_rbdyn::Frame::Output::NormalAcceleration);
}

void PositionFunction::reset()
{
  pos_ = frame_->position().translation();
}

void PositionFunction::updateValue()
{
  value_ = frame_->position().translation() - pos_;
}

void PositionFunction::updateVelocity()
{
  velocity_ = frame_->velocity().linear() - refVel_;
}

void PositionFunction::updateJacobian()
{
  splitJacobian(frame_->jacobian().bottomRows<3>(), frame_->robot().q());
}

void PositionFunction::updateNormalAcceleration()
{
  normalAcceleration_ = frame_->normalAcceleration().linear();
}

} // namespace mc_tvm
