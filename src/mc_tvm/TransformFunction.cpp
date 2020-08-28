/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/TransformFunction.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

TransformFunction::TransformFunction(mc_rbdyn::Frame & frame) : tvm::function::abstract::Function(6), frame_(frame)
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &TransformFunction::updateValue,
                  Update::Velocity, &TransformFunction::updateVelocity,
                  Update::Jacobian, &TransformFunction::updateJacobian,
                  Update::NormalAcceleration, &TransformFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<TransformFunction>(Output::Value, Update::Value);
  addOutputDependency<TransformFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<TransformFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<TransformFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  const auto & robot = frame_->robot();
  addVariable(robot.q(), false);
  addInputDependency<TransformFunction>(Update::Value, frame_, mc_rbdyn::Frame::Output::Position);
  addInputDependency<TransformFunction>(Update::Velocity, frame_, mc_rbdyn::Frame::Output::Velocity);
  addInputDependency<TransformFunction>(Update::Jacobian, frame_, mc_rbdyn::Frame::Output::Jacobian);
  addInputDependency<TransformFunction>(Update::NormalAcceleration, frame_,
                                        mc_rbdyn::Frame::Output::NormalAcceleration);
}

void TransformFunction::reset()
{
  pose_ = frame_->position();
  refVel_.setZero();
  refAccel_.setZero();
}

void TransformFunction::updateValue()
{
  value_ = sva::transformError(pose_, frame_->position()).vector();
}

void TransformFunction::updateVelocity()
{
  velocity_ = frame_->velocity().vector() - refVel_;
}

void TransformFunction::updateJacobian()
{
  splitJacobian(frame_->jacobian(), frame_->robot().q());
}

void TransformFunction::updateNormalAcceleration()
{
  normalAcceleration_ = frame_->normalAcceleration().vector();
}

} // namespace mc_tvm
