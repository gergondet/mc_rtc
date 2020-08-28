/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/OrientationFunction.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

OrientationFunction::OrientationFunction(mc_rbdyn::Frame & frame) : tvm::function::abstract::Function(3), frame_(frame)
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &OrientationFunction::updateValue,
                  Update::Velocity, &OrientationFunction::updateVelocity,
                  Update::Jacobian, &OrientationFunction::updateJacobian,
                  Update::NormalAcceleration, &OrientationFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<OrientationFunction>(Output::Value, Update::Value);
  addOutputDependency<OrientationFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<OrientationFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<OrientationFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  const auto & robot = frame_->robot();
  addVariable(robot.q(), false);
  addInputDependency<OrientationFunction>(Update::Value, frame_, mc_rbdyn::Frame::Output::Position);
  addInputDependency<OrientationFunction>(Update::Velocity, frame_, mc_rbdyn::Frame::Output::Velocity);
  addInputDependency<OrientationFunction>(Update::Jacobian, frame_, mc_rbdyn::Frame::Output::Jacobian);
  addInputDependency<OrientationFunction>(Update::NormalAcceleration, frame_,
                                          mc_rbdyn::Frame::Output::NormalAcceleration);
}

void OrientationFunction::reset()
{
  ori_ = frame_->position().rotation();
}

void OrientationFunction::updateValue()
{
  value_ = sva::rotationError(ori_, frame_->position().rotation());
}

void OrientationFunction::updateVelocity()
{
  velocity_ = frame_->velocity().angular() - refVel_;
}

void OrientationFunction::updateJacobian()
{
  splitJacobian(frame_->jacobian().topRows<3>(), frame_->robot().q());
}

void OrientationFunction::updateNormalAcceleration()
{
  normalAcceleration_ = frame_->normalAcceleration().angular();
}

} // namespace mc_tvm
