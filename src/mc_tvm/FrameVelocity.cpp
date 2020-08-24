/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/FrameVelocity.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

FrameVelocity::FrameVelocity(mc_rbdyn::Frame & frame, const Eigen::Vector6d & dof)
: tvm::function::abstract::Function(6), frame_(frame), dof_(dof),
  jac_(rbd::Jacobian(frame.robot().mb(), frame.body(), frame.X_b_f().translation())),
  jacobian_(Eigen::MatrixXd::Zero(6, frame.robot().mb().nrDof()))
{
  // clang-format off
  registerUpdates(Update::Value, &FrameVelocity::updateValue,
                  Update::Velocity, &FrameVelocity::updateVelocity,
                  Update::Jacobian, &FrameVelocity::updateJacobian);
  // clang-format on
  addOutputDependency<FrameVelocity>(Output::Value, Update::Value);
  addOutputDependency<FrameVelocity>(Output::Velocity, Update::Velocity);
  addOutputDependency<FrameVelocity>(Output::Jacobian, Update::Jacobian);
  addVariable(tvm::dot(frame.robot().q()), false);
  addInputDependency<FrameVelocity>(Update::Value, frame.robot(), mc_rbdyn::Robot::Output::FV);
  addInputDependency<FrameVelocity>(Update::Jacobian, frame.robot(), mc_rbdyn::Robot::Output::FV);
  addInputDependency<FrameVelocity>(Update::Velocity, frame.robot(), mc_rbdyn::Robot::Output::NormalAcceleration);

  // Make all values up-to-date on creation
  updateValue();
  updateVelocity();
  updateJacobian();
}

void FrameVelocity::updateValue()
{
  value_ = dof_.cwiseProduct(jac_.bodyVelocity(frame_->robot().mb(), frame_->robot().mbc()).vector());
}

void FrameVelocity::updateJacobian()
{
  const auto & jac = jac_.bodyJacobian(frame_->robot().mb(), frame_->robot().mbc());
  jac_.fullJacobian(frame_->robot().mb(), dof_.asDiagonal() * jac, jacobian_);
  splitJacobian(jacobian_, tvm::dot(frame_->robot().q()));
}

void FrameVelocity::updateVelocity()
{
  velocity_ = dof_.cwiseProduct(
      jac_.bodyNormalAcceleration(frame_->robot().mb(), frame_->robot().mbc(), frame_->robot().normalAccB()).vector());
}

} // namespace mc_tvm
