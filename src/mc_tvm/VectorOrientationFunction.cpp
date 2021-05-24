/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/VectorOrientationFunction.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/hat.h>

namespace mc_tvm
{

VectorOrientationFunction::VectorOrientationFunction(mc_rbdyn::Frame & frame, const Eigen::Vector3d & frameVector)
: tvm::function::abstract::Function(3), frame_(frame), jac_(frame_->rbdJacobian()),
  fullJacobian_(3, frame_->robot().mb().nrDof())
{
  this->frameVector(frameVector);
  reset();
  // clang-format off
  registerUpdates(Update::Value, &VectorOrientationFunction::updateValue,
                  Update::Velocity, &VectorOrientationFunction::updateVelocity,
                  Update::Jacobian, &VectorOrientationFunction::updateJacobian,
                  Update::NormalAcceleration, &VectorOrientationFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<VectorOrientationFunction>(Output::Value, Update::Value);
  addOutputDependency<VectorOrientationFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<VectorOrientationFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<VectorOrientationFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  addVariable(frame_->robot().q(), false);
  addInputDependency<VectorOrientationFunction>(Update::Value, frame_, mc_rbdyn::Frame::Output::Position);
  addInputDependency<VectorOrientationFunction>(Update::Velocity, frame_, mc_rbdyn::Frame::Output::Velocity);
  addInputDependency<VectorOrientationFunction>(Update::Jacobian, frame_, mc_rbdyn::Frame::Output::Jacobian);
  addInputDependency<VectorOrientationFunction>(Update::NormalAcceleration, frame_,
                                                mc_rbdyn::Frame::Output::NormalAcceleration);
  addInternalDependency<VectorOrientationFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<VectorOrientationFunction>(Update::Velocity, Update::Jacobian);
  addInternalDependency<VectorOrientationFunction>(Update::NormalAcceleration, Update::Velocity);
}

void VectorOrientationFunction::frameVector(const Eigen::Vector3d & frameVector) noexcept
{
  frameVectorIn_ = frameVector;
  Eigen::Matrix3d E_b_f = frame_->X_b_f().rotation().transpose();
  bodyVector_ = E_b_f * frameVector;
  bodyVectorHat_ = mc_rbdyn::hat(bodyVector_);
}

void VectorOrientationFunction::reset()
{
  const auto & robot = frame_->robot();
  Eigen::Matrix3d E_0_b = robot.mbc().bodyPosW[robot.bodyIndexByName(frame_->body())].rotation().transpose();
  target_ = E_0_b * bodyVector_;
  refVel_.setZero();
  refAccel_.setZero();
}

void VectorOrientationFunction::updateValue()
{
  const auto & robot = frame_->robot();
  E_0_b_ = robot.mbc().bodyPosW[robot.bodyIndexByName(frame_->body())].rotation().transpose();
  actualVector_ = E_0_b_ * bodyVector_;
  value_ = actualVector_ - target_;
}

void VectorOrientationFunction::updateVelocity()
{
  const auto & robot = frame_->robot();
  w_b_b_ = jac_.bodyVelocity(robot.mb(), robot.mbc()).angular();
  velocity_ = E_0_b_ * (w_b_b_.cross(bodyVector_)) - refVel_;
}

void VectorOrientationFunction::updateJacobian()
{
  const auto & robot = frame_->robot();
  const auto & jac = jac_.bodyJacobian(robot.mb(), robot.mbc());
  jac_.fullJacobian(robot.mb(), jac.block(0, 0, 3, jac_.dof()), fullJacobian_);
  fullJacobian_ = -E_0_b_ * bodyVectorHat_ * fullJacobian_;
  splitJacobian(fullJacobian_, robot.q());
}

void VectorOrientationFunction::updateNormalAcceleration()
{
  const auto & robot = frame_->robot();
  Eigen::Vector3d bodyNormalAcc = jac_.bodyNormalAcceleration(robot.mb(), robot.mbc(), robot.normalAccB()).angular();
  normalAcceleration_ =
      E_0_b_ * (w_b_b_.cross(w_b_b_.cross(bodyVector_)) + bodyNormalAcc.cross(bodyVector_)) - refAccel_;
}

} // namespace mc_tvm
