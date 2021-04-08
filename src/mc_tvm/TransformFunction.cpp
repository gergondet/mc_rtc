/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/TransformFunction.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

TransformFunction::TransformFunction(mc_rbdyn::Frame & frame)
: tvm::function::abstract::Function(6), frame_(frame), frameJac_(frame.rbdJacobian()), shortJacMat_(6, frameJac_.dof()),
  jacMat_(6, frame.robot().mb().nrDof())
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
  addInternalDependency<TransformFunction>(Update::Velocity, Update::Value);
  addInternalDependency<TransformFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<TransformFunction>(Update::NormalAcceleration, Update::Velocity);
}

void TransformFunction::reset()
{
  pose_ = frame_->position();
  refVel_.setZero();
  refAccel_.setZero();
}

void TransformFunction::updateValue()
{
  err_p_ = sva::transformVelocity(frame_->position() * pose_.inv());
  value_ = err_p_.vector();
}

void TransformFunction::updateVelocity()
{
  const auto & robot = frame().robot();
  sva::MotionVecd V_p_p = frameJac_.velocity(robot.mb(), robot.mbc(), frame().X_b_f());
  w_p_p_ = {V_p_p.angular(), Eigen::Vector3d::Zero()};
  V_err_p_ = V_p_p - err_p_.cross(w_p_p_);
  velocity_ = V_err_p_.vector() - refVel_;
}

void TransformFunction::updateJacobian()
{
  const auto & robot = frame().robot();
  shortJacMat_ = frameJac_.jacobian(robot.mb(), robot.mbc(), frame().position());
  for(int i = 0; i < frameJac_.dof(); ++i)
  {
    shortJacMat_.col(i).head<6>() -=
        err_p_.cross(sva::MotionVecd(shortJacMat_.col(i).head<3>(), Eigen::Vector3d::Zero())).vector();
  }
  frameJac_.fullJacobian(robot.mb(), shortJacMat_, jacMat_);
  splitJacobian(jacMat_, frame_->robot().q());
}

void TransformFunction::updateNormalAcceleration()
{
  const auto & robot = frame().robot();
  sva::MotionVecd AN_p_p = frameJac_.normalAcceleration(robot.mb(), robot.mbc(), robot.normalAccB(), frame().X_b_f(),
                                                        sva::MotionVecd::Zero());
  sva::MotionVecd wAN_p_p = sva::MotionVecd(AN_p_p.angular(), Eigen::Vector3d::Zero());
  normalAcceleration_ = (AN_p_p - V_err_p_.cross(w_p_p_) - err_p_.cross(wAN_p_p)).vector() - refAccel_;
}

} // namespace mc_tvm
