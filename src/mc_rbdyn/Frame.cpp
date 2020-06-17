/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Frame.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/hat.h>

namespace mc_rbdyn
{

namespace details
{

inline unsigned int frameGetBodyId(std::string_view frame, const RobotPtr & robot, std::string_view body)
{
  if(!robot->hasBody(body))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Failed to create frame {}: no body named {} in {}", frame, body,
                                                     robot->name());
  }
}
} // namespace details

Frame::Frame(ctor_token, std::string_view name, RobotPtr robot, std::string_view body, sva::PTransformd X_b_f)
: name_(name), robot_(robot), bodyId_(details::frameGetBodyId(name, robot, body)), jac_(robot->mb(), std::string(body)),
  X_b_f_(std::move(X_b_f)), jacTmp_(6, jac_.dof()), jacobian_(6, robot->mb().nrDof()), jacDot_(6, robot->mb().nrDof())
{
  // clang-format off
  registerUpdates(
                  Update::Position, &Frame::updatePosition,
                  Update::Jacobian, &Frame::updateJacobian,
                  Update::Velocity, &Frame::updateVelocity,
                  Update::NormalAcceleration, &Frame::updateNormalAcceleration,
                  Update::JDot, &Frame::updateJDot);
  // clang-format off

  addOutputDependency(Output::Position, Update::Position);
  addInputDependency(Update::Position, robot_, Robot::Output::FK);

  addOutputDependency(Output::Jacobian, Update::Jacobian);
  addInternalDependency(Update::Jacobian, Update::Position);
  addInputDependency(Update::Jacobian, robot_, Robot::Output::FV);

  addOutputDependency(Output::Velocity, Update::Velocity);
  addInputDependency(Update::Velocity, robot_, Robot::Output::FV);

  addOutputDependency(Output::NormalAcceleration, Update::NormalAcceleration);
  addInputDependency(Update::NormalAcceleration, robot_, Robot::Output::NormalAcceleration);

  addOutputDependency(Output::JDot, Update::JDot);
  addInputDependency(Update::JDot, robot_, Robot::Output::FV);

  // Not strictly true but they use the same internal variable, so in case the
  // graph gets parallelized and we start to use JDot...
  addInternalDependency(Update::JDot, Update::Jacobian);

  /** Initialize all data */
  updateAll();
}

Frame::Frame(ctor_token tkn, std::string_view name, ConstFramePtr frame, sva::PTransformd X_f1_f2)
: Frame(tkn, name, frame->robot_, frame->body(), X_f1_f2 * frame->X_b_f_)
{
}

const std::string & Frame::body() const noexcept
{
  return robot_->mb().body(static_cast<int>(bodyId_)).name();
}


void Frame::updatePosition()
{
  const auto & X_0_b = robot_->mbc().bodyPosW[bodyId_];
  position_ = X_b_f_ * X_0_b;
  h_ = -hat(X_0_b.rotation().transpose() * X_b_f_.translation());
}

void Frame::updateJacobian()
{
  assert(jacobian_.rows() == 6 && jacobian_.cols() == robot_->mb().nrDof());
  const auto & partialJac = jac_.jacobian(robot_->mb(), robot_->mbc());
  jacTmp_ = partialJac;
  jacTmp_.block(3, 0, 3, jac_.dof()).noalias() += h_ * partialJac.block(3, 0, 3, jac_.dof());
  jac_.fullJacobian(robot_->mb(), jacTmp_, jacobian_);

}

void Frame::updateVelocity()
{
  velocity_ = X_b_f_ * robot_->mbc().bodyVelW[bodyId_];
}

void Frame::updateNormalAcceleration()
{
  normalAcceleration_ = X_b_f_ * jac_.normalAcceleration(robot_->mb(), robot_->mbc(), robot_->normalAccB());
}

void Frame::updateJDot()
{
  assert(jacobian_.rows() == 6 && jacobian_.cols() == robot_->mb().nrDof());
  const auto & partialJac = jac_.jacobianDot(robot_->mb(), robot_->mbc());
  jacTmp_ = partialJac;
  jacTmp_.block(3, 0, 3, jac_.dof()).noalias() += h_ * partialJac.block(3, 0, 3, jac_.dof());
  jac_.fullJacobian(robot_->mb(), jacTmp_, jacDot_);

}

void Frame::updateAll()
{
  updatePosition();
  updateJacobian();
  updateVelocity();
  updateNormalAcceleration();
  updateJDot();
}

} // namespace mc_rbdyn
