/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotFrame.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/hat.h>

namespace mc_rbdyn
{

namespace details
{

inline unsigned int frameGetBodyId(std::string_view frame, const Robot & robot, std::string_view body)
{
  if(!robot.hasBody(body))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Failed to create frame {}: no body named {} in {}", frame, body,
                                                     robot.name());
  }
  return robot.bodyIndexByName(body);
}
} // namespace details

RobotFrame::RobotFrame(ctor_token, std::string_view name, Robot & robot, std::string_view body)
: Frame(nullptr, name, sva::PTransformd::Identity()), robot_(robot),
  bodyId_(details::frameGetBodyId(name, robot, body)), jac_(robot.mb(), std::string(body)), jacTmp_(6, jac_.dof()),
  jacobian_(6, robot.mb().nrDof()), jacDot_(6, robot.mb().nrDof())
{
  // clang-format off
  registerUpdates(
                  Update::Position, &RobotFrame::updatePosition,
                  Update::Jacobian, &RobotFrame::updateJacobian,
                  Update::Velocity, &RobotFrame::updateVelocity,
                  Update::NormalAcceleration, &RobotFrame::updateNormalAcceleration,
                  Update::JDot, &RobotFrame::updateJDot);
  // clang-format off

  addOutputDependency(Output::Position, Update::Position);
  addInputDependency(Update::Position, robot_, Robot::Output::FK);

  addOutputDependency<RobotFrame>(Output::Jacobian, Update::Jacobian);
  addInputDependency<RobotFrame>(Update::Jacobian, robot_, Robot::Output::FV);

  addOutputDependency(Output::Velocity, Update::Velocity);
  addInputDependency(Update::Velocity, robot_, Robot::Output::FV);

  addOutputDependency<RobotFrame>(Output::NormalAcceleration, Update::NormalAcceleration);
  addInputDependency<RobotFrame>(Update::NormalAcceleration, robot_, Robot::Output::NormalAcceleration);

  addOutputDependency<RobotFrame>(Output::JDot, Update::JDot);
  addInputDependency<RobotFrame>(Update::JDot, robot_, Robot::Output::FV);

  addInternalDependency(Update::Velocity, Update::Position); // for h_
  addInternalDependency<RobotFrame>(Update::Jacobian, Update::Position); // for h_
  addInternalDependency<RobotFrame>(Update::NormalAcceleration, Update::Velocity);
  addInternalDependency<RobotFrame>(Update::JDot, Update::Velocity);
  // Not strictly true but they use the same internal variable, so in case the
  // graph gets parallelized and we start to use JDot...
  addInternalDependency<RobotFrame>(Update::JDot, Update::Jacobian);

  updatePosition();
  updateJacobian();
  updateVelocity();
  updateNormalAcceleration();
  updateJDot();
}

RobotFrame::RobotFrame(ctor_token, std::string_view name, RobotFrame & frame, sva::PTransformd X_f1_f2)
: Frame(frame, name, X_f1_f2), robot_(frame.robot()),
  bodyId_(details::frameGetBodyId(name, robot_, frame.body())), jac_(robot_.mb(), frame.body()),
  jacTmp_(6, jac_.dof()), jacobian_(6, robot_.mb().nrDof()), jacDot_(6, robot_.mb().nrDof())
{
  // clang-format off
  registerUpdates(
                  Update::Jacobian, &RobotFrame::updateJacobian,
                  Update::NormalAcceleration, &RobotFrame::updateNormalAcceleration,
                  Update::JDot, &RobotFrame::updateJDot);
  // clang-format off

  addOutputDependency<RobotFrame>(Output::Jacobian, Update::Jacobian);
  addInputDependency<RobotFrame>(Update::Jacobian, robot_, Robot::Output::FV);

  addOutputDependency<RobotFrame>(Output::NormalAcceleration, Update::NormalAcceleration);
  addInputDependency<RobotFrame>(Update::NormalAcceleration, robot_, Robot::Output::NormalAcceleration);

  addOutputDependency<RobotFrame>(Output::JDot, Update::JDot);
  addInputDependency<RobotFrame>(Update::JDot, robot_, Robot::Output::FV);

  addInternalDependency<RobotFrame>(Update::Jacobian, Update::Position); // for h_
  addInternalDependency<RobotFrame>(Update::NormalAcceleration, Update::Velocity);
  addInternalDependency<RobotFrame>(Update::JDot, Update::Velocity);
  // Not strictly true but they use the same internal variable, so in case the
  // graph gets parallelized and we start to use JDot...
  addInternalDependency<RobotFrame>(Update::JDot, Update::Jacobian);

  updateJacobian();
  updateNormalAcceleration();
  updateJDot();
}

const std::string & RobotFrame::body() const noexcept
{
  return robot_.mb().body(static_cast<int>(bodyId_)).name();
}

void RobotFrame::updatePosition()
{
  // Only called for body frames
  const auto & X_0_b = robot_.mbc().bodyPosW[bodyId_];
  position_ = X_0_b;
}

void RobotFrame::updateJacobian()
{
  assert(jacobian_.rows() == 6 && jacobian_.cols() == robot_.mb().nrDof());
  const auto & partialJac = jac_.jacobian(robot_.mb(), robot_.mbc());
  jacTmp_ = partialJac;
  jacTmp_.bottomRows<3>().noalias() += h_ * partialJac.topRows<3>();
  jac_.fullJacobian(robot_.mb(), jacTmp_, jacobian_);
}

void RobotFrame::updateVelocity()
{
  // Only called for body frames
  velocity_ = robot_.mbc().bodyVelW[bodyId_];
}

void RobotFrame::updateNormalAcceleration()
{
  normalAcceleration_ = jac_.normalAcceleration(robot_.mb(), robot_.mbc(), robot_.normalAccB());
  normalAcceleration_.linear().noalias() += h_ * normalAcceleration_.angular() + velocity_.angular().cross(h_*velocity_.angular());
}

void RobotFrame::updateJDot()
{
  assert(jacDot_.rows() == 6 && jacDot_.cols() == robot_.mb().nrDof());
  const auto & partialJac = jac_.jacobianDot(robot_.mb(), robot_.mbc());
  jacTmp_ = partialJac;
  jacTmp_.bottomRows<3>().noalias() += h_ * partialJac.topRows<3>();
  jacTmp_.bottomRows<3>().noalias() -= hat(h_*velocity_.angular()) * jac_.jacobian(robot_.mb(), robot_.mbc()).topRows<3>();
  jac_.fullJacobian(robot_.mb(), jacTmp_, jacDot_);
}

} // namespace mc_rbdyn

