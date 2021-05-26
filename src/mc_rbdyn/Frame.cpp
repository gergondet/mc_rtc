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

Frame::Frame(ctor_token, std::string_view name, Robot & robot, std::string_view body, sva::PTransformd X_b_f)
: name_(name), robot_(robot), bodyId_(details::frameGetBodyId(name, robot, body)), jac_(robot.mb(), std::string(body)),
  X_b_f_(std::move(X_b_f)), jacTmp_(6, jac_.dof()), jacobian_(6, robot.mb().nrDof()), jacDot_(6, robot.mb().nrDof())
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

  addInternalDependency(Update::Velocity, Update::Position);  //for h_
  addInternalDependency(Update::Jacobian, Update::Position);  //for h_
  addInternalDependency(Update::NormalAcceleration, Update::Velocity);
  addInternalDependency(Update::JDot, Update::Velocity);
  // Not strictly true but they use the same internal variable, so in case the
  // graph gets parallelized and we start to use JDot...
  addInternalDependency(Update::JDot, Update::Jacobian);

  /** Initialize all data */
  updateAll();
}

Frame::Frame(ctor_token tkn, std::string_view name, const Frame & frame, sva::PTransformd X_f1_f2)
: Frame(tkn, name, frame.robot_, frame.body(), X_f1_f2 * frame.X_b_f_)
{
}

const std::string & Frame::body() const noexcept
{
  return robot_.mb().body(static_cast<int>(bodyId_)).name();
}


void Frame::updatePosition()
{
  const auto & X_0_b = robot_.mbc().bodyPosW[bodyId_];
  position_ = X_b_f_ * X_0_b;
  h_ = -hat(X_0_b.rotation().transpose() * X_b_f_.translation());
}

void Frame::updateJacobian()
{
  assert(jacobian_.rows() == 6 && jacobian_.cols() == robot_.mb().nrDof());
  const auto & partialJac = jac_.jacobian(robot_.mb(), robot_.mbc());
  jacTmp_ = partialJac;
  jacTmp_.bottomRows<3>().noalias() += h_ * partialJac.topRows<3>();
  jac_.fullJacobian(robot_.mb(), jacTmp_, jacobian_);
}

void Frame::updateVelocity()
{
  velocity_ = robot_.mbc().bodyVelW[bodyId_];
  velocity_.linear().noalias() += h_ * velocity_.angular();
}

void Frame::updateNormalAcceleration()
{
  normalAcceleration_ = jac_.normalAcceleration(robot_.mb(), robot_.mbc(), robot_.normalAccB());
  normalAcceleration_.linear().noalias() += h_ * normalAcceleration_.angular() + velocity_.angular().cross(h_*velocity_.angular());
}

void Frame::updateJDot()
{
  assert(jacDot_.rows() == 6 && jacDot_.cols() == robot_.mb().nrDof());
  const auto & partialJac = jac_.jacobianDot(robot_.mb(), robot_.mbc());
  jacTmp_ = partialJac;
  jacTmp_.bottomRows<3>().noalias() += h_ * partialJac.topRows<3>();
  jacTmp_.bottomRows<3>().noalias() -= hat(h_*velocity_.angular()) * jac_.jacobian(robot_.mb(), robot_.mbc()).topRows<3>();
  jac_.fullJacobian(robot_.mb(), jacTmp_, jacDot_);
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
