#include <mc_rbdyn/CoM.h>

#include <mc_rbdyn/Robot.h>

namespace mc_rbdyn
{

CoM::CoM(ctor_token, Robot & robot) : robot_(robot), jac_(robot_.mb())
{
  // clang-format off
  registerUpdates(
                  Update::CoM, &CoM::updateCoM,
                  Update::Jacobian, &CoM::updateJacobian,
                  Update::Velocity, &CoM::updateVelocity,
                  Update::NormalAcceleration, &CoM::updateNormalAcceleration,
                  Update::Acceleration, &CoM::updateAcceleration,
                  Update::JDot, &CoM::updateJDot);
  // clang-format off

  addOutputDependency(Output::CoM, Update::CoM);
  addInputDependency(Update::CoM, robot_, Robot::Output::FK);

  addOutputDependency(Output::Jacobian, Update::Jacobian);
  addInputDependency(Update::Jacobian, robot_, Robot::Output::FV);

  addOutputDependency(Output::Velocity, Update::Velocity);
  addInputDependency(Update::Velocity, robot_, Robot::Output::FV);

  addOutputDependency(Output::NormalAcceleration, Update::NormalAcceleration);
  addInputDependency(Update::NormalAcceleration, robot_, Robot::Output::NormalAcceleration);

  addOutputDependency(Output::Acceleration, Update::Acceleration);
  addInputDependency(Update::Acceleration, robot_, Robot::Output::FA);

  addOutputDependency(Output::JDot, Update::JDot);
  addInputDependency(Update::JDot, robot_, Robot::Output::FV);

  addInternalDependency(Update::Velocity, Update::Jacobian);
  addInternalDependency(Update::NormalAcceleration, Update::Jacobian);
}

void CoM::updateCoM()
{
  if(robot_.mass() > 0)
  {
    com_ = rbd::computeCoM(robot_.mb(), robot_.mbc());
  }
  else
  {
    com_ = robot_.posW().translation();
  }
}

void CoM::updateVelocity()
{
  velocity_ = jac_.velocity(robot_.mb(), robot_.mbc());
}

void CoM::updateNormalAcceleration()
{
  normalAcceleration_ = jac_.normalAcceleration(robot_.mb(), robot_.mbc(), robot_.normalAccB());
}

void CoM::updateAcceleration()
{
  if(robot_.mass() > 0)
  {
    acceleration_ = rbd::computeCoMAcceleration(robot_.mb(), robot_.mbc());
  }
  else
  {
    acceleration_ = Eigen::Vector3d::Zero();
  }
}

void CoM::updateJacobian()
{
  jac_.jacobian(robot_.mb(), robot_.mbc());
}

void CoM::updateJDot()
{
  jac_.jacobianDot(robot_.mb(), robot_.mbc());
}

} // namespace mc_rbdyn
