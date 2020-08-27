#include <mc_rbdyn/Momentum.h>

#include <mc_rbdyn/Robot.h>

namespace mc_rbdyn
{

Momentum::Momentum(ctor_token, CoM & com) : com_(com), mat_(robot().mb())
{
  // clang-format off
  registerUpdates(
                  Update::Momentum, &Momentum::updateMomentum,
                  Update::Jacobian, &Momentum::updateJacobian,
                  Update::NormalAcceleration, &Momentum::updateNormalAcceleration,
                  Update::JDot, &Momentum::updateJDot);
  // clang-format off

  addOutputDependency(Output::Momentum, Update::Momentum);
  addInputDependency(Update::Momentum, robot(), Robot::Output::FK);
  addInputDependency(Update::Momentum, com_, CoM::Output::CoM);

  addOutputDependency(Output::Jacobian, Update::Jacobian);
  addInputDependency(Update::Jacobian, robot(), Robot::Output::FV);
  addInputDependency(Update::Jacobian, com_, CoM::Output::CoM);

  addOutputDependency(Output::NormalAcceleration, Update::NormalAcceleration);
  addInputDependency(Update::NormalAcceleration, robot(), Robot::Output::NormalAcceleration);
  addInputDependency(Update::NormalAcceleration, com_, CoM::Output::Velocity);

  addOutputDependency(Output::JDot, Update::JDot);
  addInputDependency(Update::JDot, robot(), Robot::Output::FV);
  addInputDependency(Update::JDot, com_, CoM::Output::Velocity);

  addInternalDependency(Update::NormalAcceleration, Update::Jacobian);

  updateMomentum();
  updateJacobian();
  updateNormalAcceleration();
  updateJDot();
}

void Momentum::updateMomentum()
{
  momentum_ = rbd::computeCentroidalMomentum(robot().mb(), robot().mbc(), com_.com());
}

void Momentum::updateNormalAcceleration()
{
  normalAcceleration_ = mat_.normalMomentumDot(robot().mb(), robot().mbc(), com_.com(), com_.velocity());
}

void Momentum::updateJacobian()
{
  mat_.computeMatrix(robot().mb(), robot().mbc(), com_.com());
}

void Momentum::updateJDot()
{
  mat_.computeMatrixDot(robot().mb(), robot().mbc(), com_.com(), com_.velocity());
}

} // namespace mc_rbdyn
