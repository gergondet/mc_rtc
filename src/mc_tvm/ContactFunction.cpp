/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/ContactFunction.h>

#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

ContactFunction::ContactFunction(mc_rbdyn::FramePtr f1, mc_rbdyn::FramePtr f2, const Eigen::Vector6d & dof)
: tvm::function::abstract::Function(6), dof_(dof)
{
  registerUpdates(Update::Value, &ContactFunction::updateValue, Update::Derivatives,
                  &ContactFunction::updateDerivatives);

  addOutputDependency<ContactFunction>(Output::Value, Update::Value);
  addOutputDependency<ContactFunction>(Output::Velocity, Update::Derivatives);
  addOutputDependency<ContactFunction>(Output::NormalAcceleration, Update::Derivatives);
  addOutputDependency<ContactFunction>(Output::Jacobian, Update::Derivatives);

  auto addRobot = [this](mc_rbdyn::FramePtr & fIn, mc_rbdyn::FramePtr & fOut, bool & useF, rbd::Jacobian & jac) {
    fOut = fIn;
    if(fIn->rbdJacobian().dof() > 0)
    {
      const auto & r = fIn->robot();
      useF = true;
      jac = fIn->rbdJacobian();
      addInputDependency<ContactFunction>(Update::Value, fOut, mc_rbdyn::Frame::Output::Position);
      addInputDependency<ContactFunction>(Update::Derivatives, fOut, mc_rbdyn::Frame::Output::Velocity);
      addInputDependency<ContactFunction>(Update::Derivatives, fOut, mc_rbdyn::Frame::Output::NormalAcceleration);
      addInputDependency<ContactFunction>(Update::Derivatives, fOut, mc_rbdyn::Frame::Output::Jacobian);
      addVariable(r.q(), false);
    }
    else
    {
      useF = false;
      addInputDependency<ContactFunction>(Update::Value, fOut, mc_rbdyn::Frame::Output::Position);
    }
  };
  addRobot(f1, f1_, use_f1_, f1Jacobian_);
  addRobot(f2, f2_, use_f2_, f2Jacobian_);
  X_f1_f2_init_ = f2->position() * f1->position().inv();

  jacTmp_.resize(6, std::max(f1_->rbdJacobian().dof(), f2_->rbdJacobian().dof()));
  jac_.resize(6, std::max(f1_->robot().mb().nrDof(), f2_->robot().mb().nrDof()));
}

void ContactFunction::updateValue()
{
  const auto & X_0_f1cf = f1_->position();
  const auto & X_0_f2cf = X_f1_f2_init_.inv() * f2_->position();
  const auto & X_f1cf_f2cf = X_0_f2cf * X_0_f1cf.inv();
  value_.head<3>() = dof_.head<3>().asDiagonal() * sva::rotationVelocity(X_f1cf_f2cf.rotation());
  value_.tail<3>() = dof_.tail<3>().asDiagonal() * X_f1cf_f2cf.translation();
}

void ContactFunction::updateDerivatives()
{
  velocity_.setZero();
  normalAcceleration_.setZero();
  for(const auto & var : variables())
  {
    jacobian_[var.get()].setZero();
  }
  auto updateDerivatives = [this](const mc_rbdyn::Frame & frame, rbd::Jacobian & jac, double sign) {
    const auto & robot = frame.robot();
    const auto & mb = robot.mb();
    const auto & mbc = robot.mbc();
    const auto & NAB = robot.normalAccB();

    const auto & X_0_f = frame.position();
    const auto & jacMat = jac.jacobian(mb, mbc, X_0_f);
    jacTmp_.block(0, 0, 6, jac.dof()).noalias() = sign * dof_.asDiagonal() * jacMat;
    jac.fullJacobian(mb, jacTmp_.block(0, 0, 6, jac.dof()), jac_);
    jacobian_[robot.q().get()] += jac_.block(0, 0, 6, mb.nrDof());

    normalAcceleration_ += sign * dof_.asDiagonal()
                           * (jac.normalAcceleration(mb, mbc, NAB, frame.X_b_f(), sva::MotionVecd::Zero()).vector());

    velocity_ += sign * dof_.asDiagonal() * jac.velocity(mb, mbc, frame.X_b_f()).vector();
  };
  if(use_f1_)
  {
    updateDerivatives(*f1_, f1Jacobian_, 1.0);
  }
  if(use_f2_)
  {
    updateDerivatives(*f2_, f2Jacobian_, -1.0);
  }
}

} // namespace mc_tvm
