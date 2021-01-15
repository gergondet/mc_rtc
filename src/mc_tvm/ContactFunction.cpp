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
  auto X_f1_f2 = f2_->position() * f1_->position().inv();
  value_ = dof_.asDiagonal() * sva::transformError(X_f1_f2, X_f1_f2_init_).vector();
}

void ContactFunction::updateDerivatives()
{
  const auto & r1 = f1_->robot();
  const auto & mb1 = r1.mb();
  const auto & mbc1 = r1.mbc();
  const auto & NAB1 = r1.normalAccB();
  const auto & r2 = f2_->robot();
  const auto & mb2 = r2.mb();
  const auto & mbc2 = r2.mbc();
  const auto & NAB2 = r2.normalAccB();
  // FIXME This implementation is wrong but the more correct one is obviously not correct :D
  velocity_.setZero();
  normalAcceleration_.setZero();
  if(use_f1_)
  {
    velocity_ += dof_.asDiagonal() * f1_->velocity().vector();
    normalAcceleration_ += dof_.asDiagonal() * f1_->normalAcceleration().vector();
    splitJacobian(dof_.asDiagonal() * f1_->jacobian(), r1.q());
  }
  if(use_f2_)
  {
    velocity_ -= dof_.asDiagonal() * f2_->velocity().vector();
    normalAcceleration_ -= dof_.asDiagonal() * f2_->normalAcceleration().vector();
    splitJacobian(dof_.asDiagonal() * f2_->jacobian(), r2.q());
  }
  //// The error we computed previously in a MotionVecd
  // sva::MotionVecd err_f1(value_);
  //// Transformation from f1 to f2
  // auto X_f1_f2 = f2_->position() * f1_->position().inv();
  //// Rotation from f2 to f1
  // auto E_f2_f1 = sva::PTransformd(Eigen::Matrix3d(X_f1_f2.rotation().transpose()));
  //// f2 with f1 orientation in f2's body coordinates
  // auto X_r2b_f2_f1 = E_f2_f1 * f2_->X_b_f();
  //// Velocity of f1 expressed in f1
  // auto V_f1_f1 = f1_->rbdJacobian().velocity(mb1, mbc1, f1_->X_b_f());
  //// Angular velocity of f1 in f1
  // auto w_f1_f1 = sva::MotionVecd(V_f1_f1.angular(), Eigen::Vector3d::Zero());
  //// Velocity of f2 expressed in f1
  // auto V_f2_f1 = f2_->rbdJacobian().velocity(mb2, mbc2, X_r2b_f2_f1);
  //// Difference between the two velocities in the same frame
  // auto V_err = V_f2_f1 - V_f1_f1;
  //// Angular part of that difference
  // auto w_err = sva::MotionVecd(V_err.angular(), Eigen::Vector3d::Zero());
  //// Error derivative (i.e. the velocity difference in the moving frame f1)
  // auto V_err_f1 = err_f1.cross(w_f1_f1) + V_err;
  // velocity_ = V_err_f1.vector();
  //// Normal acceleration of f1 in f1
  // auto NA_f1_f1 = f1_->rbdJacobian().normalAcceleration(mb1, mbc1, NAB1, f1_->X_b_f(), sva::MotionVecd::Zero());
  //// Angular part of the acceleration of f1 in f1
  // auto wNA_f1_f1 = sva::MotionVecd(NA_f1_f1.angular(), Eigen::Vector3d::Zero());
  //// Normal acceleration of f2 in f1
  // auto NA_f2_f1 = f2_->rbdJacobian().normalAcceleration(mb2, mbc2, NAB2, X_r2b_f2_f1, w_err);
  //// Difference between the two normal accelerations
  // auto NA_err = NA_f2_f1 - NA_f1_f1;
  //// Error normal acceleration (i.e. the normal acceleration difference in the moving frame f1)
  // auto NA_err_f1 = V_err_f1.cross(w_f1_f1) + err_f1.cross(wNA_f1_f1) + NA_err;
  // normalAcceleration_ = NA_err_f1.vector();
  // if(use_f1_)
  //{
  //  jacTmp_.block(0, 0, 6, f1Jacobian_.dof()).noalias() = f1Jacobian_.jacobian(mb1, mbc1, f1_->position());
  //  for(int i = 0; i < f1Jacobian_.dof(); ++i)
  //  {
  //    jacTmp_.col(i).head<6>() -=
  //        err_f1.cross(sva::MotionVecd(jacTmp_.col(i).head<3>(), Eigen::Vector3d::Zero())).vector();
  //  }
  //  f1Jacobian_.fullJacobian(mb1, jacTmp_.block(0, 0, 6, f1Jacobian_.dof()), jac_);
  //  splitJacobian(jac_.block(0, 0, 6, mb1.nrDof()), r1.q());
  //}
  // if(use_f2_)
  //{
  //  jacTmp_.block(0, 0, 6, f2Jacobian_.dof()).noalias() = -f2Jacobian_.jacobian(mb2, mbc2, E_f2_f1 * f2_->position());
  //  f2Jacobian_.fullJacobian(mb2, jacTmp_.block(0, 0, 6, f2Jacobian_.dof()), jac_);
  //  splitJacobian(jac_.block(0, 0, 6, mb2.nrDof()), r2.q());
  //}
}

} // namespace mc_tvm
