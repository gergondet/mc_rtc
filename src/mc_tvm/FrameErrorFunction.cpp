/* Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL  */

#include <mc_tvm/FrameErrorFunction.h>

#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

FrameErrorFunction::FrameErrorFunction(mc_rbdyn::FramePtr f1, mc_rbdyn::FramePtr f2, const Eigen::Vector6d & dof)
: tvm::function::abstract::Function(6), dof_(dof)
{
  // clang-format off
    registerUpdates(Update::Value, &FrameErrorFunction::updateValue,
                    Update::Jacobian, &FrameErrorFunction::updateJacobian,
                    Update::Velocity, &FrameErrorFunction::updateVelocity,
                    Update::NormalAcceleration, &FrameErrorFunction::updateNormalAcceleration);
  // clang-format on

  addOutputDependency<FrameErrorFunction>(Output::Value, Update::Value);
  addOutputDependency<FrameErrorFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<FrameErrorFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<FrameErrorFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  addInternalDependency<FrameErrorFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<FrameErrorFunction>(Update::Velocity, Update::Value);
  addInternalDependency<FrameErrorFunction>(Update::NormalAcceleration, Update::Velocity);

  auto addRobot = [this](mc_rbdyn::FramePtr & fIn, mc_rbdyn::FramePtr & fOut, bool & useF, rbd::Jacobian & jac) {
    fOut = fIn;
    if(fIn->rbdJacobian().dof() > 0)
    {
      const auto & r = fIn->robot();
      useF = true;
      jac = fIn->rbdJacobian();
      addInputDependency<FrameErrorFunction>(Update::Value, fOut, mc_rbdyn::Frame::Output::Position);
      addInputDependency<FrameErrorFunction>(Update::Jacobian, fOut, mc_rbdyn::Frame::Output::Jacobian);
      addInputDependency<FrameErrorFunction>(Update::Velocity, fOut, mc_rbdyn::Frame::Output::Velocity);
      addInputDependency<FrameErrorFunction>(Update::NormalAcceleration, fOut,
                                             mc_rbdyn::Frame::Output::NormalAcceleration);
      addVariable(r.q(), false);
    }
    else
    {
      useF = false;
      addInputDependency<FrameErrorFunction>(Update::Value, fOut, mc_rbdyn::Frame::Output::Position);
    }
  };
  addRobot(f1, f1_, use_f1_, f1Jacobian_);
  addRobot(f2, f2_, use_f2_, f2Jacobian_);
  assert(((*f1->robot().q() == *f2->robot().q()) || !f1->robot().q()->intersects(*f2->robot().q()))
         && "Current implementation does not handle robots sharing sub-parts of their variables. updateJacobian should "
            "be changed if you want to handle that.");
  sameVariable_ = use_f1_ && use_f2_ && (*f1->robot().q() == *f2->robot().q());
}

void FrameErrorFunction::updateValue()
{
  value_.head<3>() = sva::rotationError(f1_->position().rotation(), f2_->position().rotation());
  value_.tail<3>() = f2_->position().translation() - f1_->position().translation();
  dLog1_ = sva::SO3RightJacInv(Eigen::Vector3d(value_.head<3>()));
}

void FrameErrorFunction::updateJacobian()
{
  if(use_f1_)
  {
    auto & J1 = jacobian_[f1_->robot().q().get()];
    J1.topRows<3>().noalias() = -dLog1_ * f1_->jacobian().topRows<3>();
    J1.bottomRows<3>() = -f1_->jacobian().bottomRows<3>();
  }
  if(use_f2_)
  {
    auto & J2 = jacobian_[f2_->robot().q().get()];
    if(sameVariable_)
    {
      J2.topRows<3>().noalias() += dLog1_.transpose() * f2_->jacobian().topRows<3>();
      J2.bottomRows<3>() += f2_->jacobian().bottomRows<3>();
    }
    else
    {
      J2.topRows<3>().noalias() = dLog1_.transpose() * f2_->jacobian().topRows<3>();
      J2.bottomRows<3>() = f2_->jacobian().bottomRows<3>();
    }
  }
}

void FrameErrorFunction::updateVelocity()
{
  if(use_f1_)
  {
    velocity_.head<3>() = -dLog1_ * f1_->velocity().angular();
    velocity_.tail<3>() = -f1_->velocity().linear();
  }
  else
  {
    velocity_.setZero();
  }
  if(use_f2_)
  {
    velocity_.head<3>() += dLog1_.transpose() * f2_->velocity().angular();
    velocity_.tail<3>() += f2_->velocity().linear();
  }
}

void FrameErrorFunction::updateNormalAcceleration()
{
  const auto & d2Log =
      sva::SO3RightJacInvDot(Eigen::Vector3d(value_.head<3>()), Eigen::Vector3d((velocity_.head<3>())));
  if(use_f1_)
  {
    normalAcceleration_.head<3>() = -d2Log * f1_->velocity().angular() - dLog1_ * f1_->normalAcceleration().angular();
    normalAcceleration_.tail<3>() = -f1_->normalAcceleration().linear();
  }
  else
  {
    normalAcceleration_.setZero();
  }
  if(use_f2_)
  {
    normalAcceleration_.head<3>() +=
        d2Log.transpose() * f2_->velocity().angular() + dLog1_.transpose() * f2_->normalAcceleration().angular();
    normalAcceleration_.tail<3>() += f2_->normalAcceleration().linear();
  }
}

} // namespace mc_tvm
