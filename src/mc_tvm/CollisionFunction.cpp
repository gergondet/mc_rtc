/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/CollisionFunction.h>

#include <mc_tvm/utils.h>

#include <mc_rbdyn/SCHAddon.h>

namespace mc_tvm
{

static mc_rbdyn::Convex & max(mc_rbdyn::Convex & c1, mc_rbdyn::Convex & c2)
{
  auto f1_ptr = dynamic_cast<mc_rbdyn::RobotFrame *>(&c1.frame());
  auto f2_ptr = dynamic_cast<mc_rbdyn::RobotFrame *>(&c2.frame());
  if(f1_ptr && !f2_ptr)
  {
    return c1;
  }
  if(f2_ptr && !f1_ptr)
  {
    return c2;
  }
  if(!f2_ptr && !f1_ptr)
  {
    return c1;
  }
  if(f1_ptr->robot().mb().nrDof() < f2_ptr->robot().mb().nrDof())
  {
    return c2;
  }
  else
  {
    return c1;
  }
}

CollisionFunction::CollisionFunction(mc_rbdyn::Convex & c1, mc_rbdyn::Convex & c2, double dt)
: tvm::function::abstract::Function(1), c1_(max(c1, c2)), c2_(c1_.get() == &c1 ? c2 : c1), dt_(dt),
  pair_(c1_->convex().get(), c2_->convex().get())
{
  // clang-format off
  registerUpdates(Update::Value, &CollisionFunction::updateValue,
                  Update::Velocity, &CollisionFunction::updateVelocity,
                  Update::Jacobian, &CollisionFunction::updateJacobian,
                  Update::NormalAcceleration, &CollisionFunction::updateNormalAcceleration);
  // clang-format on

  addOutputDependency<CollisionFunction>(Output::Value, Update::Value);
  addOutputDependency<CollisionFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<CollisionFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<CollisionFunction>(Output::NormalAcceleration, Update::NormalAcceleration);

  addInternalDependency<CollisionFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<CollisionFunction>(Update::Velocity, Update::Jacobian);
  addInternalDependency<CollisionFunction>(Update::NormalAcceleration, Update::Value);

  auto addConvex = [this](mc_rbdyn::ConvexPtr & convex) {
    auto f_ptr = std::dynamic_pointer_cast<mc_rbdyn::RobotFrame>(convex->frame().shared_from_this());
    if(!f_ptr)
    {
      return 0;
    }
    auto & r = f_ptr->robot();
    if(r.mb().nrDof() > 0)
    {
      addInputDependency<CollisionFunction>(Update::Value, *convex, mc_rbdyn::Convex::Output::Position);
      addInputDependency<CollisionFunction>(Update::Jacobian, r, mc_rbdyn::Robot::Output::FV);
      addInputDependency<CollisionFunction>(Update::NormalAcceleration, r, mc_rbdyn::Robot::Output::NormalAcceleration);
      addVariable(r.q(), false);
      data_.push_back({f_ptr, Eigen::Vector3d::Zero(), f_ptr->rbdJacobian()});
    }
    return r.mb().nrDof();
  };
  // By construction c1's robot has more dofs than c2's
  auto maxDof = addConvex(c1_);
  addConvex(c2_);

  fullJac_.resize(1, maxDof);
  distJac_.resize(1, maxDof);
}

void CollisionFunction::updateValue()
{
  double dist = sch::mc_rbdyn::distance(pair_, p1_, p2_);
  if(dist == 0)
  {
    dist = sch::epsilon;
  }
  dist = dist >= 0 ? std::sqrt(dist) : -std::sqrt(-dist);
  normVecDist_ = (p1_ - p2_) / dist;
  if(iter_ == 1)
  {
    prevNormVecDist_ = normVecDist_;
  }
  if(prevIter_ != iter_)
  {
    speedVec_ = (normVecDist_ - prevNormVecDist_) / dt_;
    prevNormVecDist_ = normVecDist_;
    prevIter_ = iter_;
  }
  auto object = std::ref(c1_);
  auto point = std::ref(p1_);
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & d = data_[i];
    d.nearestPoint_ = (sva::PTransformd(point) * object.get()->frame().position().inv()).translation();
    d.jac_.point(d.nearestPoint_);
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
  value_(0) = dist;
}

void CollisionFunction::tick()
{
  iter_++;
}

void CollisionFunction::updateJacobian()
{
  for(int i = 0; i < variables_.numberOfVariables(); ++i)
  {
    jacobian_[variables_[i].get()].setZero();
  }
  double sign = 1.0;
  auto object = std::ref(c1_);
  auto point = std::ref(p1_);
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & d = data_[i];
    const auto & r = d.frame_->robot();
    const auto & jac = d.jac_.jacobian(r.mb(), r.mbc());
    distJac_.block(0, 0, 1, d.jac_.dof()).noalias() =
        (sign * normVecDist_).transpose() * jac.block(3, 0, 3, d.jac_.dof());
    d.jac_.fullJacobian(r.mb(), distJac_.block(0, 0, 1, d.jac_.dof()), fullJac_);
    jacobian_[r.q().get()] += fullJac_;
    sign = -1.0;
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
}

void CollisionFunction::updateVelocity()
{
  velocity_(0) = 0;
  double sign = 1.0;
  auto object = std::ref(c1_);
  auto point = std::ref(p1_);
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & d = data_[i];
    const auto & r = d.frame_->robot();
    velocity_(0) += sign * d.jac_.velocity(r.mb(), r.mbc()).linear().dot(normVecDist_);
    sign = -1.0;
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
}

void CollisionFunction::updateNormalAcceleration()
{
  normalAcceleration_(0) = 0;
  double sign = 1.0;
  auto object = std::ref(c1_);
  auto point = std::ref(p1_);
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & d = data_[i];
    const auto & r = d.frame_->robot();
    Eigen::Vector3d pNormalAcc = d.jac_.normalAcceleration(r.mb(), r.mbc(), r.normalAccB()).linear();
    Eigen::Vector3d pSpeed = d.jac_.velocity(r.mb(), r.mbc()).linear();
    normalAcceleration_(0) += sign * (pNormalAcc.dot(normVecDist_) + pSpeed.dot(speedVec_));
    sign = -1.0;
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
}

} // namespace mc_tvm
