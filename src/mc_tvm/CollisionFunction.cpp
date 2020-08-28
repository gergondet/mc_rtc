/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/CollisionFunction.h>

#include <mc_tvm/utils.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/SCHAddon.h>

namespace mc_tvm
{

static mc_rbdyn::Convex & max(mc_rbdyn::Convex & c1, mc_rbdyn::Convex & c2)
{
  if(c1.frame().robot().mb().nrDof() < c2.frame().robot().mb().nrDof())
  {
    return c2;
  }
  else
  {
    return c1;
  }
}

static mc_rbdyn::Convex & min(mc_rbdyn::Convex & c1, mc_rbdyn::Convex & c2)
{
  if(c1.frame().robot().mb().nrDof() < c2.frame().robot().mb().nrDof())
  {
    return c1;
  }
  else
  {
    return c2;
  }
}

CollisionFunction::CollisionFunction(mc_rbdyn::Convex & c1, mc_rbdyn::Convex & c2, double dt)
: tvm::function::abstract::Function(1), c1_(max(c1, c2)), c2_(min(c1, c2)), dt_(dt),
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
    auto & r = convex->frame().robot();
    if(r.mb().nrDof() > 0)
    {
      addInputDependency<CollisionFunction>(Update::Value, *convex, mc_rbdyn::Convex::Output::Position);
      addInputDependency<CollisionFunction>(Update::Jacobian, r, mc_rbdyn::Robot::Output::FV);
      addInputDependency<CollisionFunction>(Update::NormalAcceleration, r, mc_rbdyn::Robot::Output::NormalAcceleration);
      addVariable(r.q(), false);
      data_.push_back({Eigen::Vector3d::Zero(), convex->frame().rbdJacobian()});
    }
  };
  addConvex(c1_);
  addConvex(c2_);

  // By construction c1's robot has more dofs than c2's
  auto maxDof = c1_->frame().robot().mb().nrDof();
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
    const auto & r = object.get()->frame().robot();
    const auto & jac = d.jac_.jacobian(r.mb(), r.mbc());
    distJac_.block(0, 0, 1, d.jac_.dof()).noalias() =
        (sign * normVecDist_).transpose() * jac.block(3, 0, 3, d.jac_.dof());
    d.jac_.fullJacobian(r.mb(), distJac_.block(0, 0, 1, d.jac_.dof()), fullJac_);
    int ffSize = r.qFloatingBase()->space().tSize();
    int jSize = r.qJoints().totalSize();
    if(ffSize)
    {
      jacobian_[r.qFloatingBase().get()].block(0, 0, 1, ffSize) += fullJac_.block(0, 0, 1, ffSize);
    }
    if(jSize)
    {
      mc_tvm::splitAddJacobian(fullJac_.block(0, ffSize, 1, jSize), r.qJoints(), jacobian_);
    }
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
    const auto & r = object.get()->frame().robot();
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
    const auto & r = object.get()->frame().robot();
    Eigen::Vector3d pNormalAcc = d.jac_.normalAcceleration(r.mb(), r.mbc(), r.normalAccB()).linear();
    Eigen::Vector3d pSpeed = d.jac_.velocity(r.mb(), r.mbc()).linear();
    normalAcceleration_(0) += sign * (pNormalAcc.dot(normVecDist_) + pSpeed.dot(speedVec_));
    sign = -1.0;
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
}

} // namespace mc_tvm
