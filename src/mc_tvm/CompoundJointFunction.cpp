/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/CompoundJointFunction.h>

#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

CompoundJointFunction::CompoundJointFunction(mc_rbdyn::RobotPtr robot,
                                             const mc_rbdyn::CompoundJointConstraintDescription & desc,
                                             double dt)
: tvm::function::abstract::LinearFunction(1), robot_(robot), dt_(dt)
{
  registerUpdates(Update::B, &CompoundJointFunction::updateB);
  addOutputDependency<CompoundJointFunction>(Output::B, Update::B);
  auto checkJoint = [&](const std::string & jName) {
    if(!robot_->hasJoint(jName))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("No joint named {} in {}", jName, robot->name());
    }
    auto qIdx = robot_->jointIndexByName(jName);
    if(robot_->mb().joint(static_cast<int>(qIdx)).dof() != 1)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Joint {} does not have exactly one dof", jName);
    }
    return qIdx;
  };
  auto q1Idx = checkJoint(desc.j1);
  auto q2Idx = checkJoint(desc.j2);
  if(q1Idx == q2Idx)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot add a compound joint constraint of a joint with itself");
  }
  desc_ = {q1Idx, q2Idx, desc.p1.x(), desc.p1.y(), desc.p2.x() - desc.p1.x(), desc.p2.y() - desc.p1.y()};
  b_cst_ = desc_.p1_y * desc_.P_x - desc_.p1_x * desc_.P_y;
  const auto & q1Var = robot_->qJoint(q1Idx);
  const auto & q2Var = robot_->qJoint(q2Idx);
  addVariable(q1Var, true);
  auto & jac1 = jacobian_.at(q1Var.get());
  jac1.properties({tvm::internal::MatrixProperties::Constness(true)});
  jac1(0, 0) = dt_ * dt_ * desc_.P_y / 2;
  addVariable(q2Var, true);
  auto & jac2 = jacobian_.at(q2Var.get());
  jac2.properties({tvm::internal::MatrixProperties::Constness(true)});
  jac2(0, 0) = -dt_ * dt_ * desc_.P_x / 2;
}

void CompoundJointFunction::updateB()
{
  const auto & q1 = robot_->mbc().q[desc_.q1Idx][0];
  const auto & alpha1 = robot_->mbc().alpha[desc_.q1Idx][0];
  const auto & q2 = robot_->mbc().q[desc_.q2Idx][0];
  const auto & alpha2 = robot_->mbc().alpha[desc_.q2Idx][0];
  b_(0) = b_cst_ + desc_.P_y * q1 - desc_.P_x * q2 + dt_ * (desc_.P_y * alpha1 - desc_.P_x * alpha2);
}

} // namespace mc_tvm
