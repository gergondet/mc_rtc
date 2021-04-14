/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/DynamicFunction.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/Surface.h>

namespace mc_tvm
{

DynamicFunction::DynamicFunction(mc_rbdyn::RobotPtr robot)
: tvm::function::abstract::LinearFunction(robot->mb().nrDof()), robot_(robot)
{
  registerUpdates(Update::B, &DynamicFunction::updateb);
  registerUpdates(Update::Jacobian, &DynamicFunction::updateJacobian);
  addOutputDependency<DynamicFunction>(Output::B, Update::B);
  addOutputDependency<DynamicFunction>(Output::Jacobian, Update::Jacobian);
  addInputDependency<DynamicFunction>(Update::Jacobian, robot, mc_rbdyn::Robot::Output::H);
  addInputDependency<DynamicFunction>(Update::B, robot, mc_rbdyn::Robot::Output::C);
  addVariable(dot(robot->q(), 2), true);
  addVariable(robot->tau(), true);
  jacobian_[robot_->tau().get()] = -Eigen::MatrixXd::Identity(robot_->mb().nrDof(), robot_->mb().nrDof());
  jacobian_[robot_->tau().get()].properties(tvm::internal::MatrixProperties::MINUS_IDENTITY);
  velocity_.setZero();
}

DynamicFunction::ForceContact::ForceContact(mc_rbdyn::Frame & frame, std::vector<sva::PTransformd> points, double dir)
: frame_(frame), points_(std::move(points)), dir_(dir), force_jac_(6, frame.rbdJacobian().dof()),
  full_jac_(6, frame.robot().mb().nrDof())
{
  for(size_t i = 0; i < points_.size(); ++i)
  {
    forces_.add(tvm::Space(3).createVariable("force" + std::to_string(i)));
  }
  forces_.value(Eigen::VectorXd::Zero(3 * static_cast<Eigen::DenseIndex>(points_.size())));
}

void DynamicFunction::ForceContact::updateJacobians(DynamicFunction & parent)
{
  const auto & robot = frame_->robot();
  const auto & bodyJac = frame_->rbdJacobian().bodyJacobian(robot.mb(), robot.mbc());
  for(int i = 0; i < forces_.numberOfVariables(); ++i)
  {
    const auto & force = forces_[i];
    const auto & point = points_[static_cast<size_t>(i)];
    frame_->rbdJacobian().translateBodyJacobian(bodyJac, robot.mbc(), point.translation(), force_jac_);
    frame_->rbdJacobian().fullJacobian(robot.mb(), force_jac_, full_jac_);
    parent.jacobian_[force.get()].noalias() = -dir_ * full_jac_.block(3, 0, 3, robot.mb().nrDof()).transpose();
  }
}

sva::ForceVecd DynamicFunction::ForceContact::force() const
{
  sva::ForceVecd ret = sva::ForceVecd::Zero();
  for(int i = 0; i < forces_.numberOfVariables(); ++i)
  {
    const auto & force = forces_[i];
    const auto & point = points_[static_cast<size_t>(i)];
    ret += point.transMul(sva::ForceVecd(Eigen::Vector3d::Zero(), force->value()));
  }
  return ret;
}

const tvm::VariableVector & DynamicFunction::addContact(mc_rbdyn::Frame & frame,
                                                        std::vector<sva::PTransformd> points,
                                                        double dir)
{
  if(frame.robot().name() != robot_->name())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Attempted to add a contact for {} to dynamic function belonging to {}", frame.robot().name(), robot_->name());
  }
  auto & fc = contacts_.emplace_back(frame, std::move(points), dir);
  for(const auto & var : fc.forces_)
  {
    addVariable(var, true);
  }
  addInputDependency<DynamicFunction>(Update::Jacobian, frame, mc_rbdyn::Frame::Output::Jacobian);
  return fc.forces_;
}

void DynamicFunction::removeContact(const mc_rbdyn::Frame & frame)
{
  auto it = findContact(frame);
  if(it != contacts_.end())
  {
    // FIXME Remove variable from the function
    contacts_.erase(it);
  }
}

sva::ForceVecd DynamicFunction::contactForce(const mc_rbdyn::Frame & frame) const
{
  auto it = findContact(frame);
  if(it != contacts_.end())
  {
    return (*it).force();
  }
  else
  {
    mc_rtc::log::error("No contact at frame {} in dynamic function for {}", frame.name(), robot_->name());
    return sva::ForceVecd(Eigen::Vector6d::Zero());
  }
}

void DynamicFunction::updateb()
{
  b_ = robot_->C();
}

void DynamicFunction::updateJacobian()
{
  splitJacobian(robot_->H(), dot(robot_->q(), 2));
  for(auto & c : contacts_)
  {
    c.updateJacobians(*this);
  }
}

auto DynamicFunction::findContact(const mc_rbdyn::Frame & frame) const -> std::vector<ForceContact>::const_iterator
{
  return std::find_if(contacts_.begin(), contacts_.end(), [&](const auto & c) { return c.frame_.get() == &frame; });
}

} // namespace mc_tvm
