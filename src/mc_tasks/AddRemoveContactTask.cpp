/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/AddRemoveContactTask.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

AddRemoveContactTask::AddRemoveContactTask(mc_rbdyn::Frame & frame, double speed, double stiffness, double weight)
: constraint_(std::make_shared<mc_solver::BoundedSpeedConstr>()),
  task_(std::make_shared<mc_tasks::VelocityTask>(frame, stiffness, weight)), speed_(speed)
{
  type_ = "add_remove_contact";
  name_ = fmt::format("{}_{}", type_, frame.name());
  task_->name(fmt::format("{}_task", name_));
  constraint_->name(fmt::format("{}_constraint", name_));
  Eigen::Vector6d dimW = Eigen::Vector6d::Zero();
  dimW(5) = 1.0;
  task_->dimWeight(dimW);
  dimW(5) = speed_;
  task_->refVel(dimW);
}

void AddRemoveContactTask::addToSolver(mc_solver::QPSolver & solver)
{
  Eigen::Vector6d dof = Eigen::Vector6d::Ones();
  dof(5) = 0.0;
  constraint_->addBoundedSpeed(solver, task_->frame(), dof, Eigen::Vector6d::Zero());
  solver.addConstraint(constraint_);
  MetaTask::addToSolver(*task_, solver);
}

void AddRemoveContactTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  solver.removeConstraint(constraint_);
  MetaTask::removeFromSolver(*task_, solver);
}

void AddRemoveContactTask::speed(double s)
{
  speed_ = s;
  auto vel = task_->refVel();
  vel(5) = s;
  task_->refVel(vel);
}

void AddRemoveContactTask::addToGUI(mc_rtc::GUI & gui)
{
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::NumberInput("Speed", [this]() { return speed_; }, [this](double s) { speed(s); }));
}

} // namespace mc_tasks

namespace
{

template<bool Negative>
mc_tasks::AddRemoveContactTaskPtr load_add_remove_contact_task(mc_solver::QPSolver & solver,
                                                               const mc_rtc::Configuration & config)
{
  double speed = config("speed", 0.01);
  if constexpr(Negative)
  {
    speed = -speed;
  }
  auto & robot = solver.robots().fromConfig(config, "AddRemoveContactTask");
  return std::make_shared<mc_tasks::AddRemoveContactTask>(robot.frame(config("frame")), speed, config("stiffness", 2.0),
                                                          config("weight", 1000.0));
}

static auto ac_rc_registered =
    mc_tasks::MetaTaskLoader::register_load_function("addRemoveContact", &load_add_remove_contact_task<false>);
static auto ac_registered =
    mc_tasks::MetaTaskLoader::register_load_function("addContact", &load_add_remove_contact_task<true>);
static auto rc_registered =
    mc_tasks::MetaTaskLoader::register_load_function("removeContact", &load_add_remove_contact_task<false>);

} // namespace
