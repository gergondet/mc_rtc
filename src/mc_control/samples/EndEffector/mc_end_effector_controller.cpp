/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_end_effector_controller.h"

#include <mc_rtc/logging.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

MCEndEffectorController::MCEndEffectorController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module,
                                                 double dt,
                                                 const mc_rtc::Configuration & config)
: MCController(robot_module, dt)
{
  solver().addConstraint(dynamicsConstraint_);
  solver().addConstraint(collisionConstraint_);
  solver().addConstraint(compoundJointConstraint_);
  solver().addTask(postureTask_);
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    solver().addContact({robot().name(), "ground", "LFullSole", "AllGround"});
    solver().addContact({robot().name(), "ground", "RFullSole", "AllGround"});
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    solver().addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    solver().addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  }
  else if(robot().mb().joint(0).dof() != 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("EndEffector sample does not support robot {}", robot().name());
  }

  std::string body = robot().mb().bodies().back().name();
  if(robot().hasBody("RARM_LINK7"))
  {
    body = "RARM_LINK7";
  }
  else if(robot().hasBody("r_wrist"))
  {
    body = "r_wrist";
  }
  efTask_ = std::make_shared<mc_tasks::TransformTask>(robot().frame(body), 5.0, 200.0);
  solver().addTask(efTask_);
  if(robot().mb().joint(0).dof() != 0)
  {
    comTask_ = std::make_shared<mc_tasks::CoMTask>(robot());
    solver().addTask(comTask_);
  }
  postureTask_->weight(1.0);
  if(config.has(robot().name()))
  {
    auto tasks = config(robot().name())("tasks", std::vector<mc_rtc::Configuration>{});
    for(auto & t : tasks)
    {
      tasks_.emplace_back(mc_tasks::MetaTaskLoader::load(solver(), t));
      solver().addTask(tasks_.back());
    }
  }
}

void MCEndEffectorController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  efTask_->reset();
  if(comTask_)
  {
    comTask_->reset();
  }
  for(auto & t : tasks_)
  {
    t->reset();
  }
}

} // namespace mc_control
