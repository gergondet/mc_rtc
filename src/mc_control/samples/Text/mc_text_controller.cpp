/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_text_controller.h"

#include <mc_solver/ConstraintLoader.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/deprecated.h>
#include <mc_rtc/logging.h>

namespace mc_control
{

MCTextController::MCTextController(std::shared_ptr<mc_rbdyn::RobotModule> robot,
                                   double dt,
                                   const mc_rtc::Configuration & config)
: MCController(robot, dt)
{
  if(!config.has("Text"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No entries relative to the Text controller in the loaded configuration");
  }
  config_ = config("Text");
}

void MCTextController::reset(const mc_control::ControllerResetData & data)
{
  MCController::reset(data);
  if(!config_.has("constraints"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No constraints in the provided text file");
  }
  if(!config_.has("tasks"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No tasks in the provided text file");
  }
  for(const auto & c : config_("constraints"))
  {
    constraints_.push_back(mc_solver::ConstraintLoader::load(solver(), c));
    solver().addConstraint(constraints_.back());
  }
  solver().addTask(postureTask_);
  for(const auto & t : config_("tasks"))
  {
    tasks_.push_back(mc_tasks::MetaTaskLoader::load(solver(), t));
    solver().addTask(tasks_.back());
  }
  auto contacts = config_("contacts", std::vector<mc_rtc::Configuration>{});
  for(auto & c : contacts)
  {
    if(!c.has("r1"))
    {
      mc_rtc::log::missing("Text", "r1", "In \"contacts\" configuration");
      c.add("r1", robots().robot().name());
    }
    if(!c.has("r2"))
    {
      mc_rtc::log::missing("Text", "r2", "In \"contacts\" configuration");
      c.add("r2", robots().robots()[1]->name());
    }
    solver().addContact(c);
  }
}

} // namespace mc_control
