/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/configuration_io.h>

#include <mc_solver/ConstraintLoader.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Form.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/io_utils.h>

namespace mc_control
{

namespace fsm
{

Controller::Controller(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt, const mc_rtc::Configuration & config)
: MCController(std::vector<mc_rbdyn::RobotModulePtr>{rm}, dt, config),
#ifndef MC_RTC_BUILD_STATIC
  factory_(config("StatesLibraries", std::vector<std::string>{}),
           config("StatesFiles", std::vector<std::string>{}),
           config("VerboseStateFactory", false))
#else
  factory_(factory())
#endif
{
#ifdef MC_RTC_BUILD_STATIC
  factory_.load_files(config("StatesFiles", std::vector<std::string>{}));
  factory_.set_verbosity(config("VerboseStateFactory", false));
#endif
  idle_keep_state_ = config("IdleKeepState", false);
  /** Load additional robots from the configuration */
  {
    auto config_robots = config("robots", std::map<std::string, mc_rtc::Configuration>{});
    for(const auto & cr : config_robots)
    {
      const auto & name = cr.first;
      if(hasRobot(name))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("FSM controller cannot have two robots with the same name");
      }
      std::string module = cr.second("module");
      auto params = cr.second("params", std::vector<std::string>{});
      mc_rbdyn::RobotModulePtr rm = nullptr;
      if(params.size() == 0)
      {
        rm = mc_rbdyn::RobotLoader::get_robot_module(module);
      }
      else if(params.size() == 1)
      {
        rm = mc_rbdyn::RobotLoader::get_robot_module(module, params.at(0));
      }
      else if(params.size() == 2)
      {
        rm = mc_rbdyn::RobotLoader::get_robot_module(module, params.at(0), params.at(1));
      }
      else
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "FSM controller only handles robot modules that require two parameters at most");
      }
      if(!rm)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Failed to load {} as specified in configuration", name);
      }
      loadRobot(rm, name, cr.second("init_pos", sva::PTransformd::Identity()));
    }
    mc_rtc::log::info("Robots loaded in FSM controller:");
    for(const auto & r : robots())
    {
      mc_rtc::log::info("- {}", r->name());
    }
  }
  /** Load global constraints (robots' kinematics/dynamics constraints and contact constraint */
  {
    auto config_constraints = config("constraints", std::vector<mc_rtc::Configuration>{});
    for(const auto & cc : config_constraints)
    {
      constraints_.emplace_back(mc_solver::ConstraintLoader::load(solver(), cc));
      solver().addConstraint(*constraints_.back());
    }
  }
  /** Load collision managers */
  {
    auto config_collisions = config("collisions", std::vector<mc_rtc::Configuration>{});
    for(auto & config_cc : config_collisions)
    {
      auto & r1 = robots().fromConfig(config_cc, "collision", false, "r1Index", "r1");
      auto & r2 = robots().fromConfig(config_cc, "collision", false, "r2Index", "r2");
      if(r1.name() == r2.name())
      {
        if(config_cc("useCommon", false))
        {
          addCollisions(r1.name(), r1.name(), r1.module().commonSelfCollisions());
        }
        if(config_cc("useMinimal", false))
        {
          addCollisions(r1.name(), r1.name(), r1.module().minimalSelfCollisions());
        }
      }
      addCollisions(r1.name(), r2.name(), config_cc("collisions", std::vector<mc_rbdyn::CollisionDescription>{}));
    }
  }
  /** Create posture task for actuated robots */
  for(auto & robot : robots())
  {
    if(robot->mb().nrDof() - robot->mb().joint(0).dof() > 0)
    {
      double stiffness = 1.0;
      double weight = 10.0;
      if(config.has(robot->name()))
      {
        auto robot_config = config(robot->name());
        if(robot_config.has("posture"))
        {
          robot_config("posture")("stiffness", stiffness);
          robot_config("posture")("weight", weight);
        }
      }
      auto t = std::make_shared<mc_tasks::PostureTask>(*robot, stiffness, weight);
      t->name("FSM_" + t->name());
      posture_tasks_[robot->name()] = t;
      solver().addTask(t);
    }
    if(robot->mb().joint(0).type() == rbd::Joint::Free)
    {
      double stiffness = 2.0;
      double weight = 100.0;
      if(config.has(robot->name()))
      {
        auto robot_config = config(robot->name());
        if(robot_config.has("ff"))
        {
          robot_config("ff")("stiffness", stiffness);
          robot_config("ff")("weight", weight);
        }
      }
      auto t = std::make_shared<mc_tasks::TransformTask>(robot->frame(robot->mb().body(0).name()), stiffness, weight);
      t->name("FSM_" + t->name());
      ff_tasks_[robot->name()] = t;
    }
  }
  /** Create contacts */
  auto contacts = config("contacts", std::vector<mc_rbdyn::Contact>{});
  for(const auto & c : contacts)
  {
    addContact(c);
  }
  /** Load more states if they are provided in the configuration */
  if(config.has("states"))
  {
    factory_.load(config("states"));
  }
  /** Setup executor */
  executor_.init(*this, config_);
}

Controller::~Controller()
{
  executor_.teardown(*this);
}

bool Controller::run()
{
  return run(mc_solver::FeedbackType::None);
}

bool Controller::run(mc_solver::FeedbackType fType)
{
  executor_.run(*this, idle_keep_state_);
  if(!executor_.running())
  {
    if(running_)
    {
      running_ = false;
      startIdleState();
    }
  }
  else
  {
    if(!running_)
    {
      running_ = true;
      teardownIdleState();
    }
  }
  return MCController::run(fType);
}

void Controller::reset(const ControllerResetData & data)
{
  MCController::reset(data);
  if(config().has("init_pos"))
  {
    robot().posW(config()("init_pos"));
    realRobot().posW(robot().posW());
  }
  /** GUI information */
  auto all_states = factory_.states();
  std::sort(all_states.begin(), all_states.end());
  gui_->data().add("states", all_states);
  startIdleState();
}

void Controller::resetPostures()
{
  for(auto & pt : posture_tasks_)
  {
    pt.second->reset();
  }
}

void Controller::startIdleState()
{
  resetPostures();
  // Save posture weights
  saved_posture_weights_.clear();
  for(auto & pt : posture_tasks_)
  {
    saved_posture_weights_[pt.first] = pt.second->weight();
    // Set high weight to prevent the robot from changing configuration
    pt.second->weight(10000);
  }
  for(auto & fft : ff_tasks_)
  {
    fft.second->reset();
    solver().addTask(fft.second);
  }
}

void Controller::teardownIdleState()
{
  // Reset default posture weights
  for(auto & pt : posture_tasks_)
  {
    pt.second->weight(saved_posture_weights_[pt.first]);
  }

  for(auto & fft : ff_tasks_)
  {
    solver().removeTask(fft.second);
  }
}

mc_tasks::PostureTaskPtr Controller::getPostureTask(const std::string & robot)
{
  if(posture_tasks_.count(robot))
  {
    return posture_tasks_.at(robot);
  }
  return nullptr;
}

bool Controller::resume(const std::string & state)
{
  if(!factory_.hasState(state))
  {
    mc_rtc::log::error("Cannot play unloaded state: {}", state);
    return false;
  }
  return executor_.resume(state);
}

} // namespace fsm

} // namespace mc_control
