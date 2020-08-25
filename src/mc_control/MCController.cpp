/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/MCController.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/constants.h>

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/config.h>
#include <mc_rtc/gui/Schema.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <array>
#include <fstream>

namespace mc_control
{

MCController::MCController(mc_rbdyn::RobotModulePtr robot, double dt) : MCController(robot, dt, {}) {}

MCController::MCController(mc_rbdyn::RobotModulePtr robot, double dt, const mc_rtc::Configuration & config)
: MCController({robot, mc_rbdyn::RobotLoader::get_robot_module("env",
                                                               std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                               std::string("ground"))},
               dt,
               config)
{
}

MCController::MCController(const std::vector<mc_rbdyn::RobotModulePtr> & robots_modules, double dt)
: MCController(robots_modules, dt, {})
{
}

namespace internal
{

static mc_rbdyn::Robot & loadRobot(mc_rbdyn::Robots & robots,
                                   mc_rtc::GUI & gui,
                                   const mc_rbdyn::RobotModulePtr & rm,
                                   std::string_view name)
{
  auto & r = robots.load(*rm, name);
  r.mbc().gravity = mc_rtc::constants::gravity;
  r.updateKinematics();
  auto data = gui.data();
  if(!data.has("robots"))
  {
    data.array("robots");
  }
  if(!data.has("bodies"))
  {
    data.add("bodies");
  }
  if(!data.has("surfaces"))
  {
    data.add("surfaces");
  }
  data("robots").push(r.name());
  auto bs = data("bodies").array(r.name());
  for(const auto & b : r.mb().bodies())
  {
    bs.push(b.name());
  }
  data("surfaces").add(r.name(), r.availableSurfaces());
  std::string rName = name;
  gui.addElement({"Robots"}, mc_rtc::gui::Robot(r.name(), [rName, &robots]() -> const mc_rbdyn::Robot & {
                   return robots.robot(rName);
                 }));
  return r;
}

static mc_rbdyn::RobotsPtr loadRobots(const std::vector<mc_rbdyn::RobotModulePtr> & modules,
                                      mc_rtc::GUI & gui,
                                      mc_rbdyn::Robots & realRobots)
{
  auto robots = std::make_shared<mc_rbdyn::Robots>();
  mc_rtc::map<std::string_view, size_t> count;
  for(auto & m : modules)
  {
    auto it = count.find(m->name);
    if(it == count.end())
    {
      count[m->name] = 1;
      const auto & r = loadRobot(*robots, gui, m, m->name);
      realRobots.robotCopy(r, r.name());
    }
    else
    {
      count[m->name] += 1;
      const auto & r = loadRobot(*robots, gui, m, fmt::format("{}_{}", m->name, count[m->name]));
      realRobots.robotCopy(r, r.name());
    }
  }
  return robots;
}

} // namespace internal

MCController::MCController(const std::vector<mc_rbdyn::RobotModulePtr> & modules,
                           double dt,
                           const mc_rtc::Configuration & config)
: logger_(std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED, "", "")),
  gui_(std::make_shared<mc_rtc::gui::StateBuilder>()), realRobots_(std::make_shared<mc_rbdyn::Robots>()),
  robots_(internal::loadRobots(modules, *gui_, *realRobots_)), solver_(robots_, realRobots_, logger_, gui_, dt),
  config_(config)
{
  gui_->addElement({"Global", "Add task"},
                   mc_rtc::gui::Schema("Add MetaTask", "MetaTask", [this](const mc_rtc::Configuration & config) {
                     try
                     {
                       auto t = mc_tasks::MetaTaskLoader::load(this->solver(), config);
                       this->solver().addTask(t);
                     }
                     catch(...)
                     {
                       mc_rtc::log::error("Failed to load MetaTask from request\n{}", config.dump(true));
                     }
                   }));
  /* Initialize constraints and tasks */
  std::array<double, 3> damper = {0.1, 0.01, 0.5};
  kinematicsConstraint_ = std::make_shared<mc_solver::KinematicsConstraint>(robot(), damper, 0.5);
  dynamicsConstraint_ = std::make_shared<mc_solver::DynamicsConstraint>(robot(), damper, 0.5);
  collisionConstraint_ = std::make_shared<mc_solver::CollisionsConstraint>();
  collisionConstraint_->addCollisions(solver(), {robot().name(), modules[0]->minimalSelfCollisions()});
  compoundJointConstraint_ = std::make_shared<mc_solver::CompoundJointConstraint>(robot(), dt);
  postureTask_ = std::make_shared<mc_tasks::PostureTask>(robot());
  mc_rtc::log::info("MCController(base) ready");
}

MCController::~MCController() {}

mc_rbdyn::Robot & MCController::loadRobot(mc_rbdyn::RobotModulePtr rm,
                                          std::string_view name,
                                          const sva::PTransformd & posW)
{
  assert(rm);
  auto & r = internal::loadRobot(*robots_, *gui_, rm, name);
  r.posW(posW);
  realRobots_->robotCopy(r, r.name());
  return r;
}

void MCController::removeRobot(std::string_view name)
{
  robots().removeRobot(name);
  realRobots().removeRobot(name);
}

void MCController::createObserverPipelines(const mc_rtc::Configuration & config)
{
  if(config.has("EnabledObservers") || config.has("RunObservers") || config.has("UpdateObservers"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}] The observer pipeline can no longer be configured by \"EnabledObservers\", \"RunObservers\" and "
        "\"UpdateObservers\".\nMultiple "
        "pipelines are now supported, allowing for estimation of multiple robots and/or multiple observations of the "
        "same robot.\nFor details on upgrading, please refer to:\n"
        "- The observer pipelines tutorial: https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html\n"
        "- The JSON Schema documentation: https://jrl-umi3218.github.io/mc_rtc/json.html#Observers/ObserverPipelines",
        name_);
  }
  if(!config.has("ObserverPipelines"))
  {
    mc_rtc::log::warning("[MCController::{}] No state observation pipeline configured: the state of the real robots "
                         "will not be estimated",
                         name_);
    return;
  }
  auto pipelineConfigs = mc_rtc::fromVectorOrElement(config, "ObserverPipelines", std::vector<mc_rtc::Configuration>{});
  for(const auto & pipelineConfig : pipelineConfigs)
  {
    observerPipelines_.emplace_back(*this);
    auto & pipeline = observerPipelines_.back();
    pipeline.create(pipelineConfig, timeStep);
    if(pipelineConfig("log", true))
    {
      pipeline.addToLogger(logger());
    }
    if(pipelineConfig("gui", false))
    {
      pipeline.addToGUI(*gui());
    }
  }
}

bool MCController::resetObserverPipelines()
{
  std::string desc;
  for(auto & pipeline : observerPipelines_)
  {
    pipeline.reset();
    if(desc.size())
    {
      desc += "\n";
    }
    desc += "- " + pipeline.desc();
  }
  if(desc.size())
  {
    mc_rtc::log::success("[MCController::{}] State observation pipelines:\n{}", name_, desc);
  }
  return true;
}

bool MCController::runObserverPipelines()
{
  bool success = true;
  for(auto & pipeline : observerPipelines_)
  {
    success = pipeline.run() && success;
  }
  return success;
}

bool MCController::hasObserverPipeline(const std::string & name) const
{
  return std::find_if(observerPipelines_.begin(), observerPipelines_.end(),
                      [&name](const mc_observers::ObserverPipeline & pipeline) { return pipeline.name() == name; })
         != observerPipelines_.end();
}

const std::vector<mc_observers::ObserverPipeline> & MCController::observerPipelines() const
{
  return observerPipelines_;
}

std::vector<mc_observers::ObserverPipeline> & MCController::observerPipelines()
{
  return observerPipelines_;
}

const mc_observers::ObserverPipeline & MCController::observerPipeline(std::string_view name) const
{
  auto pipelineIt =
      std::find_if(observerPipelines_.begin(), observerPipelines_.end(),
                   [&name](const mc_observers::ObserverPipeline & pipeline) { return pipeline.name() == name; });
  if(pipelineIt != observerPipelines_.end())
  {
    return *pipelineIt;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Observer pipeline {} does not exist", name);
  }
}

mc_observers::ObserverPipeline & MCController::observerPipeline(std::string_view name)
{
  return const_cast<mc_observers::ObserverPipeline &>(static_cast<const MCController *>(this)->observerPipeline(name));
}

bool MCController::hasObserverPipeline() const
{
  return !observerPipelines_.empty();
}

const mc_observers::ObserverPipeline & MCController::observerPipeline() const
{
  if(!hasObserverPipeline())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Controller {} does not have a default observer pipeline", name_);
  }
  return observerPipeline(observerPipelines_.front().name());
}

mc_observers::ObserverPipeline & MCController::observerPipeline()
{
  return const_cast<mc_observers::ObserverPipeline &>(static_cast<const MCController *>(this)->observerPipeline());
}

bool MCController::run()
{
  return run(mc_solver::FeedbackType::None);
}

bool MCController::run(mc_solver::FeedbackType fType)
{
  if(!solver_.run(fType))
  {
    mc_rtc::log::error("QP failed to run()");
    return false;
  }
  return true;
}

void MCController::reset(const ControllerResetData & reset_data)
{
  std::vector<std::string> supported;
  supported_robots(supported);
  if(supported.size() && std::find(supported.cbegin(), supported.cend(), robot().name()) == supported.end())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MCController] The main robot {} is not supported by this controller. Supported robots are: [{}]",
        robot().name(), mc_rtc::io::to_string(supported));
  }

  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  postureTask_->posture(reset_data.q);
  robot().updateKinematics();
  gui_->removeCategory({"Contacts"});
  gui_->addElement({"Contacts"}, mc_rtc::gui::Label("Contacts", [this]() {
                     std::string ret;
                     for(const auto & c : contacts())
                     {
                       ret += fmt::format("{}::{}/{}::{} | {} | {}\n", c.r1, c.r1Surface, c.r2, c.r2Surface,
                                          c.dof.transpose(), c.friction);
                     }
                     if(ret.size())
                     {
                       ret.pop_back();
                     }
                     return ret;
                   }));
  gui_->addElement(
      {"Contacts", "Add"},
      mc_rtc::gui::Form("Add contact",
                        [this](const mc_rtc::Configuration & data) {
                          std::string r1 = data("R1");
                          std::string r2 = data("R2");
                          std::string r1Surface = data("R1 surface");
                          std::string r2Surface = data("R2 surface");
                          double friction = data("Friction", mc_rbdyn::Contact::defaultFriction);
                          Eigen::Vector6d dof = data("dof", Eigen::Vector6d::Ones().eval());
                          addContact({r1, r2, r1Surface, r2Surface, friction, dof});
                        },
                        mc_rtc::gui::FormDataComboInput{"R1", true, {"robots"}},
                        mc_rtc::gui::FormDataComboInput{"R1 surface", true, {"surfaces", "$R1"}},
                        mc_rtc::gui::FormDataComboInput{"R2", true, {"robots"}},
                        mc_rtc::gui::FormDataComboInput{"R2 surface", true, {"surfaces", "$R2"}},
                        mc_rtc::gui::FormNumberInput{"Friction", false, mc_rbdyn::Contact::defaultFriction},
                        mc_rtc::gui::FormArrayInput<Eigen::Vector6d>{"dof", false, Eigen::Vector6d::Ones()}));
}

const mc_rbdyn::Robot & MCController::realRobot(std::string_view name) const
{
  return realRobots().robot(name);
}

mc_rbdyn::Robot & MCController::realRobot(std::string_view name)
{
  return realRobots().robot(name);
}

void MCController::supported_robots(std::vector<std::string> & out) const
{
  out = {};
}

bool MCController::hasContact(const mc_rbdyn::Contact & c) const
{
  return std::find(contacts().begin(), contacts().end(), c) != contacts().end();
}

void MCController::stop() {}

} // namespace mc_control
