/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_controller.h>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/config.h>
#include <mc_rtc/gui/Schema.h>
#include <mc_rtc/logging.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <array>
#include <fstream>

namespace mc_control
{

MCController::MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt)
: MCController({robot, mc_rbdyn::RobotLoader::get_robot_module("env",
                                                               std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                               std::string("ground"))},
               dt)
{
}

MCController::MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots_modules, double dt)
: qpsolver(std::make_shared<mc_solver::QPSolver>(dt)),
  logger_(std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED, "", "")),
  gui_(std::make_shared<mc_rtc::gui::StateBuilder>()), timeStep(dt)
{
  /* Load robots */
  qpsolver->logger(logger_);
  qpsolver->gui(gui_);
  for(auto rm : robots_modules)
  {
    loadRobot(rm, rm->name);
  }

  if(gui_)
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
                         LOG_ERROR("Failed to load MetaTask from request\n" << config.dump(true))
                       }
                     }));
  }

  /* Initialize grippers */
  const auto & mimics = robots_modules[0]->gripperMimics();
  if(mimics.size())
  {
    for(const auto & gripper : robots_modules[0]->grippers())
    {
      if(!mimics.count(gripper.name))
      {
        LOG_ERROR_AND_THROW(std::runtime_error, "Mimics information not specified for gripper: " << gripper.name)
      }
      grippers[gripper.name] = std::make_shared<mc_control::Gripper>(robot(), gripper.joints, mimics.at(gripper.name),
                                                                     std::vector<double>(gripper.joints.size(), 0.0),
                                                                     timeStep, gripper.reverse_limits);
    }
  }
  else
  {
    std::string urdfPath = robots_modules[0]->urdf_path;
    std::ifstream ifs(urdfPath);
    if(ifs.is_open())
    {
      std::stringstream urdf;
      urdf << ifs.rdbuf();
      auto urdfRobot = mc_rbdyn::loadRobotFromUrdf("temp_robot", urdf.str());
      for(const auto & gripper : robots_modules[0]->grippers())
      {
        grippers[gripper.name] = std::make_shared<mc_control::Gripper>(urdfRobot->robot(), gripper.joints, urdf.str(),
                                                                       std::vector<double>(gripper.joints.size(), 0.0),
                                                                       timeStep, gripper.reverse_limits);
      }
    }
    else
    {
      LOG_ERROR("Could not open urdf file " << urdfPath << " for robot " << robots_modules[0]->name
                                            << ", cannot initialize grippers")
      LOG_ERROR_AND_THROW(std::runtime_error, "Failed to initialize grippers")
    }
  }

  /* Initialize constraints and tasks */
  std::array<double, 3> damper = {0.1, 0.01, 0.5};
  contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Velocity);
  dynamicsConstraint = mc_solver::DynamicsConstraint(robots(), 0, timeStep, damper, 0.5);
  kinematicsConstraint = mc_solver::KinematicsConstraint(robots(), 0, timeStep, damper, 0.5);
  selfCollisionConstraint = mc_solver::CollisionsConstraint(robots(), 0, 0, timeStep);
  selfCollisionConstraint.addCollisions(solver(), robots_modules[0]->minimalSelfCollisions());
  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), 0, 10.0, 5.0);
  LOG_INFO("MCController(base) ready")
}

MCController::~MCController() {}

mc_rbdyn::Robot & MCController::loadRobot(mc_rbdyn::RobotModulePtr rm, const std::string & name)
{
  assert(rm);
  auto & r = robots().load(*rm);
  r.name(name);
  r.mbc().gravity = Eigen::Vector3d{0, 0, 9.81};
  r.forwardKinematics();
  r.forwardVelocity();
  if(gui_)
  {
    auto data = gui_->data();
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
  }
  return r;
}

bool MCController::resetObservers()
{
  auto pipelineDesc = std::string{};
  for(const auto & observerPair : pipelineObservers_)
  {
    auto observer = observerPair.first;
    bool updateRobots = observerPair.second;
    observer->reset(*this);

    if(updateRobots)
    {
      observer->updateRobots(*this, realRobots());
      pipelineDesc += " -> " + observer->desc();
    }
    else
    {
      pipelineDesc += " -> [" + observer->desc() + "]";
    }
    observer->addToLogger(*this, logger());
    if(gui_)
    {
      observer->addToGUI(*this, *gui_);
    }
  }
  if(!pipelineObservers_.empty())
  {
    LOG_INFO("Observers: " << pipelineDesc);
  }
  return true;
}

bool MCController::runObservers()
{
  for(const auto & observerPair : pipelineObservers_)
  {
    auto observer = observerPair.first;
    bool updateRobots = observerPair.second;
    bool r = observer->run(*this);
    if(!r)
    {
      LOG_ERROR("Observer " << observer->name() << " failed to run");
      return false;
    }
    if(updateRobots)
    {
      observer->updateRobots(*this, realRobots());
    }
  }
  return true;
}

bool MCController::run()
{
  return run(mc_solver::FeedbackType::None);
}

bool MCController::run(mc_solver::FeedbackType fType)
{
  if(!qpsolver->run(fType))
  {
    LOG_ERROR("QP failed to run()")
    return false;
  }
  qpsolver->fillTorque(dynamicsConstraint);
  return true;
}

bool MCController::runClosedLoop()
{
  if(!qpsolver->runClosedLoop(real_robots))
  {
    LOG_ERROR("QP failed to run()")
    return false;
  }
  qpsolver->fillTorque(dynamicsConstraint);
  return true;
}

const mc_solver::QPResultMsg & MCController::send(const double & t)
{
  return qpsolver->send(t);
}

void MCController::reset(const ControllerResetData & reset_data)
{
  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  postureTask->posture(reset_data.q);
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
}

const mc_rbdyn::Robot & MCController::robot() const
{
  return qpsolver->robot();
}

const mc_rbdyn::Robot & MCController::env() const
{
  return qpsolver->env();
}

mc_rbdyn::Robot & MCController::robot()
{
  return qpsolver->robot();
}

mc_rbdyn::Robot & MCController::env()
{
  return qpsolver->env();
}

const mc_rbdyn::Robots & MCController::robots() const
{
  return qpsolver->robots();
}

mc_rbdyn::Robots & MCController::robots()
{
  return qpsolver->robots();
}

const mc_solver::QPSolver & MCController::solver() const
{
  assert(qpsolver);
  return *qpsolver;
}

mc_solver::QPSolver & MCController::solver()
{
  assert(qpsolver);
  return *qpsolver;
}

mc_rtc::Logger & MCController::logger()
{
  return *logger_;
}

std::vector<std::string> MCController::supported_robots() const
{
  return {};
}

const mc_rbdyn::Robots & MCController::realRobots() const
{
  return *real_robots;
}

mc_rbdyn::Robots & MCController::realRobots()
{
  return *real_robots;
}

const mc_rbdyn::Robot & MCController::realRobot() const
{
  return real_robots->robot();
}

mc_rbdyn::Robot & MCController::realRobot()
{
  return real_robots->robot();
}

sva::PTransformd MCController::anchorFrame(const mc_rbdyn::Robot &) const
{
  LOG_ERROR_AND_THROW(std::runtime_error, "MCController::anchorFrame() requested but no implementation available. "
                                          "Please override this function in your controller.");
}

void MCController::stop() {}

} // namespace mc_control
