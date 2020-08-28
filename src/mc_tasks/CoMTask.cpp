/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_tasks/CoMTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/Surface.h>

#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

CoMTask::CoMTask(mc_rbdyn::Robot & robot, double stiffness, double weight) : TrajectoryBase(robot, stiffness, weight)
{
  finalize(robot_);
  type_ = "com";
  name_ = "com_" + robot.name();
}

void CoMTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TrajectoryBase::load(solver, config);
  if(config.has("com"))
  {
    this->com(config("com"));
  }
  if(config.has("above"))
  {
    auto surfaces = mc_rtc::fromVectorOrElement<std::string>(config, "above");
    auto com = this->com();
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    std::string rName = config("robot", solver.robots().begin()->get()->name());
    auto & robot = solver.robots().robot(rName);
    for(const auto & s : surfaces)
    {
      target += robot.surface(s).frame().position().translation();
    }
    target /= static_cast<double>(surfaces.size());
    this->com({target.x(), target.y(), com.z()});
  }
  if(config.has("move_com"))
  {
    Eigen::Vector3d move = config("move_com");
    com(com() + move);
  }
  if(config.has("offset"))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED][" + name()
                         + "] The \"offset\" property is deprecated, use move_com instead");
    Eigen::Vector3d offset = config("offset", Eigen::Vector3d::Zero().eval());
    this->com(this->com() + offset);
  }
}

void CoMTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  MC_RTC_LOG_GETTER(name_ + "_pos", actual);
  MC_RTC_LOG_GETTER(name_ + "_target", com);
}

void CoMTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Point3D("target", [this]() -> const Eigen::Vector3d & { return this->com(); },
                                      [this](const Eigen::Vector3d & com) { this->com(com); }),
                 mc_rtc::gui::Point3D("com", [this]() -> const Eigen::Vector3d & { return this->actual(); }));
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "com",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::CoMTask>(solver.robots().fromConfig(config, "CoMTask"));
      t->load(solver, config);
      return t;
    });
}
