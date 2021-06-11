/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/PositionTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>
#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

PositionTask::PositionTask(mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: TrajectoryBase(frame.robot(), stiffness, weight)
{

  finalize(frame);
  type_ = "position";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
}

void PositionTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target", this, [this]() { return position(); });
  logger.addLogEntry(name_ + "_curPos", this,
                     [this]() -> const Eigen::Vector3d & { return frame().position().translation(); });
  logger.addLogEntry(name_ + "_curVel", this,
                     [this]() -> const Eigen::Vector3d & { return frame().velocity().linear(); });
}

void PositionTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Point3D(
                     "pos_target", [this]() { return this->position(); },
                     [this](const Eigen::Vector3d & pos) { this->position(pos); }),
                 mc_rtc::gui::Point3D(
                     "pos", [this]() -> const Eigen::Vector3d & { return errorT_->frame().position().translation(); }));
}

} // namespace mc_tasks

static mc_tasks::MetaTaskPtr loadPositionTask(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  auto & robot = solver.robots().fromConfig(config, "PositionTask");
  if(config.has("body"))
  {
    mc_rtc::log::deprecated("PositionTask", "body", "frame");
    auto conf = config;
    if(config.has("bodyPoint"))
    {
      std::string bodyName = conf("body");
      Eigen::Vector3d bodyPoint = conf("bodyPoint");
      size_t i = 0;
      while(robot.hasFrame(fmt::format("{}_posTask_{}", bodyName, ++i)))
        ;
      std::string frameName = fmt::format("{}_posTask_{}", i);
      robot.makeFrame(frameName, bodyName, bodyPoint);
      conf.add("frame", frameName);
    }
    else
    {
      conf.add("frame", conf("body"));
    }
    conf.remove("body");
    return loadPositionTask(solver, conf);
  }
  auto t = std::make_shared<mc_tasks::PositionTask>(robot.frame(config("frame")));
  t->load(solver, config);
  if(config.has("position"))
  {
    t->position(config("position"));
  }
  if(config.has("relative"))
  {
    auto relative = config("relative");
    auto f1 = [&]() -> std::string_view {
      if(relative.has("s1"))
      {
        mc_rtc::log::deprecated("PositionTask", "s1", "f1");
        return relative("s1");
      }
      return relative("f1");
    }();
    auto f2 = [&]() -> std::string_view {
      if(relative.has("s2"))
      {
        mc_rtc::log::deprecated("PositionTask", "s2", "f2");
        return relative("s2");
      }
      return relative("f2");
    }();
    const auto & X_0_f1 = robot.frame(f1).position();
    const auto & X_0_f2 = robot.frame(f2).position();
    auto X_0_relative = sva::interpolate(X_0_f1, X_0_f2, 0.5);
    t->position((sva::PTransformd(relative("position", Eigen::Vector3d::Zero().eval())) * X_0_relative).translation());
  }
  return t;
}

static auto reg = mc_tasks::MetaTaskLoader::register_load_function("position", &loadPositionTask);
