/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/OrientationTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>
#include <mc_rtc/gui/Rotation.h>

namespace mc_tasks
{

OrientationTask::OrientationTask(mc_rbdyn::Frame & frame, double stiffness, double weight)
: TrajectoryBase(frame.robot(), stiffness, weight)
{
  finalize(frame);
  type_ = "orientation";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
}

void OrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::Rotation("ori_target",
                            [this]() -> sva::PTransformd {
                              sva::PTransformd out = errorT_->frame().position();
                              out.rotation() = errorT_->orientation();
                              return out;
                            },
                            [this](const Eigen::Quaterniond & ori) { this->orientation(ori.toRotationMatrix()); }),
      mc_rtc::gui::Rotation("ori", [this]() -> const sva::PTransformd & { return errorT_->frame().position(); }));
}

void OrientationTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target", this, [this]() { return Eigen::Quaterniond(orientation()); });
  logger.addLogEntry(name_, this, [this]() { return Eigen::Quaterniond(errorT_->frame().position().rotation()); });
}

} // namespace mc_tasks

static mc_tasks::MetaTaskPtr loadOrientationTask(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("body"))
  {
    mc_rtc::log::deprecated("OrientationTask", "body", "frame");
    auto conf = config;
    conf.add("frame", conf("body"));
    conf.remove("body");
    return loadOrientationTask(solver, conf);
  }
  auto & robot = solver.robots().fromConfig(config, "OrientationTask");
  auto t = std::make_shared<mc_tasks::OrientationTask>(robot.frame(config("frame")));
  t->load(solver, config);
  if(config.has("orientation"))
  {
    t->orientation(config("orientation"));
  }
  if(config.has("relative"))
  {
    auto relative = config("relative");
    auto f1 = [&]() -> std::string_view {
      if(relative.has("s1"))
      {
        mc_rtc::log::deprecated("OrientationTask", "s1", "f1");
        return relative("s1");
      }
      return relative("f1");
    }();
    auto f2 = [&]() -> std::string_view {
      if(relative.has("s2"))
      {
        mc_rtc::log::deprecated("OrientationTask", "s2", "f2");
        return relative("s2");
      }
      return relative("f2");
    }();
    const auto & X_0_f1 = robot.frame(f1).position();
    const auto & X_0_f2 = robot.frame(f2).position();
    auto X_0_relative = sva::interpolate(X_0_f1, X_0_f2, 0.5);
    t->orientation(
        (sva::PTransformd(relative("orientation", Eigen::Matrix3d::Identity().eval())) * X_0_relative).rotation());
  }
  return t;
}

static auto reg = mc_tasks::MetaTaskLoader::register_load_function("orientation", &loadOrientationTask);
