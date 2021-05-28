/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/VectorOrientationTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>
#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

VectorOrientationTask::VectorOrientationTask(mc_rbdyn::Frame & frame,
                                             const Eigen::Vector3d & frameVector,
                                             double stiffness,
                                             double weight)
: TrajectoryBase(frame.robot(), stiffness, weight)
{
  finalize(frame, frameVector);
  type_ = "vectorOrientation";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
  bIndex_ = frame.robot().bodyIndexByName(frame.body());
}

void VectorOrientationTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  MC_RTC_LOG_GETTER(name_ + "_target", targetVector);
  MC_RTC_LOG_GETTER(name_ + "_current", actual);
  MC_RTC_LOG_GETTER(name_ + "_error", eval);
}

void VectorOrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::ArrayInput(
                     "Target Direction", {"x", "y", "z"}, [this]() { return targetVector(); },
                     [this](const Eigen::Vector3d & target) { targetVector(target); }),
                 mc_rtc::gui::Arrow(
                     "Actual", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 0., 1.)),
                     [this]() -> const Eigen::Vector3d & { return bodyPos(); },
                     [this]() -> Eigen::Vector3d { return bodyPos() + 0.25 * actual(); }),
                 mc_rtc::gui::Arrow(
                     "Target", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(1., 0., 0.)),
                     [this]() -> const Eigen::Vector3d & { return bodyPos(); },
                     [this]() -> Eigen::Vector3d { return bodyPos() + .25 * targetVector(); }),
                 mc_rtc::gui::Point3D(
                     "Arrow end point", [this]() -> Eigen::Vector3d { return bodyPos() + .25 * targetVector(); },
                     [this](const Eigen::Vector3d & point) {
                       Eigen::Vector3d direction = point - bodyPos();
                       targetVector(direction);
                     }));
}
} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "vectorOrientation",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & configIn) {
      auto config = configIn;
      auto & robot = solver.robots().fromConfig(config, "VectorOrientationTask");
      if(config.has("body"))
      {
        mc_rtc::log::deprecated("VectorOrientationTask", "body", "frame");
        config.add("frame", config("body"));
      }
      if(config.has("bodyVector"))
      {
        mc_rtc::log::deprecated("VectorOrientationTask", "bodyVector", "frameVector");
        config.add("frameVector", config("bodyVector"));
      }
      auto t = std::make_shared<mc_tasks::VectorOrientationTask>(robot.frame(config("frame")), config("frameVector"));
      t->load(solver, config);
      if(config.has("targetVector"))
      {
        t->targetVector(config("targetVector"));
      }
      return t;
    });
}
