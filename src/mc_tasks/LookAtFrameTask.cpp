/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/LookAtFrameTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

LookAtFrameTask::LookAtFrameTask(mc_rbdyn::RobotFrame & frame,
                                 const Eigen::Vector3d & frameVector,
                                 mc_rbdyn::FreeFrame & targetFrame,
                                 double stiffness,
                                 double weight)
: LookAtTask(frame, frameVector, stiffness, weight), targetFrame_(targetFrame), offset_(sva::PTransformd::Identity())
{
  type_ = "lookAtFrame";
  name_ = fmt::format("{}_{}_{}_{}", type_, robot().name(), frame.name(), targetFrame.name());
  errorT_->addFrameDependency(targetFrame);
}

void LookAtFrameTask::update(mc_solver::QPSolver &)
{
  LookAtTask::target((offset_ * targetFrame_->position()).translation());
}

} // namespace mc_tasks

namespace
{

static mc_tasks::LookAtFrameTaskPtr loadLookAtFrameTask(mc_solver::QPSolver & solver,
                                                        const mc_rtc::Configuration & configIn)
{
  auto config = configIn;
  auto & robot = solver.robots().fromConfig(config, "LookAtFrame");
  if(config.has("body"))
  {
    mc_rtc::log::deprecated("LookAtFrameTask", "body", "frame");
    config.add("frame", config("body"));
  }
  if(config.has("bodyVector"))
  {
    mc_rtc::log::deprecated("LookAtFrameTask", "bodyVector", "frameVector");
    config.add("frameVector", config("bodyVector"));
  }
  if(config.has("surfaceRobot"))
  {
    mc_rtc::log::deprecated("LookAtFrameTask", "surfaceRobot", "targetRobot");
    config.add("targetRobot", config("surfaceRobot"));
  }
  if(config.has("surface"))
  {
    mc_rtc::log::deprecated("LookAtFrameTask", "surface", "targetFrame");
    config.add("targetFrame", config("surface"));
  }
  auto & targetRobot = solver.robots().fromConfig(config, "LookAtFrame", false, "", "targetRobot");
  auto & targetFrame = targetRobot.frame(config("targetFrame"));
  auto t =
      std::make_shared<mc_tasks::LookAtFrameTask>(robot.frame(config("frame")), config("frameVector"), targetFrame);
  if(config.has("offset"))
  {
    t->offset(config("offset"));
  }
  t->load(solver, config);
  return t;
}

static auto registered_lookat_frame = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAtFrame",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & configIn) {
      return loadLookAtFrameTask(solver, configIn);
    });

static auto registered_lookat_surface = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAtSurface",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & configIn) {
      mc_rtc::log::deprecated("TaskLoading", "lookAtSurface", "lookAtFrame");
      return loadLookAtFrameTask(solver, configIn);
    });
} // namespace
