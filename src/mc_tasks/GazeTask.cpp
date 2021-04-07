/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/GazeTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

GazeTask::GazeTask(mc_rbdyn::Frame & frame, double stiffness, double weight)
: TrajectoryBase(frame.robot(), stiffness, weight)
{
  type_ = "gaze";
  name_ = fmt::format("{}_{}", type_, frame.name());
  finalize(frame);
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "gaze",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config_) {
      auto config = config_;
      auto & robot = solver.robots().fromConfig(config, "GazeTask");
      if(config.has("body"))
      {
        mc_rtc::log::deprecated("GazeTask", "body", "frame");
        config.add("frame", config("body"));
      }
      auto t = std::make_shared<mc_tasks::GazeTask>(robot.frame(config("frame")));
      t->load(solver, config);
      return t;
    });
}
