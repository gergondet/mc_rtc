/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/VelocityTask.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

VelocityTask::VelocityTask(mc_rbdyn::Frame & frame, double stiffness, double weight)
: TrajectoryBase(frame.robot(), stiffness, weight)
{
  finalize(frame, Eigen::Vector6d::Ones());
  type_ = "velocity";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
}

} // namespace mc_tasks

static auto reg = mc_tasks::MetaTaskLoader::register_load_function(
    "velocity",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto & robot = solver.robots().fromConfig(config, "VelocityTask");
      auto t = std::make_shared<mc_tasks::VelocityTask>(robot.frame(config("frame")));
      t->load(solver, config);
      return t;
    });
