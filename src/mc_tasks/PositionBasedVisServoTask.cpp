/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/PositionBasedVisServoTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>

#include <Eigen/Geometry>

namespace mc_tasks
{

PositionBasedVisServoTask::PositionBasedVisServoTask(mc_rbdyn::Frame & frame, double stiffness, double weight)
: TrajectoryBase(frame.robot(), stiffness, weight)
{
  type_ = "pbvs";
  name_ = fmt::format("{}_{}", type_, frame.name());
  finalize(frame);
}

void PositionBasedVisServoTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_error", this, [this]() -> const sva::PTransformd & { return error(); });
  logger.addLogEntry(name_ + "_eval", this, [this]() -> sva::PTransformd {
    Eigen::Vector6d eval = errorT_->value();
    Eigen::Vector3d angleAxis = eval.head(3);
    Eigen::Vector3d axis = angleAxis / angleAxis.norm();
    double angle = angleAxis.dot(axis);
    Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, axis));
    return sva::PTransformd(quat, eval.tail(3));
  });
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "pbvs",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      std::shared_ptr<mc_tasks::PositionBasedVisServoTask> t;
      using Allocator = Eigen::aligned_allocator<mc_tasks::PositionBasedVisServoTask>;
      auto & robot = solver.robots().fromConfig(config, "PositionBasedVisServoTask");
      if(config.has("surface"))
      {
        mc_rtc::log::deprecated("PositionBasedVisServoTask", "surface", "frame");
        t = std::allocate_shared<mc_tasks::PositionBasedVisServoTask>(Allocator{}, robot.frame(config("surface")));
      }
      else if(config.has("body"))
      {
        mc_rtc::log::deprecated("PositionBasedVisServoTask", "body", "frame");
        t = std::allocate_shared<mc_tasks::PositionBasedVisServoTask>(Allocator{}, robot.frame(config("body")));
      }
      else
      {
        t = std::allocate_shared<mc_tasks::PositionBasedVisServoTask>(Allocator{}, robot.frame(config("frame")));
      }
      t->load(solver, config);
      return t;
    });
}
