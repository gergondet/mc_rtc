/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TransformTask.h>

#include <mc_rtc/gui/Transform.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

TransformTask::TransformTask(mc_rbdyn::Frame & frame, double stiffness, double weight)
: TrajectoryBase(frame.robot(), stiffness, weight)
{
  finalize(frame);
  type_ = "transform";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
}

/*! \brief Load parameters from a Configuration object */
void TransformTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TrajectoryBase::load(solver, config);

  // Current surface position
  auto X_0_t = errorT_->frame().position();

  // Apply global world transformations first
  if(config.has("targetSurface"))
  {
    mc_rtc::log::deprecated("TransformTask", "targetSurface", "target",
                            "Note: target should contain specific keys to mimic the behavior of targetSurface");
    const auto & c = config("targetSurface");
    const auto & robot = solver.robots().fromConfig(c, "TransformTask::targetSurface");
    sva::PTransformd offset = config("offset", sva::PTransformd::Identity());
    // FIXME Remove at a later point
    if(config.has("offset_rotation"))
    {
      mc_rtc::log::deprecated("TransformTask", "offset_rotation", "offset");
      offset.rotation() = config("offset_rotation");
    }
    if(config.has("offset_translation"))
    {
      mc_rtc::log::deprecated("TransformTask", "offset_translation", "offset");
      offset.translation() = config("offset_translation");
    }
    // FIXME Remove at a later point
    if(config.has("surface"))
    {
      mc_rtc::log::deprecated("TransformTask", "surface", "frame");
      target(robot.frame(config("surface")), offset);
    }
    else
    {
      target(robot.frame(config("frame")), offset);
    }
    X_0_t = this->target();
  }
  else if(config.has("target"))
  {
    const auto & target = config("target");
    if(target.has("frame"))
    {
      const auto & robot = solver.robots().fromConfig(target, "TransformTask::target");
      sva::PTransformd offset = target("offset", sva::PTransformd::Identity());
      X_0_t = offset * robot.frame(target("frame")).position();
    }
    else
    {
      X_0_t = config("target");
    }
  }
  else if(config.has("relative"))
  {
    const auto & relative = config("relative");
    const auto & robot = solver.robots().fromConfig(relative, "TransformTask::relative");
    std::string_view f1;
    if(relative.has("s1"))
    {
      mc_rtc::log::deprecated("TransformTask", "s1", "f1");
      f1 = relative("s1");
    }
    else
    {
      f1 = relative("f1");
    }
    std::string_view f2;
    if(relative.has("s2"))
    {
      mc_rtc::log::deprecated("TransformTask", "s2", "f2");
      f2 = relative("s2");
    }
    else
    {
      f2 = relative("f2");
    }
    sva::PTransformd target = config("relative")("target");
    const auto & X_0_f1 = robot.frame(f1).position();
    const auto & X_0_f2 = robot.frame(f2).position();
    auto X_f1_f2 = X_0_f2 * X_0_f1.inv();
    X_f1_f2.translation() = X_f1_f2.translation() / 2;
    auto X_0_relative = X_f1_f2 * X_0_f1;
    X_0_t = target * X_0_relative;
  }

  if(config.has("moveWorld"))
  {
    sva::PTransformd move = config("moveWorld");
    X_0_t = X_0_t * move;
  }
  else if(config.has("move"))
  {
    sva::PTransformd move = config("move");
    X_0_t = move * X_0_t;
  }

  this->target(X_0_t);
}

void TransformTask::target(const mc_rbdyn::Frame & f, const sva::PTransformd & offset)
{
  target(offset * f.position());
}

void TransformTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_pose", this,
                     [this]() -> const sva::PTransformd & { return errorT_->frame().position(); });
  logger.addLogEntry(name_ + "_target", this, [this]() -> const sva::PTransformd & { return target(); });
}

void TransformTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::Transform("target", [this]() -> const sva::PTransformd & { return this->target(); },
                             [this](const sva::PTransformd & pos) { this->target(pos); }),
      mc_rtc::gui::Transform("pose", [this]() -> const sva::PTransformd & { return errorT_->frame().position(); }));
}

} // namespace mc_tasks

namespace
{

template<bool Deprecated>
static mc_tasks::MetaTaskPtr loadTransformTask(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if constexpr(Deprecated)
  {
    mc_rtc::log::deprecated("TaskLoading", "surfaceTransform", "transform");
  }
  auto & robot = solver.robots().fromConfig(config, "TransformTask");
  // FIXME Remove after a while
  if(config.has("surface"))
  {
    mc_rtc::log::deprecated("TransformTask", "surface", "frame");
    auto t = std::make_shared<mc_tasks::TransformTask>(robot.frame(config("surface")));
    t->load(solver, config);
    return t;
  }
  else
  {
    auto t = std::make_shared<mc_tasks::TransformTask>(robot.frame(config("frame")));
    t->load(solver, config);
    return t;
  }
}

static auto reg_dep = mc_tasks::MetaTaskLoader::register_load_function("surfaceTransform", &loadTransformTask<true>);
static auto reg = mc_tasks::MetaTaskLoader::register_load_function("transform", &loadTransformTask<false>);

} // namespace
