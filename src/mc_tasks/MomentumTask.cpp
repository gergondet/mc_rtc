/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/MomentumTask.h>

#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

MomentumTask::MomentumTask(mc_rbdyn::Robot & robot, double stiffness, double weight)
: TrajectoryBase(robot, stiffness, weight)
{
  finalize(robot);
  type_ = "momentum";
  name_ = "momentum_" + robot.name();
}

void MomentumTask::reset()
{
  TrajectoryBase::reset();
  momentum(robot_->momentum());
}

/*! \brief Load parameters from a Configuration object */
void MomentumTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("momentum"))
  {
    this->momentum(config("momentum"));
  }
  TrajectoryBase::load(solver, config);
}

void MomentumTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target_momentum", this, [this]() -> const sva::ForceVecd & { return momentum(); });
  logger.addLogEntry(name_ + "_momentum", this, [this]() -> const sva::ForceVecd & { return robot_->momentum(); });
}

void MomentumTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::ArrayInput(
                     "target", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> const sva::ForceVecd & { return this->momentum(); },
                     [this](const sva::ForceVecd & m) { this->momentum(m); }),
                 mc_rtc::gui::ArrayLabel("momentum", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() -> const sva::ForceVecd & { return robot_->momentum(); }));
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "momentum",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::MomentumTask>(solver.robots().fromConfig(config, "MomentumTask"));
      t->load(solver, config);
      return t;
    });
}
