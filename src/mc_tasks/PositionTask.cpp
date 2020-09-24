/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/PositionTask.h>

#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

PositionTask::PositionTask(mc_rbdyn::Frame & frame, double stiffness, double weight)
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
  logger.addLogEntry(name_ + "_curVel", this, [this]() -> const Eigen::Vector3d & { return frame().velocity().linear(); });
}

void PositionTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Point3D("pos_target", [this]() { return this->position(); },
                                      [this](const Eigen::Vector3d & pos) { this->position(pos); }),
                 mc_rtc::gui::Point3D(
                     "pos", [this]() -> const Eigen::Vector3d & { return errorT_->frame().position().translation(); }));
}

} // namespace mc_tasks
