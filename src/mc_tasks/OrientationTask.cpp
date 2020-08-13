/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/OrientationTask.h>

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
