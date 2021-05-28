/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/LookAtTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>
#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

LookAtTask::LookAtTask(mc_rbdyn::Frame & frame, const Eigen::Vector3d & frameVector, double stiffness, double weight)
: VectorOrientationTask(frame, frameVector, stiffness, weight)
{
  type_ = "lookAt";
  name_ = fmt::format("{}_{}_{}", type_, robot().name(), frame.name());
}

void LookAtTask::reset()
{
  target(bodyPos() + frameVector());
}

void LookAtTask::target(const Eigen::Vector3d & target)
{
  target_ = target;
  auto target_ori = (target - bodyPos()).normalized();
  VectorOrientationTask::targetVector(target_ori);
}

void LookAtTask::addToLogger(mc_rtc::Logger & logger)
{
  VectorOrientationTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_target_pos", this, [this]() -> const Eigen::Vector3d { return target(); });
  logger.addLogEntry(name_ + "_current_pos", this, [this]() -> const Eigen::Vector3d & { return bodyPos(); });
}

void LookAtTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  VectorOrientationTask::addToGUI(gui);
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Point3D(
                                       "Target Point", [this]() { return this->target(); },
                                       [this](const Eigen::Vector3d & pos) { this->target(pos); }));
}

} // namespace mc_tasks

namespace
{
static auto registered_lookat = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAt",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & configIn) {
      auto config = configIn;
      auto & robot = solver.robots().fromConfig(config, "LookAt");
      if(config.has("body"))
      {
        mc_rtc::log::deprecated("LookAtTask", "body", "frame");
        config.add("frame", config("body"));
      }
      if(config.has("bodyVector"))
      {
        mc_rtc::log::deprecated("LookAtTask", "bodyVector", "frameVector");
        config.add("frameVector", config("bodyVector"));
      }
      auto t = std::make_shared<mc_tasks::LookAtTask>(robot.frame(config("frame")), config("frameVector"));
      t->load(solver, config);
      if(config.has("targetPos"))
      {
        t->target(config("targetPos"));
      }
      if(config.has("targetVector"))
      {
        t->targetVector(config("targetVector"));
      }
      if(config.has("relativeVector"))
      {
        // FIXME Strange specification as this is the same robot
        auto bodyPos = solver.robots().fromConfig(config, t->name()).posW();
        bodyPos.translation() = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = config("relativeVector");
        sva::PTransformd target{v};
        t->targetVector((target * bodyPos).translation());
      }
      return t;
    });
}
