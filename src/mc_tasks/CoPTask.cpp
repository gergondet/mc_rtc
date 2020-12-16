/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/CoPTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

#include <mc_rtc/gui/ArrayLabel.h>

namespace mc_tasks
{

namespace force
{

CoPTask::CoPTask(mc_rbdyn::Frame & frame,
                 double stiffness,
                 double weight)
: DampingTask(frame, stiffness, weight)
{
  type_ = "cop";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
}

void CoPTask::reset()
{
  targetCoP_ = Eigen::Vector2d::Zero();
  targetForce_ = Eigen::Vector3d::Zero();
  DampingTask::reset();
}

void CoPTask::update(mc_solver::QPSolver & solver)
{
  double pressure = std::max(0., measuredWrench().force().z());
  Eigen::Vector3d targetTorque = {+targetCoP_.y() * pressure, -targetCoP_.x() * pressure, 0.};
  DampingTask::targetWrench({targetTorque, targetForce_});
  DampingTask::update(solver);
}

void CoPTask::addToLogger(mc_rtc::Logger & logger)
{
  DampingTask::addToLogger(logger);
  MC_RTC_LOG_HELPER(name_ + "_measured_cop", measuredCoP);
  MC_RTC_LOG_HELPER(name_ + "_target_cop", targetCoP_);
}

void CoPTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::ArrayLabel("cop_measured", [this]() -> Eigen::Vector2d { return this->measuredCoP(); }),
                 mc_rtc::gui::ArrayInput("cop_target",
                                         [this]() -> const Eigen::Vector2d & { return this->targetCoP(); },
                                         [this](const Eigen::Vector2d & cop) { this->targetCoP(cop); }));
  // Don't add SurfaceTransformTask as target configuration is different
  DampingTask::addToGUI(gui);
}

std::function<bool(const mc_tasks::MetaTask &, std::string &)> CoPTask::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{
  if(config.has("copError"))
  {
    double copError = config("copError");
    assert(copError >= 0);
    return [copError](const mc_tasks::MetaTask & t, std::string & out) {
      const auto & self = static_cast<const CoPTask &>(t);
      Eigen::Vector2d error = self.measuredCoP() - self.targetCoP();
      if(error.norm() < copError)
      {
        out += "CoP error";
        return true;
      }
      return false;
    };
  }
  if(config.has("force"))
  {
    Eigen::Vector3d force = config("force");
    Eigen::Vector3d dof = Eigen::Vector3d::Ones();
    for(int i = 0; i < 3; ++i)
    {
      if(std::isnan(force(i)))
      {
        dof(i) = 0.;
        force(i) = 0.;
      }
      else if(force(i) < 0)
      {
        dof(i) = -1.;
      }
    }
    return [dof, force](const mc_tasks::MetaTask & t, std::string & out) {
      const auto & self = static_cast<const CoPTask &>(t);
      Eigen::Vector3d f = self.measuredWrench().force();
      for(int i = 0; i < 3; ++i)
      {
        if(dof(i) * fabs(f(i)) < force(i))
        {
          return false;
        }
      }
      out += "force";
      return true;
    };
  }
  return DampingTask::buildCompletionCriteria(dt, config);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "cop",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      std::shared_ptr<mc_tasks::force::CoPTask> t;
      auto & robot = solver.robots().fromConfig(config, "CoPTask");
      using Allocator = Eigen::aligned_allocator<mc_tasks::force::CoPTask>;
      if(config.has("surface"))
      {
        mc_rtc::log::warning("Deprecated use of surface while loading a CoPTask, use \"frame\" instead");
        t = std::allocate_shared<mc_tasks::force::CoPTask>(Allocator{}, robot.frame(config("surface")));
      }
      else
      {
        t = std::allocate_shared<mc_tasks::force::CoPTask>(Allocator{}, robot.frame(config("frame")));
      }
      if(config.has("admittance"))
      {
        t->admittance(config("admittance"));
      }
      if(config.has("cop"))
      {
        t->targetCoP(config("cop"));
      }
      if(config.has("force"))
      {
        t->targetForce(config("force"));
      }
      if(config.has("targetSurface"))
      {
        mc_rtc::log::warning(
            "Deprecated use of targetSurface while loading a DampingTask, use \"targetFrame\" instead");
        const auto & c = config("targetSurface");
        const auto & r = solver.robots().fromConfig(c, t->name() + "::targetSurface");
        t->targetPose(r.frame(c("surface")), {c("offset_rotation", Eigen::Matrix3d::Identity().eval()),
                                              c("offset_translation", Eigen::Vector3d::Zero().eval())});
      }
      else if(config.has("targetFrame"))
      {
        const auto & c = config("targetFrame");
        const auto & r = solver.robots().fromConfig(c, t->name() + "::targetFrame");
        const auto & frame = r.frame(c("frame"));
        if(c.has("offset"))
        {
          t->targetPose(frame, c("offset", sva::PTransformd::Identity()));
        }
        else
        {
          t->targetPose(frame, {c("offset_rotation", Eigen::Matrix3d::Identity().eval()),
                                c("offset_translation", Eigen::Vector3d::Zero().eval())});
        }
      }
      else if(config.has("targetPose"))
      {
        t->targetPose(config("targetPose"));
      }
      if(config.has("weight"))
      {
        t->weight(config("weight"));
      }
      t->load(solver, config);
      return t;
    });
}
