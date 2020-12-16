/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_filter/utils/clamp.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/DampingTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

namespace force
{

using mc_filter::utils::clampInPlaceAndWarn;

DampingTask::DampingTask(mc_rbdyn::Frame & frame, double stiffness, double weight)
: AdmittanceTask(frame, stiffness, weight)
{
  type_ = "admittance";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
  reset();
}

void DampingTask::update(mc_solver::QPSolver &)
{
  wrenchError_ = measuredWrench() - targetWrench_;

  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");
  refVelB_ = feedforwardVelB_ + sva::MotionVecd{angularVel, linearVel};

  // SC: we could do add an anti-windup strategy here, e.g. back-calculation.
  // Yet, keep in mind that our velocity bounds are artificial. Whenever
  // possible, the best is to set to gains so that they are not saturated.

  TransformTask::refVelB(refVelB_);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "damping",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      std::shared_ptr<mc_tasks::force::DampingTask> t;
      auto & robot = solver.robots().fromConfig(config, "DampingTask");
      if(config.has("surface"))
      {
        mc_rtc::log::warning("Deprecated use of surface while loading a DampingTask, use \"frame\" instead");
        t = std::make_shared<mc_tasks::force::DampingTask>(robot.frame(config("surface")));
      }
      else
      {
        t = std::make_shared<mc_tasks::force::DampingTask>(robot.frame(config("frame")));
      }
      if(config.has("admittance"))
      {
        t->admittance(config("admittance"));
      }
      if(config.has("damping"))
      {
        double d = config("damping");
        t->damping(d);
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
      if(config.has("wrench"))
      {
        t->targetWrench(config("wrench"));
      }
      t->load(solver, config);
      return t;
    });
}
