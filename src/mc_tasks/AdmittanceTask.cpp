/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/AdmittanceTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

namespace force
{

using mc_filter::utils::clampInPlaceAndWarn;

AdmittanceTask::AdmittanceTask(mc_rbdyn::Frame & frame, double stiffness, double weight)
: TransformTask(frame, stiffness, weight)
{
  if(!frame.hasForceSensor())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks::AdmittanceTask] Frame {} does not have a force sensor attached", frame.name());
  }
  type_ = "admittance";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
  reset();
}

void AdmittanceTask::update(mc_solver::QPSolver &)
{
  // Compute wrench error
  wrenchError_ = measuredWrench() - targetWrench_;

  // Compute linear and angular velocity based on wrench error and admittance
  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());

  // Clamp both values in order to have a 'security'
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");

  // Filter
  refVelB_ = velFilterGain_ * refVelB_ + (1 - velFilterGain_) * sva::MotionVecd(angularVel, linearVel);

  // Compute position and rotation delta
  sva::PTransformd delta(mc_rbdyn::rpyToMat(timestep_ * refVelB_.angular()), timestep_ * refVelB_.linear());

  // Apply feed forward term
  refVelB_ += feedforwardVelB_;

  // Acceleration
  TransformTask::refAccel((refVelB_ - TransformTask::refVelB()) / timestep_);

  // Velocity
  TransformTask::refVelB(refVelB_);

  // Position
  target(delta * target());
}

void AdmittanceTask::reset()
{
  TransformTask::reset();
  admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  feedforwardVelB_ = sva::MotionVecd(Eigen::Vector6d::Zero());
  targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());
}

/*! \brief Load parameters from a Configuration object */
void AdmittanceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("admittance"))
  {
    admittance(config("admittance"));
  }
  else if(config.has("targetPose"))
  {
    mc_rtc::log::warning("[{}] property \"targetPose\" is deprecated, use \"target\" instead", name());
    targetPose(config("targetPose"));
  }
  if(config.has("wrench"))
  {
    targetWrench(config("wrench"));
  }
  if(config.has("refVelB"))
  {
    refVelB(config("refVelB"));
  }
  if(config.has("maxVel"))
  {
    sva::MotionVecd maxVel = config("maxVel");
    maxLinearVel(maxVel.linear());
    maxAngularVel(maxVel.angular());
  }
  TransformTask::load(solver, config);
}

void AdmittanceTask::addToLogger(mc_rtc::Logger & logger)
{
  TransformTask::addToLogger(logger);
  MC_RTC_LOG_HELPER(name_ + "_admittance", admittance_);
  MC_RTC_LOG_GETTER(name_ + "_measured_wrench", measuredWrench);
  MC_RTC_LOG_HELPER(name_ + "_target_body_vel", feedforwardVelB_);
  MC_RTC_LOG_HELPER(name_ + "_target_wrench", targetWrench_);
  MC_RTC_LOG_HELPER(name_ + "_vel_filter_gain", velFilterGain_);
}

void AdmittanceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Transform("pos_target", [this]() { return this->targetPose(); },
                                        [this](const sva::PTransformd & pos) { this->targetPose(pos); }),
                 mc_rtc::gui::Transform("pos", [this]() -> const sva::PTransformd & { return pose(); }),
                 mc_rtc::gui::ArrayInput("admittance", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->admittance().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->admittance(a); }),
                 mc_rtc::gui::ArrayInput("wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->targetWrench().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->targetWrench(a); }),
                 mc_rtc::gui::ArrayLabel("measured_wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->measuredWrench().vector(); }),
                 mc_rtc::gui::NumberInput("Velocity filter gain", [this]() { return velFilterGain_; },
                                          [this](double g) { velFilterGain(g); }));
  // Don't add SurfaceTransformTask as target configuration is different
  TrajectoryBase::addToGUI(gui);
}

void AdmittanceTask::addToSolver(mc_solver::QPSolver & solver)
{
  timestep_ = solver.dt();
  TransformTask::addToSolver(solver);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "admittance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto & robot = solver.robots().fromConfig(config, "AdmittanceTask");
      if(config.has("surface"))
      {
        mc_rtc::log::warning("Deprecate use of surface while loading an AdmittanceTask, use \"frame\" instead");
        auto t = std::make_shared<mc_tasks::force::AdmittanceTask>(robot.frame(config("surface")));
        t->reset();
        t->load(solver, config);
        return t;
      }
      auto t = std::make_shared<mc_tasks::force::AdmittanceTask>(robot.frame(config("frame")));
      t->reset();
      t->load(solver, config);
      return t;
    });
}
