/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/ComplianceTask.h>

#include <mc_filter/utils/clamp.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

namespace force
{

ComplianceTask::ComplianceTask(mc_rbdyn::Frame & frame,
                               const Eigen::Vector6d & dof,
                               double stiffness,
                               double weight,
                               double forceThresh,
                               double torqueThresh,
                               std::pair<double, double> forceGain,
                               std::pair<double, double> torqueGain)
: task_(std::make_shared<TransformTask>(frame, stiffness, weight)), forceThresh_(forceThresh),
  torqueThresh_(torqueThresh), forceGain_(forceGain), torqueGain_(torqueGain), dof_(dof)
{
  type_ = "compliance";
  name_ = fmt::format("{}_{}", type_, frame.name());
  if(!frame.hasForceSensor())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}] No force sensor accessible through {}, you cannot use a compliance task", name_, frame.name());
  }
}

void ComplianceTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!task_->inSolver())
  {
    firstUpdate_ = true;
  }
  MetaTask::addToSolver(*task_, solver);
}

void ComplianceTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*task_, solver);
}

sva::PTransformd ComplianceTask::computePose()
{
  Eigen::Vector3d trans = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  auto clamper = [](double limit) {
    return [limit](double value) { return mc_filter::utils::clamp(value, -limit, limit); };
  };
  // FIXME Clamper values are hard-coded here, this follows existing practice in mc_rtc 1.x but should be fixed
  if(error_.force().norm() > forceThresh_)
  {
    trans = (forceGain_.first * error_.force() + forceGain_.second * errorD_.force()).unaryExpr(clamper(0.01));
  }
  if(error_.couple().norm() > torqueThresh_)
  {
    Eigen::Vector3d rpy =
        (torqueGain_.first * error_.couple() + torqueGain_.second * errorD_.couple()).unaryExpr(clamper(0.01));
    rot = mc_rbdyn::rpyToMat(rpy);
  }
  const auto & sensor = task_->frame().forceSensor();
  const auto & X_p_f = sensor.X_p_f();
  const auto & X_0_p = task_->frame().robot().frame(sensor.parent()).position();
  sva::PTransformd move(rot, trans);
  const auto & X_f_ds = sensor.X_fsmodel_fsactual();
  return ((X_f_ds * X_p_f).inv() * move * (X_f_ds * X_p_f)) * X_0_p;
}

void ComplianceTask::update(mc_solver::QPSolver & solver)
{
  prevError_ = error_;
  /* Get wrench, remove gravity, use dof_ to deactivate some axis */
  error_ = task_->frame().wrench();
  error_ = sva::ForceVecd(dof_.cwiseProduct((error_ - obj_).vector()));
  if(firstUpdate_)
  {
    firstUpdate_ = false;
    prevError_ = error_;
  }
  errorD_ = (error_ - prevError_) / solver.dt();
  task_->target(computePose());
  /* Does nothing for now, but is here in case of changes */
  MetaTask::update(*task_, solver);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "compliance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      Eigen::Vector6d dof = Eigen::Vector6d::Ones();
      config("dof", dof);
      auto & robot = solver.robots().fromConfig(config, "ComplianceTask");
      std::string_view frame;
      if(config.has("body"))
      {
        mc_rtc::log::warning("Deprecate use of body while loading a ComplianceTask, use \"frame\" instead");
        frame = config("body");
      }
      else
      {
        frame = config("frame");
      }
      auto t = std::make_shared<mc_tasks::force::ComplianceTask>(robot.frame(frame), dof);
      if(config.has("stiffness"))
      {
        t->stiffness(config("stiffness"));
      }
      if(config.has("weight"))
      {
        t->weight(config("weight"));
      }
      if(config.has("forceThresh"))
      {
        mc_rtc::log::warning(
            "Deprecate use of forceThresh while loading a ComplianceTask, use \"forceThreshold\" instead");
        t->forceThreshold(config("forceThresh"));
      }
      if(config.has("forceThreshold"))
      {
        t->forceThreshold(config("forceThreshold"));
      }
      if(config.has("torqueThresh"))
      {
        mc_rtc::log::warning(
            "Deprecate use of torqueThresh while loading a ComplianceTask, use \"torqueThreshold\" instead");
        t->torqueThreshold(config("torqueThresh"));
      }
      if(config.has("torqueThreshold"))
      {
        t->torqueThreshold(config("torqueThreshold"));
      }
      if(config.has("forceGain"))
      {
        t->forceGain(config("forceGain"));
      }
      if(config.has("torqueGain"))
      {
        t->torqueGain(config("torqueGain"));
      }
      if(config.has("wrench"))
      {
        t->setTargetWrench(config("wrench"));
      }
      t->load(solver, config);
      return t;
    });
}
