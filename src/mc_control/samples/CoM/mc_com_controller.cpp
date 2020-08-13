/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_com_controller.h"

#include <mc_filter/utils/clamp.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/SurfaceTransformTask.h>

namespace mc_control
{

MCCoMController::MCCoMController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    leftFootSurface_ = "LFullSole";
    rightFootSurface_ = "RFullSole";
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    leftFootSurface_ = "LeftFoot";
    rightFootSurface_ = "RightFoot";
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("MCCoMController does not support robot {}", robot().name());
  }

  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.frame(leftFootSurface_).position(), robot.frame(rightFootSurface_).position(), 0.5);
  });

  postureTask_->stiffness(1);
  postureTask_->weight(1);
  solver().addTask(postureTask_);

  mc_rtc::log::success("CoM sample controller initialized");
}

void MCCoMController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  comTask_ = std::make_shared<mc_tasks::CoMTask>(robot());
  comTask_->weight(1000);
  solver().addTask(comTask_);
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    solver().addContact({robot().name(), "ground", "LFullSole", "AllGround"});
    solver().addContact({robot().name(), "ground", "RFullSole", "AllGround"});
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    solver().addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    solver().addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("CoM sample controller does not support this robot {}",
                                                     robot().name());
  }
  solver().addConstraint(dynamicsConstraint_);
  solver().addConstraint(collisionConstraint_);
  // FIXME
  // solver().addConstraint(compoundJointConstraint_);
}

bool MCCoMController::run()
{
  updateAnchorFrame();
  return MCController::run();
}

} // namespace mc_control
