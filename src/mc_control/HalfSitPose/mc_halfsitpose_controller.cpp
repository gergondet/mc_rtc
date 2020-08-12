/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_halfsitpose_controller.h"

#include <mc_rbdyn/RobotModule.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

/* Common stuff */
MCHalfSitPoseController::MCHalfSitPoseController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt), halfSitPose(robot().mbc().q)
{

  /* Set the halfSitPose in posture Task */
  const auto & halfSit = robot_module->stance();
  const auto & ref_joint_order = robot_module->ref_joint_order();
  for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
  {
    if(robot().hasJoint(ref_joint_order[i]))
    {
      halfSitPose[robot().jointIndexByName(ref_joint_order[i])] = halfSit.at(ref_joint_order[i]);
    }
  }

  /* Get the complete collision constraint set */
  collisionConstraint_->addCollisions(solver(), {robot().name(), robot_module->commonSelfCollisions()});
  solver().addConstraint(collisionConstraint_);

  solver().addConstraint(kinematicsConstraint_);

  postureTask_->weight(100.0);
  postureTask_->stiffness(2.0);
  solver().addTask(postureTask_);
  mc_rtc::log::success("Half-sitting controller initialized");
}

void MCHalfSitPoseController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  postureTask_->reset();
  gui_->removeElement({}, "Go half-sitting");
  gui_->addElement({}, mc_rtc::gui::Button("Go half-sitting", [this]() { postureTask_->posture(halfSitPose); }));
}

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("HalfSitPose", mc_control::MCHalfSitPoseController)
