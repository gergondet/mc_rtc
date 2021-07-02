/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_posture_controller.h"

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/PositionTask.h>

namespace mc_control
{

/* Common stuff */
MCPostureController::MCPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module,
                                         double dt,
                                         const mc_rtc::Configuration & config)
: MCController(robot_module, dt, config)
{
  solver().addConstraint(kinematicsConstraint_);
  solver().addConstraint(collisionConstraint_);
  solver().addConstraint(compoundJointConstraint_);

  postureTask_->stiffness(5.0);
  postureTask_->weight(1.0);
  solver().addTask(postureTask_);

  for(auto & r_ptr : robots())
  {
    auto & r = *r_ptr;
    if(r.name() != robot().name() && r.mb().nrDof())
    {
      auto pt = std::make_shared<mc_tasks::PostureTask>(r);
      pt->stiffness(5.0);
      pt->weight(1.0);
      solver().addTask(pt);
      solver().addConstraint(std::make_shared<mc_solver::KinematicsConstraint>(r));
      solver().addConstraint(std::make_shared<mc_solver::CompoundJointConstraint>(r, solver().dt()));
    }
  }

  mc_rtc::log::success("Posture sample controller initialized");
}

bool MCPostureController::run()
{
  return mc_control::MCController::run();
}

void MCPostureController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  for(auto & r_ptr : robots())
  {
    auto & robot = *r_ptr;
    if(robot.hasSurface("LeftFoot"))
    {
      auto comT = std::make_shared<mc_tasks::CoMTask>(robot);
      comT->weight(1000.0);
      solver().addTask(comT);
      solver().addContact({robot.name(), "ground", "LeftFoot", "AllGround"});
      solver().addContact({robot.name(), "ground", "RightFoot", "AllGround"});
    }
  }
}

} // namespace mc_control

CONTROLLER_CONSTRUCTOR("Posture", mc_control::MCPostureController)
