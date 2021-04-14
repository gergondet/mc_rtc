/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>
#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCPythonController : public MCController
{
public:
  MCPythonController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots, double dt);

  virtual void reset(const ControllerResetData & reset_data) override;

  virtual bool run() override;

  std::function<bool()> run_callback;
  std::function<void(const ControllerResetData &)> reset_callback;

  // Expose some protected members

  inline mc_solver::DynamicsConstraintPtr dynamicsConstraint() noexcept
  {
    return dynamicsConstraint_;
  }
  inline mc_solver::KinematicsConstraintPtr kinematicsConstraint() noexcept
  {
    return kinematicsConstraint_;
  }
  inline mc_solver::CollisionsConstraintPtr collisionConstraint() noexcept
  {
    return collisionConstraint_;
  }
  inline mc_tasks::PostureTaskPtr postureTask() noexcept
  {
    return postureTask_;
  }
};

} // namespace mc_control
