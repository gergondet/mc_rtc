/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCPostureController : public MCController
{
public:
  MCPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const ControllerResetData & reset_data) override;
};

} // namespace mc_control
