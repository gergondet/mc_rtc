/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>
#include <mc_rbdyn/fwd.h>

#include <Eigen/Core>

namespace mc_rbdyn
{

/** Simple joint limits description extracted from a RobotModule */
struct MC_RBDYN_DLLAPI Limits
{
  /** Lower joint limits */
  Eigen::VectorXd ql;
  /** Upper joint limits */
  Eigen::VectorXd qu;
  /** Lower joint velocity limits */
  Eigen::VectorXd vl;
  /** Upper joint velocity limits */
  Eigen::VectorXd vu;
  /** Lower joint acceleration limits */
  Eigen::VectorXd al;
  /** Upper joint acceleration limits */
  Eigen::VectorXd au;
  /** Lower joint torque limits */
  Eigen::VectorXd tl;
  /** Upper joint torque limits */
  Eigen::VectorXd tu;
  /** Lower joint torque derivative limits */
  Eigen::VectorXd tdl;
  /** Upper joint torque derivative limits */
  Eigen::VectorXd tdu;
};

} // namespace mc_rbdyn
