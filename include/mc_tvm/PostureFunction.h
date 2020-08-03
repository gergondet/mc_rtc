/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

namespace mc_tvm
{

/** This class implements a posture function for a given robot */
class MC_TVM_DLLAPI PostureFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(PostureFunction, Value, Velocity)

  /** Constructor
   *
   * Set the objective to the current posture of robot
   *
   */
  PostureFunction(const mc_rbdyn::Robot & robot);

  /** Set the target posture to the current robot's posture */
  void reset();

  /** Set the target for a given joint
   *
   *  \param j Joint name
   *
   *  \param q Target configuration
   *
   */
  void posture(const std::string & j, const std::vector<double> & q);

  /** Set the fully body posture */
  void posture(const std::vector<std::vector<double>> & p);

protected:
  void updateValue();

  void updateVelocity();

  mc_rbdyn::ConstRobotPtr robot_;

  /** Target */
  std::vector<std::vector<double>> posture_;

  /** Starting joint */
  int j0_;
};

} // namespace mc_tvm
