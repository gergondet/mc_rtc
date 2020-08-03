/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <RBDyn/Jacobian.h>

namespace mc_tvm
{

/** Represents a geometric contact function
 *
 * Given two frames (f1, f2) belonging to (r1, r2) this is the difference
 * between their current relative position and their initial relative position.
 * This is a function of r1.q and r2.q.
 *
 * If r1 or r2 is not actuated, then it is not taken into account in
 * computations. If neither are actuated this does nothing.
 *
 * The degrees of freedom computed by this function can be specified through a
 * 6x6 dof matrix.
 *
 * Outputs:
 *
 * - Value: dof*transformError(X_f1_f2, X_f1_f2_init)
 * - Velocity
 * - NormalAcceleration
 * - Jacobian
 *
 */
class TVM_DLLAPI ContactFunction : public tvm::function::abstract::Function
{
public:
  using Output = tvm::function::abstract::Function::Output;
  DISABLE_OUTPUTS(Output::JDot)
  SET_UPDATES(ContactFunction, Value, Derivatives)

  /** Constructor
   *
   * \param contact The Contact that this function computes
   *
   * \param dof The function dof matrix
   *
   */
  ContactFunction(mc_rbdyn::ConstFramePtr f1, mc_rbdyn::ConstFramePtr f2);

private:
  mc_rbdyn::ConstFramePtr f1_;
  mc_rbdyn::ConstFramePtr f2_;

  bool use_f1_ = false;
  rbd::Jacobian f1Jacobian_;
  bool use_f2_ = false;
  rbd::Jacobian f2Jacobian_;
  sva::PTransformd X_f1_f2_init_;

  Eigen::MatrixXd jacTmp_;
  Eigen::MatrixXd jac_;

  void updateValue();
  // Derivatives update involve many intermediary that are used for all computations
  void updateDerivatives();
};

} // namespace mc_tvm
