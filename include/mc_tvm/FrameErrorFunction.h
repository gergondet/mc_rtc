/* Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <RBDyn/Jacobian.h>

namespace mc_tvm
{

/** Represents the error between two frame positions.
 *
 * Given two frames (f1, f2) belonging to (r1, r2) this is the difference
 * between their current relative position.
 * This is a function of r1.q and r2.q.
 *
 * If r1 or r2 is not actuated, then it is not taken into account in
 * computations. If neither are actuated this is a constant function.
 *
 * The degrees of freedom computed by this function can be specified through a
 * 6x6 dof matrix.
 *
 * Outputs:
 *
 * - Value: dof*transformError(X_f1, X_f2)
 * - Velocity
 * - NormalAcceleration
 * - Jacobian
 *
 */
class MC_TVM_DLLAPI FrameErrorFunction : public tvm::function::abstract::Function
{
public:
  using Output = tvm::function::abstract::Function::Output;
  DISABLE_OUTPUTS(Output::JDot)
  SET_UPDATES(FrameErrorFunction, Value, Jacobian, Velocity, NormalAcceleration)

  /** Constructor
   *
   * \param f1 First contact frame
   *
   * \param f2 Second contact frame
   *
   * \param dof Contact dof
   *
   */
  FrameErrorFunction(mc_rbdyn::FramePtr f1,
                     mc_rbdyn::FramePtr f2,
                     const Eigen::Vector6d & dof = Eigen::Vector6d::Ones());

  /** Access the contact dof vector */
  inline const Eigen::Vector6d & dof() const noexcept
  {
    return dof_;
  }

  /** Set the contact dof vector */
  inline void dof(const Eigen::Vector6d & dof) noexcept
  {
    dof_ = dof;
  }

private:
  enum Dof
  {
    X = 1,
    Y = X * 2,
    Z = Y * 2,
    ___ = 0,
    X__ = X,
    _Y_ = Y,
    __Z = Z,
    XY_ = X + Y,
    X_Z = X + Z,
    _YZ = Y + Z,
    XYZ = X + Y + Z,
  };

  /** Helper structure to perform out = in (\p add = \c false) or out += in (\p add = \c true) for only the dofs given
   * by \p dof.*/
  template<bool add>
  struct assign
  {
    template<typename Derived>
    static void run(Eigen::Ref<Eigen::MatrixXd> out,
                    const Eigen::MatrixBase<Derived> & in,
                    mc_tvm::FrameErrorFunction::Dof dof,
                    Eigen::MatrixXd & buffer);
  };

  mc_rbdyn::FramePtr f1_;
  mc_rbdyn::FramePtr f2_;
  Eigen::Vector6d dof_; // specification of the dof used in the output
  Dof tDof_; // dofs used in the translation part of the error
  Dof rDof_; // dofs used in the rotation part of the error
  int nt_; // number of dofs used in the translation part of the error
  int nr_; // number of dofs used in the rotation part of the error

  bool use_f1_ = false;
  rbd::Jacobian f1Jacobian_;
  bool use_f2_ = false;
  rbd::Jacobian f2Jacobian_;
  bool sameVariable_ = false;

  Eigen::Vector3d rotErr_; // intermediate buffer for keeping the 3d rotation error
  Eigen::Vector3d rotVel_; // intermediate buffer for keeping the 3d rotation error velocity
  Eigen::Matrix3d dLog_;
  Eigen::Matrix3d d2Log_;
  Eigen::MatrixXd tmpMat_; // buffer to be used by assign::run

  void updateValue();
  void updateJacobian();
  void updateVelocity();
  void updateNormalAcceleration();
};

using ContactFunctionPtr = std::shared_ptr<mc_tvm::FrameErrorFunction>;

} // namespace mc_tvm
