/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/LinearFunction.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_tvm
{

/** Implement the equation of motion for a given robot.
 *
 * It can be given contacts that will be integrated into the equation of
 * motion (\see DynamicFunction::addContact).
 *
 * It manages the force variables related to these contacts.
 *
 * It also takes care of linearizing the contacts, either by having them in a
 * linearized form in the solver (lambdas) or as forces. The related
 * constraint (mu*f_n >= ||f_t||) can be added to the problem through this
 * function.
 *
 * Notably, it does not take care of enforcing Newton 3rd law of motion when
 * two actuated robots are in contact.
 *
 */
struct MC_TVM_DLLAPI DynamicFunction : public tvm::function::abstract::LinearFunction
{
public:
  using Output = tvm::function::abstract::LinearFunction::Output;
  DISABLE_OUTPUTS(Output::JDot)
  SET_UPDATES(DynamicFunction, Jacobian, B)

  /** Construct the equation of motion for a given robot */
  DynamicFunction(mc_rbdyn::RobotPtr robot);

  /** Add a contact to the function
   *
   * This adds forces variables for every contact point belonging to the
   * robot of this dynamic function.
   *
   * \param frame Contact frame
   *
   * \param points Contact points in the contact frame
   *
   * \param dir Contact direction
   *
   * Returns the force variables that were created by this contact
   */
  const tvm::VariableVector & addContact(const mc_rbdyn::Frame & frame,
                                         std::vector<sva::PTransformd> points,
                                         double dir);

  /** Removes the contact associated to the given frame
   *
   * \param frame Contact frame
   */
  void removeContact(const mc_rbdyn::Frame & frame);

  /** Returns the contact force at the given contact frame
   *
   * \param f Contact frame
   *
   * \throws If no contact has been added with that frame
   */
  sva::ForceVecd contactForce(const mc_rbdyn::Frame & f) const;

protected:
  void updateb();

  mc_rbdyn::RobotPtr robot_;

  /** Holds data for the force part of the motion equation */
  struct ForceContact
  {
    /** Constructor */
    ForceContact(const mc_rbdyn::Frame & frame, std::vector<sva::PTransformd> points, double dir);

    /** Update jacobians */
    void updateJacobians(DynamicFunction & parent);

    /** Compute the contact force */
    sva::ForceVecd force() const;

    /** Associated frame */
    mc_rbdyn::ConstFramePtr frame_;

    /** Force associated to a contact */
    tvm::VariableVector forces_;

    /** Contact points */
    std::vector<sva::PTransformd> points_;

    /** Contact direction */
    double dir_;

    /** Used for intermediate Jacobian computation */
    Eigen::MatrixXd force_jac_;
    Eigen::MatrixXd full_jac_;
  };
  std::vector<ForceContact> contacts_;

  std::vector<ForceContact>::const_iterator findContact(const mc_rbdyn::Frame & frame) const;

  void updateJacobian();
};

} // namespace mc_tvm
