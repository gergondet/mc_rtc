/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

namespace mc_tvm
{

/** This class implements joints' selection on a given fucntion */
class MC_TVM_DLLAPI JointsSelectorFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(JointsSelector, Jacobian, JDot)

  /** Construct a JointsSelector from a vector of active joints
   *
   * \param f Function selected by this JointsSelector
   *
   * \param robot Robot controlled by \p f
   *
   * \param activeJoints Joints active in this selector
   *
   * \throws If \p does not depend on \p robot or any joint in activeJoints
   * is not part of \p robot
   */
  static std::unique_ptr<JointsSelector> ActiveJoints(tvm::FunctionPtr f,
                                                      const mc_rbdyn::Robot & robot,
                                                      const std::vector<std::string> & activeJoints);

  /** Construct a JointsSelector from a vector of inactive joints
   *
   * \param f Function selected by this JointsSelector
   *
   * \param robot Robot controlled by \p f
   *
   * \param inactiveJoints Joints not active in this selector
   *
   * \throws If \p does not depend on \p robot or any joint in activeJoints
   * is not part of \p robot
   */
  static std::unique_ptr<JointsSelector> InactiveJoints(tvm::FunctionPtr f,
                                                        const mc_rbdyn::Robot & robot,
                                                        const std::vector<std::string> & inactiveJoints);

  inline const Eigen::VectorXd & value() const override
  {
    return f_->value();
  }

  inline const Eigen::VectorXd & velocity() const override
  {
    return f_->velocity();
  }

  inline const Eigen::VectorXd & normalAcceleration() const override
  {
    return f_->normalAcceleration();
  }

protected:
  /** Constructor
   *
   * \param f Function selected by this JointsSelector
   *
   * \param robot Robot controlled by \p f
   *
   * \param fbActive True if the selected joints include the floating-base
   *
   * \param activeIndex For regular joints selected by this JointsSelector,
   * this iss a list of (start, size) pair corresponding to the list of
   * joints
   *
   */
  JointsSelectorFunction(tvm::FunctionPtr f,
                         const mc_rbdyn::Robot & robot,
                         bool fbActive,
                         const std::vector<std::pair<Eigen::DenseIndex, Eigen::DenseIndex>> & activeIndex);

protected:
  void updateJacobian();
  void updateJDot();

  tvm::FunctionPtr f_;
  mc_rbdyn::ConstRobotPtr robot_;
  bool fbActive_;
  std::vector<std::pair<Eigen::DenseIndex, Eigen::DenseIndex>> activeIndex_;
};

} // namespace mc_tvm
