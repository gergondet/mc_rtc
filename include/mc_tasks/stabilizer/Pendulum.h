/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_tasks/stabilizer/Contact.h>

namespace mc_tasks
{
namespace stabilizer
{
/** Inverted pendulum model.
 *
 */
struct Pendulum
{
  /** Initialize state.
   *
   * \param gravity Gravity vector
   *
   * \param com Initial CoM position.
   *
   * \param comd Initial CoM velocity.
   *
   * \param comdd Initial CoM acceleration.
   *
   */
  Pendulum(const Eigen::Vector3d & gravity,
           const Eigen::Vector3d & com = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  /** Complete IPM inputs (ZMP and omega) from CoM and contact plane.
   *
   * \param plane Contact plane.
   *
   */
  void completeIPM(const Contact & plane);

  /** Integrate constant CoM jerk for a given duration.
   *
   * \param comddd CoM jerk.
   *
   * \param dt Integration step.
   *
   */
  void integrateCoMJerk(const Eigen::Vector3d & comddd, double dt);

  /** Integrate in floating-base inverted pendulum mode with constant inputs.
   *
   * \param zmp Zero-tilting Moment Point, i.e. net force application point.
   *
   * \param lambda Normalized stiffness of the pendulum.
j
   * \param dt Duration of integration step.
   *
   */
  void integrateIPM(Eigen::Vector3d zmp, double lambda, double dt);

  /** Reset to a new state.
   *
   * \param com New CoM position.
   *
   * \param comd New CoM velocity.
   *
   * \param comdd Initial CoM acceleration.
   *
   */
  void reset(const Eigen::Vector3d & com,
             const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  /** Reset CoM height above a given contact plane.
   *
   * \param height CoM height above contact plane.
   *
   * \param contact Contact plane.
   *
   */
  void resetCoMHeight(double height, const Contact & contact);

  /** Get CoM position of the inverted pendulum model.
   *
   */
  const Eigen::Vector3d & com() const
  {
    return com_;
  }

  /** Get CoM velocity of the inverted pendulum model.
   *
   */
  const Eigen::Vector3d & comd() const
  {
    return comd_;
  }

  /** Get CoM acceleration of the inverted pendulum.
   *
   */
  const Eigen::Vector3d & comdd() const
  {
    return comdd_;
  }

  /** Instantaneous Divergent Component of Motion.
   *
   */
  Eigen::Vector3d dcm() const
  {
    return com_ + comd_ / omega_;
  }

  /** Natural frequency of last IPM integration.
   *
   */
  double omega() const
  {
    return omega_;
  }

  /** Zero-tilting moment point from last integration.
   *
   * \note In the linear inverted pendulum mode, the ZMP coincides with the
   * centroidal moment pivot (CMP) or its extended version (eCMP).
   *
   */
  const Eigen::Vector3d & zmp() const
  {
    return zmp_;
  }

  /** Velocity of the zero-tilting moment point.
   *
   */
  const Eigen::Vector3d & zmpd() const
  {
    return zmpd_;
  }

protected:
  Eigen::Vector3d gravity_; /**< Gravity vector */
  Eigen::Vector3d com_; /**< Position of the center of mass */
  Eigen::Vector3d comd_; /**< Velocity of the center of mass */
  Eigen::Vector3d comdd_; /**< Acceleration of the center of mass */
  Eigen::Vector3d comddd_; /**< Jerk of the center of mass */
  Eigen::Vector3d zmp_; /**< Position of the zero-tilting moment point */
  Eigen::Vector3d zmpd_; /**< Velocity of the zero-tilting moment point */
  double omega_; /**< Natural frequency of the linear inverted pendulum */
};

} // namespace stabilizer
} // namespace mc_tasks
