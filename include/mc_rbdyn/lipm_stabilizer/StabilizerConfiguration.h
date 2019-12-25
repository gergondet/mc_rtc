/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rbdyn/api.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

/** Weights for force distribution quadratic program (FDQP).
 *
 */
struct MC_RBDYN_DLLAPI FDQPWeights
{
  double ankleTorqueSqrt;
  double netWrenchSqrt;
  double pressureSqrt;
};
} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
using FDQPWeights = mc_rbdyn::lipm_stabilizer::FDQPWeights;

/**
 * @brief Read force distribution QP weights from configuration.
 */
template<>
struct ConfigurationLoader<FDQPWeights>
{
  static FDQPWeights load(const mc_rtc::Configuration & config)
  {
    FDQPWeights weights;
    double ankleTorqueWeight = config("ankle_torque");
    double netWrenchWeight = config("net_wrench");
    double pressureWeight = config("pressure");
    weights.ankleTorqueSqrt = std::sqrt(ankleTorqueWeight);
    weights.netWrenchSqrt = std::sqrt(netWrenchWeight);
    weights.pressureSqrt = std::sqrt(pressureWeight);
    return weights;
  }

  static mc_rtc::Configuration save(const FDQPWeights & weights)
  {
    mc_rtc::Configuration config;
    config.add("ankle_torque", std::pow(weights.ankleTorqueSqrt, 2));
    config.add("net_wrench", std::pow(weights.netWrenchSqrt, 2));
    config.add("pressure", std::pow(weights.pressureSqrt, 2));
    return config;
  }
};
} // namespace mc_rtc

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

struct MC_RBDYN_DLLAPI StabilizerConfiguration
{
  FDQPWeights fdqpWeights;

  double friction = 0.7;
  std::string leftFootSurface;
  std::string rightFootSurface;

  Eigen::Vector2d copAdmittance = Eigen::Vector2d::Zero(); /**< Admittance gains for foot damping control */

  double dfzAdmittance = 1e-4; /**< Admittance for foot force difference control */
  double dfzDamping = 0.; /**< Damping term in foot force difference control */

  double dcmPropGain = 1.; /**< Proportional gain on DCM error */
  double dcmIntegralGain = 5.; /**< Integral gain on DCM error */
  double dcmDerivGain = 0.; /**< Derivative gain on DCM error */
  double dcmIntegratorTimeConstant = 15.;
  double dcmDerivatorTimeConstant = 1.;

  std::vector<std::string> comActiveJoints; /**< Joints used by CoM IK task */
  Eigen::Vector3d comStiffness = {1000., 1000., 100.}; /**< Stiffness of CoM IK task */
  double comWeight = 1000.; /**< Weight of CoM IK task */
  double comHeight = 0.84; /**< Desired height of the CoM */
  double maxCoMHeight = 0.92; /**< Maximum height of the CoM */
  double minCoMHeight = 0.6; /**< Minimum height of the CoM */

  std::string torsoBodyName;
  double torsoPitch = 0;
  double minTorsoPitch = 0;
  double maxTorsoPitch = 0;
  double torsoStiffness = 10;
  double torsoWeight = 100;
  double pelvisStiffness = 10;
  double pelvisWeight = 100;

  sva::MotionVecd contactDamping = sva::MotionVecd::Zero();
  sva::MotionVecd contactStiffness = sva::MotionVecd::Zero();
  double contactWeight = 100000.; /**< Weight of contact IK tasks */

  double vdcFrequency = 1.; /**< Frequency used in double-support vertical drift compensation */
  double vdcStiffness = 1000.; /**< Stiffness used in single-support vertical drift compensation */

  void load(const mc_rtc::Configuration & config)
  {
    config("fdqp_weights", fdqpWeights);

    config("leftFootSurface", leftFootSurface);
    config("rightFootSurface", rightFootSurface);
    config("friction", friction);
    config("torsoBodyName", torsoBodyName);

    if(config.has("admittance"))
    {
      auto admittance = config("admittance");
      admittance("cop", copAdmittance);
      admittance("dfz", dfzAdmittance);
      admittance("dfz_damping", dfzDamping);
    }
    if(config.has("dcm_tracking"))
    {
      auto dcmTracking = config("dcm_tracking");
      if(dcmTracking.has("gains"))
      {
        dcmTracking("gains")("prop", dcmPropGain);
        dcmTracking("gains")("integral", dcmIntegralGain);
        dcmTracking("gains")("deriv", dcmDerivGain);
      }
      dcmTracking("derivator_time_constant", dcmDerivatorTimeConstant);
      dcmTracking("integrator_time_constant", dcmIntegratorTimeConstant);
    }
    if(config.has("tasks"))
    {
      auto tasks = config("tasks");
      if(tasks.has("com"))
      {
        tasks("com")("active_joints", comActiveJoints);
        tasks("com")("stiffness", comStiffness);
        tasks("com")("weight", comWeight);
        tasks("com")("height", comHeight);
        tasks("com")("max_height", maxCoMHeight);
        tasks("com")("min_height", minCoMHeight);
      }

      if(tasks.has("torso"))
      {

        tasks("torso")("pitch", torsoPitch);
        tasks("torso")("min_pitch", minTorsoPitch);
        tasks("torso")("max_pitch", maxTorsoPitch);
        tasks("torso")("stiffness", torsoStiffness);
        tasks("torso")("weight", torsoWeight);
      }

      if(tasks.has("pelvis"))
      {
        tasks("pelvis")("stiffness", pelvisStiffness);
        tasks("pelvis")("weight", pelvisWeight);
      }

      if(tasks.has("contact"))
      {
        if(tasks("contact").has("damping"))
        {
          double d = tasks("contact")("damping");
          contactDamping = sva::MotionVecd({d, d, d}, {d, d, d});
        }
        if(tasks("contact").has("stiffness"))
        {
          double k = tasks("contact")("stiffness");
          contactStiffness = sva::MotionVecd({k, k, k}, {k, k, k});
        }
        tasks("contact")("stiffness", contactStiffness);
        tasks("contact")("weight", contactWeight);
      }
    }
    if(config.has("vdc"))
    {
      auto vdc = config("vdc");
      vdc("frequency", vdcFrequency);
      vdc("stiffness", vdcStiffness);
    }
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration conf;
    conf.add("fdqp_weights", fdqpWeights);

    conf.add("torsoBodyName", torsoBodyName);
    conf.add("leftFootSurface", leftFootSurface);
    conf.add("rightFootSurface", rightFootSurface);

    conf.add("admittance");
    conf("admittance")("cop", copAdmittance);
    conf("admittance")("dfz", dfzAdmittance);
    conf("admittance")("dfz_damping", dfzDamping);

    conf.add("dcm_tracking");
    conf("dcm_tracking").add("gains");
    conf("dcm_tracking")("gains").add("prop", dcmPropGain);
    conf("dcm_tracking")("gains").add("integral", dcmIntegralGain);
    conf("dcm_tracking")("gains").add("deriv", dcmDerivGain);
    conf("dcm_tracking").add("derivator_time_constant", dcmDerivatorTimeConstant);
    conf("dcm_tracking").add("integrator_time_constant", dcmIntegratorTimeConstant);

    conf.add("tasks");
    conf("tasks").add("com");
    conf("tasks")("com").add("active_joints", comActiveJoints);
    conf("tasks")("com").add("stiffness", comStiffness);
    conf("tasks")("com").add("weight", comWeight);
    conf("tasks")("com").add("height", comHeight);
    conf("tasks")("com").add("max_height", maxCoMHeight);
    conf("tasks")("com").add("min_height", minCoMHeight);

    conf("tasks").add("torso");
    conf("tasks")("torso").add("pitch", torsoPitch);
    conf("tasks")("torso").add("min_pitch", minTorsoPitch);
    conf("tasks")("torso").add("max_pitch", maxTorsoPitch);
    conf("tasks")("torso").add("stiffness", torsoStiffness);
    conf("tasks")("torso").add("weight", torsoWeight);

    conf("tasks").add("pelvis");
    conf("tasks")("pelvis").add("stiffness", pelvisStiffness);
    conf("tasks")("pelvis").add("weight", pelvisWeight);

    conf("tasks").add("contact");
    conf("tasks")("contact").add("damping", contactDamping);
    conf("tasks")("contact").add("stiffness", contactStiffness);
    conf("tasks")("contact").add("weight", contactWeight);

    conf.add("vdc");
    conf("vdc").add("frequency", vdcFrequency);
    conf("vdc").add("stiffness", vdcStiffness);
    return conf;
  }
};
} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
using StabilizerConfiguration = mc_rbdyn::lipm_stabilizer::StabilizerConfiguration;

template<>
struct ConfigurationLoader<StabilizerConfiguration>
{
  static StabilizerConfiguration load(const mc_rtc::Configuration & config)
  {
    StabilizerConfiguration stabi;
    stabi.load(config);
    return stabi;
  }

  static mc_rtc::Configuration save(const StabilizerConfiguration & stabiConf)
  {
    return stabiConf.save();
  }
};
} // namespace mc_rtc
