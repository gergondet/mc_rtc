/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/MCController.h>
#include <mc_observers/EncoderObserver.h>
#include <mc_observers/ObserverMacros.h>

namespace mc_observers
{

void EncoderObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  updateRobot_ = config("updateRobot", static_cast<std::string>(robot_));
  if(!ctl.robots().hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Observer {} requires robot \"{}\" but this robot does not exit",
                                                     name(), robot_);
  }
  if(!ctl.robots().hasRobot(updateRobot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Observer {} requires robot \"{}\" (updateRobot) but this robot does not exit", name(), updateRobot_);
  }
  const std::string & position = config("position", std::string("encoderValues"));
  if(position == "control")
  {
    posUpdate_ = PosUpdate::Control;
  }
  else if(position == "encoderValues")
  {
    posUpdate_ = PosUpdate::EncoderValues;
  }
  else if(position == "none")
  {
    posUpdate_ = PosUpdate::None;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[EncoderObserver::{}] Invalid configuration value \"{}\" for field \"position\" (valid values are [control, "
        "encoderValues, none])",
        name_, position);
  }

  const std::string & velocity = config("velocity", std::string("encoderFiniteDifferences"));
  if(velocity == "control")
  {
    velUpdate_ = VelUpdate::Control;
  }
  else if(velocity == "encoderFiniteDifferences")
  {
    velUpdate_ = VelUpdate::EncoderFiniteDifferences;
  }
  else if(velocity == "encoderVelocities")
  {
    velUpdate_ = VelUpdate::EncoderVelocities;
  }
  else if(velocity == "none")
  {
    velUpdate_ = VelUpdate::None;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[EncoderObserver::{}] Invalid configuration value \"{}\" for field \"velocity\" (valid values are [control, "
        "encoderFiniteDifferences, encoderVelocities, none])",
        name_, velocity);
    ;
  }

  config("computeFK", computeFK_);
  config("computeFV", computeFV_);

  if(config.has("log"))
  {
    auto lConfig = config("log");
    lConfig("position", logPosition_);
    lConfig("velocity", logVelocity_);
  }

  desc_ = name_ + " (position=" + position + ",velocity=" + velocity + ")";
}

void EncoderObserver::reset(const mc_control::MCController & ctl)
{
  auto & robot = ctl.robots().robot(robot_);
  const auto & enc = robot.encoderValues();
  if(enc.empty()
     && (posUpdate_ == PosUpdate::EncoderValues || velUpdate_ == VelUpdate::EncoderFiniteDifferences
         || velUpdate_ == VelUpdate::EncoderVelocities))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[EncoderObserver] requires robot {} to have encoder measurements",
                                                     robot_);
  }
  if(velUpdate_ == VelUpdate::EncoderVelocities && robot.encoderVelocities().empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[EncoderObserver] requires robot {} to have encoder velocity measurements", robot_);
  }

  if(!enc.empty())
  {
    prevEncoders_ = enc;
    encodersVelocity_.resize(enc.size());
    for(unsigned i = 0; i < enc.size(); ++i)
    {
      encodersVelocity_[i] = 0;
    }
  }
}

bool EncoderObserver::run(const mc_control::MCController & ctl)
{
  auto & robot = ctl.robots().robot(robot_);
  if(velUpdate_ == VelUpdate::EncoderFiniteDifferences)
  {
    const auto & enc = robot.encoderValues();
    for(unsigned i = 0; i < enc.size(); ++i)
    {
      encodersVelocity_[i] = (enc[i] - prevEncoders_[i]) / ctl.solver().dt();
      prevEncoders_[i] = enc[i];
    }
  }
  return true;
}

void EncoderObserver::update(mc_control::MCController & ctl)
{
  auto & realRobots = ctl.realRobots();
  const auto & robot = ctl.robots().robot(robot_);
  auto & realRobot = realRobots.robot(updateRobot_);
  const auto & q = robot.encoderValues();

  // Set all joint values and velocities from encoders
  size_t nJoints = robot.module().ref_joint_order().size();
  for(size_t i = 0; i < nJoints; ++i)
  {
    const auto q_index = robot.refJointIndexToQIndex(i);
    if(q_index == -1)
    {
      continue;
    }
    // Update position
    if(posUpdate_ == PosUpdate::Control)
    {
      realRobot.q()->set(q_index, robot.q()->value()(q_index));
    }
    else if(posUpdate_ == PosUpdate::EncoderValues)
    {
      realRobot.q()->set(q_index, q[i]);
    }

    // alpha_index is guaranteed to be valid is q_index is
    const auto alpha_index = robot.refJointIndexToQDotIndex(i);
    // Update velocity
    if(velUpdate_ == VelUpdate::Control)
    {
      realRobot.alpha()->set(alpha_index, robot.alpha()->value()(alpha_index));
    }
    else if(velUpdate_ == VelUpdate::EncoderFiniteDifferences)
    {
      realRobot.alpha()->set(alpha_index, encodersVelocity_[i]);
    }
    else if(velUpdate_ == VelUpdate::EncoderVelocities)
    {
      realRobot.alpha()->set(alpha_index, robot.encoderVelocities()[i]);
    }
  }
  if(computeFK_ && posUpdate_ != PosUpdate::None)
  {
    realRobot.forwardKinematics();
  }
  if(computeFV_ && velUpdate_ != VelUpdate::None)
  {
    realRobot.forwardVelocity();
  }
}

void EncoderObserver::addToLogger(const mc_control::MCController & ctl,
                                  mc_rtc::Logger & logger,
                                  const std::string & category)
{
  if(logPosition_)
  {
    if(posUpdate_ == PosUpdate::EncoderValues)
    {
      logger.addLogEntry(category + "_encoderValues", this,
                         [this, &ctl]() -> const std::vector<double> & { return ctl.robot(robot_).encoderValues(); });
    }
    else if(posUpdate_ == PosUpdate::Control)
    {
      std::vector<double> qOut(ctl.robot(robot_).module().ref_joint_order().size(), 0);
      logger.addLogEntry(category + "_controlValues", [this, &ctl, qOut]() mutable -> const std::vector<double> & {
        const auto & robot = ctl.robot(robot_);
        const auto & q = robot.q()->value();
        for(size_t i = 0; i < qOut.size(); ++i)
        {
          auto jIdx = robot.refJointIndexToQIndex(i);
          if(jIdx != -1)
          {
            qOut[i] = q(jIdx);
          }
        }
        return qOut;
      });
    }
  }

  if(logVelocity_)
  {
    if(velUpdate_ == VelUpdate::EncoderFiniteDifferences)
    {
      MC_RTC_LOG_HELPER(category + "_encoderFiniteDifferences", encodersVelocity_);
    }
    else if(velUpdate_ == VelUpdate::EncoderVelocities)
    {
      logger.addLogEntry(category + "_encoderVelocities", this, [this, &ctl]() -> const std::vector<double> & {
        return ctl.robot(robot_).encoderVelocities();
      });
    }
    else if(velUpdate_ == VelUpdate::Control)
    {
      std::vector<double> alphaOut(ctl.robot(robot_).module().ref_joint_order().size(), 0);
      logger.addLogEntry(category + "_controlVelocities",
                         [this, &ctl, alphaOut]() mutable -> const std::vector<double> & {
                           const auto & robot = ctl.robot(robot_);
                           const auto & alpha = robot.alpha()->value();
                           for(size_t i = 0; i < alphaOut.size(); ++i)
                           {
                             auto jIdx = robot.refJointIndexToQDotIndex(i);
                             if(jIdx != -1)
                             {
                               alphaOut[i] = alpha(jIdx);
                             }
                           }
                           return alphaOut;
                         });
    }
  }
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("Encoder", mc_observers::EncoderObserver)
