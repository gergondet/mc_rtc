/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>

#include <mc_rtc/deprecated.h>

namespace mc_rbdyn
{

// Repeat static constexpr declarations
// See also https://stackoverflow.com/q/8016780
constexpr double RobotModule::Gripper::Safety::DEFAULT_PERCENT_VMAX;
constexpr double RobotModule::Gripper::Safety::DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER;
constexpr double RobotModule::Gripper::Safety::DEFAULT_RELEASE_OFFSET;
constexpr unsigned int RobotModule::Gripper::Safety::DEFAULT_OVER_COMMAND_LIMIT_ITER_N;

DevicePtrVector::DevicePtrVector(const DevicePtrVector & v) : std::vector<DevicePtr>()
{
  reserve(v.size());
  for(const auto & s : v)
  {
    push_back(s->clone());
  }
}

DevicePtrVector & DevicePtrVector::operator=(const DevicePtrVector & v)
{
  if(&v == this)
  {
    return *this;
  }
  resize(v.size());
  for(size_t i = 0; i < v.size(); ++i)
  {
    (*this)[i] = v[i]->clone();
  }
  return *this;
}

RobotModule::RobotModule(std::string_view name, const rbd::parsers::ParserResult & res)
: RobotModule("/CREATED/BY/MC/RTC/", name)
{
  rsdf_dir = "";
  init(res);
}

void RobotModule::init(const rbd::parsers::ParserResult & res)
{
  mb = res.mb;
  mbc = res.mbc;
  mbg = res.mbg;
  for(const auto & col : res.collision)
  {
    const auto & body = col.first;
    const auto & cols = col.second;
    if(cols.size())
    {
      _collisionTransforms[body] = cols[0].origin;
    }
  }
  boundsFromURDF(res.limits);
  for(const auto & v : res.visual)
  {
    _visual.emplace(v.first, v.second);
  }
  for(const auto & c : res.collision)
  {
    _collision.emplace(c.first, c.second);
  }
  make_default_ref_joint_order();
  expand_stance();
}

RobotModule::Gripper::Gripper(const std::string & name, const std::vector<std::string> & joints, bool reverse_limits)
: Gripper(name, joints, reverse_limits, nullptr, nullptr)
{
}

RobotModule::Gripper::Gripper(const std::string & name,
                              const std::vector<std::string> & joints,
                              bool reverse_limits,
                              const Safety & safety)
: Gripper(name, joints, reverse_limits, &safety, nullptr)
{
}

RobotModule::Gripper::Gripper(const std::string & name,
                              const std::vector<std::string> & joints,
                              bool reverse_limits,
                              const Safety & safety,
                              const std::vector<Mimic> & mimics)
: Gripper(name, joints, reverse_limits, &safety, &mimics)
{
}

RobotModule::Gripper::Gripper(const std::string & name,
                              const std::vector<std::string> & joints,
                              bool reverse_limits,
                              const Safety * safety,
                              const std::vector<Mimic> * mimics)
: name(name), joints(joints), reverse_limits(reverse_limits), hasSafety_(safety != nullptr),
  hasMimics_(mimics != nullptr)
{
  if(mimics)
  {
    mimics_ = *mimics;
  }
  if(safety)
  {
    safety_ = *safety;
  }
}

void RobotModule::Gripper::Safety::load(const mc_rtc::Configuration & config)
{
  if(config.has("actualCommandDiffTrigger"))
  {
    actualCommandDiffTrigger = config("actualCommandDiffTrigger");
  }
  if(config.has("threshold"))
  {
    mc_rtc::log::deprecated(
        "Gripper::Safety", "threshold", "actualCommandDiffTrigger",
        "Note: threshold was expressed in degrees while actualCommandDiffTrigger is expressed in radians");
    actualCommandDiffTrigger = mc_rtc::constants::toRad(config("threshold"));
  }

  if(config.has("overCommandLimitIterN"))
  {
    overCommandLimitIterN =
        std::max<unsigned int>(1, config("overCommandLimitIterN", DEFAULT_OVER_COMMAND_LIMIT_ITER_N));
  }
  else if(config.has("iter"))
  {
    mc_rtc::log::deprecated("Gripper::Safety", "iter", "overCommandLimitIterN");
    overCommandLimitIterN = std::max<unsigned int>(1, config("iter", DEFAULT_OVER_COMMAND_LIMIT_ITER_N));
  }

  if(config.has("releaseSafetyOffset"))
  {
    releaseSafetyOffset = config("releaseSafetyOffset");
  }
  else if(config.has("release"))
  {
    mc_rtc::log::deprecated("Gripper::Safety", "release", "releaseSafetyOffset",
                            "Note: release was expressed in degrees while releaseSafetyOffset is expressed in radians");
    releaseSafetyOffset = mc_rtc::constants::toRad(config("release"));
  }

  if(config.has("percentVMax"))
  {
    percentVMax = mc_filter::utils::clamp(static_cast<double>(config("percentVMax")), 0, 1);
  }
  else if(config.has("percentVMAX"))
  {
    mc_rtc::log::deprecated("Gripper::Safety", "percentVMAX", "percentVmax");
    percentVMax = mc_filter::utils::clamp(static_cast<double>(config("percentVMAX")), 0, 1);
  }
}

mc_rtc::Configuration RobotModule::Gripper::Safety::save() const
{
  mc_rtc::Configuration config;
  config.add("actualCommandDiffTrigger", actualCommandDiffTrigger);
  config.add("overCommandLimitIterN", overCommandLimitIterN);
  config.add("releaseSafetyOffset", releaseSafetyOffset);
  config.add("percentVMax", percentVMax);
  return config;
}

void RobotModule::boundsFromURDF(const rbd::parsers::Limits & limits)
{
  _bounds = urdf_limits_to_bounds(limits);
}

void RobotModule::expand_stance()
{
  for(const auto & j : mb.joints())
  {
    if(!_stance.count(j.name()))
    {
      _stance[j.name()] = j.zeroParam();
    }
  }
}

void RobotModule::make_default_ref_joint_order()
{
  _ref_joint_order.resize(0);
  for(const auto & j : mb.joints())
  {
    if(j.dof() >= 1 && j.type() != rbd::Joint::Free)
    {
      _ref_joint_order.push_back(j.name());
    }
  }
}

RobotModule::bounds_t urdf_limits_to_bounds(const rbd::parsers::Limits & limits)
{
  auto transform = [](const std::map<std::string, std::vector<double>> & v, bool neg = false) {
    RobotModule::bound_t res;
    res.reserve(v.size());
    for(const auto & vi : v)
    {
      res.emplace(vi.first, vi.second);
      if(neg)
      {
        for(auto & vj : res[vi.first])
        {
          vj = -vj;
        }
      }
    }
    return res;
  };
  return {transform(limits.lower),    transform(limits.upper),        transform(limits.velocity, true),
          transform(limits.velocity), transform(limits.torque, true), transform(limits.torque)};
}

} // namespace mc_rbdyn
