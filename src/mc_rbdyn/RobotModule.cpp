/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>

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

RobotModule::RobotModule(const std::string & name, const rbd::parsers::ParserResult & res)
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
  _visual = res.visual;
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
  RobotModule::bounds_t ret = {};
  ret.reserve(8);
  ret.push_back(limits.lower);
  ret.push_back(limits.upper);
  auto convert = [](const std::map<std::string, std::vector<double>> & l) {
    auto ret = l;
    for(auto & el : ret)
    {
      for(auto & e : el.second)
      {
        e = -e;
      }
    }
    return ret;
  };
  auto inf_convert = [](const std::map<std::string, std::vector<double>> & l, bool neg) {
    auto ret = l;
    auto inf = std::numeric_limits<double>::infinity();
    auto value = neg ? -inf : inf;
    for(auto & el : ret)
    {
      for(auto & e : el.second)
      {
        e = value;
      }
    }
    return ret;
  };
  ret.push_back(convert(limits.velocity));
  ret.push_back(limits.velocity);
  ret.push_back(convert(limits.torque));
  ret.push_back(limits.torque);
  ret.push_back(inf_convert(limits.torque, true));
  ret.push_back(inf_convert(limits.torque, false));
  return ret;
}

} // namespace mc_rbdyn
