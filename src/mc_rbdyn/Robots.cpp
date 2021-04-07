/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>

#include <mc_rtc/deprecated.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/pragma.h>

#include <RBDyn/FK.h>
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_rbdyn
{

MC_RTC_diagnostic_push;
MC_RTC_diagnostic_ignored(GCC, "-Wsign-conversion", ClangOnly, "-Wshorten-64-to-32");

Robots::Robots() : robots_() {}

Robots::Robots(const Robots & rhs)
{
  for(const auto & robot : rhs.robots_)
  {
    this->robotCopy(*robot, robot->name());
  }
}

Robots & Robots::operator=(const Robots & rhs)
{
  if(&rhs == this)
  {
    return *this;
  }
  robots_.clear();
  const_robots_.clear();
  for(const auto & robot : rhs.robots_)
  {
    this->robotCopy(*robot, robot->name());
  }
  return *this;
}

std::vector<RobotPtr> & Robots::robots()
{
  return robots_;
}
const std::vector<ConstRobotPtr> & Robots::robots() const
{
  return const_robots_;
}

Robots::const_iterator Robots::getRobot(std::string_view name) const
{
  return std::find_if(const_robots_.begin(), const_robots_.end(),
                      [&name](const ConstRobotPtr & r) { return r->name() == name; });
}

bool Robots::hasRobot(std::string_view name) const
{
  auto it = getRobot(name);
  return it != const_robots_.end();
}

Robot & Robots::robot(std::string_view name)
{
  return const_cast<Robot &>(static_cast<const Robots *>(this)->robot(name));
}

const Robot & Robots::robot(std::string_view name) const
{
  auto it = getRobot(name);
  if(it != const_robots_.end())
  {
    return *it->get();
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No robot named {}", name);
  }
}

void Robots::removeRobot(std::string_view name)
{
  for(size_t i = 0; i < const_robots_.size(); ++i)
  {
    if(const_robots_[i]->name() == name)
    {
      robots_.erase(robots_.begin() + i);
      const_robots_.erase(const_robots_.begin() + i);
      return;
    }
  }
}

Robot & Robots::addRobot(RobotPtr robot)
{
  robots_.push_back(robot);
  const_robots_.push_back(robot);
  return *robot;
}

Robot & Robots::robotCopy(const Robot & robot, std::string_view name)
{
  return addRobot(robot.copy(name));
}

Robot & Robots::load(const RobotModule & module,
                     std::string_view name,
                     const std::optional<sva::PTransformd> & base,
                     const std::optional<std::string_view> & bName)
{
  if(hasRobot(name))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Cannot load a robot named {}, a robot with this name already exists", name);
  }
  return addRobot(std::allocate_shared<Robot>(Eigen::aligned_allocator_indirection<Robot>{}, Robot::make_shared_token{},
                                              module, name, true, base, bName));
}

Robot & Robots::loadFromUrdf(std::string_view name,
                             const std::string & urdf,
                             bool withVirtualLinks,
                             const std::vector<std::string> & filteredLinks,
                             bool fixed,
                             const std::optional<sva::PTransformd> & base,
                             const std::optional<std::string_view> & bName)
{
  auto res = rbd::parsers::from_urdf(urdf, fixed, filteredLinks, true, "", withVirtualLinks);

  mc_rbdyn::RobotModule module(name, res);

  return load(module, name, base, bName);
}

Robots::iterator Robots::begin() noexcept
{
  return robots_.begin();
}

Robots::const_iterator Robots::begin() const noexcept
{
  return const_robots_.begin();
}

Robots::const_iterator Robots::cbegin() const noexcept
{
  return const_robots_.cbegin();
}

Robots::iterator Robots::end() noexcept
{
  return robots_.end();
}

Robots::const_iterator Robots::end() const noexcept
{
  return const_robots_.end();
}

Robots::const_iterator Robots::cend() const noexcept
{
  return const_robots_.cend();
}

Robots::reverse_iterator Robots::rbegin() noexcept
{
  return robots_.rbegin();
}

Robots::const_reverse_iterator Robots::rbegin() const noexcept
{
  return const_robots_.rbegin();
}

Robots::const_reverse_iterator Robots::crbegin() const noexcept
{
  return const_robots_.crbegin();
}

Robots::reverse_iterator Robots::rend() noexcept
{
  return robots_.rend();
}

Robots::const_reverse_iterator Robots::rend() const noexcept
{
  return const_robots_.rend();
}

Robots::const_reverse_iterator Robots::crend() const noexcept
{
  return const_robots_.crend();
}

void mc_rbdyn::Robots::reserve(mc_rbdyn::Robots::size_type new_cap)
{
  robots_.reserve(new_cap);
  const_robots_.reserve(new_cap);
}

mc_rbdyn::Robots::size_type mc_rbdyn::Robots::size() const noexcept
{
  return robots_.size();
}

mc_rbdyn::Robot & Robots::fromConfig(const mc_rtc::Configuration & config,
                                     const std::string & prefix,
                                     bool required,
                                     const std::string & robotIndexKey,
                                     const std::string & robotNameKey,
                                     const std::string & defaultRobotName)

{
  std::string p = prefix.size() ? "[" + prefix + "]" : "";
  if(config.has(robotNameKey))
  {
    std::string_view name = config(robotNameKey);
    auto it = getRobot(name);
    if(it != const_robots_.end())
    {
      return const_cast<Robot &>(*it->get());
    }
    mc_rtc::log::error_and_throw<std::runtime_error>("{} No robot named {} in this controller", p, name);
  }
  else if(config.has(robotIndexKey))
  {
    mc_rtc::log::deprecated(prefix, robotIndexKey, robotNameKey);
    size_t robotIndex = config(robotIndexKey);
    if(robotIndex < robots_.size())
    {
      return *robots_[robotIndex];
    }
    mc_rtc::log::error_and_throw<std::runtime_error>("{} No robot with index {} in this controller ({} robots loaded)",
                                                     p, robotIndex, robots_.size());
  }
  if(!required)
  {
    if(defaultRobotName.size())
    {
      auto it = getRobot(defaultRobotName);
      if(it != const_robots_.end())
      {
        return const_cast<Robot &>(*it->get());
      }
      mc_rtc::log::error_and_throw<std::runtime_error>("{} No robot named {} in this controller", p, defaultRobotName);
    }
    return *robots_[0];
  }
  mc_rtc::log::error_and_throw<std::runtime_error>("{} \"{}\" is required", p, robotNameKey);
}

MC_RTC_diagnostic_pop;

} // namespace mc_rbdyn
