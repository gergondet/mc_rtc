/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/pragma.h>

#include <RBDyn/FK.h>
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace
{
template<typename sch_T>
void applyTransformToSchById(const rbd::MultiBody & mb,
                             const rbd::MultiBodyConfig & mbc,
                             std::map<std::string, std::pair<std::string, std::shared_ptr<sch_T>>> & schByName)
{
  for(auto & p : schByName)
  {
    unsigned int index = static_cast<unsigned int>(mb.bodyIndexByName(p.second.first));
    sch::mc_rbdyn::transform(*(p.second.second.get()), mbc.bodyPosW[index]);
  }
}

template<typename X, typename Y>
inline void update(std::map<X, Y> & oldData, const std::map<X, Y> & nData)
{
  for(const auto & p : nData)
  {
    oldData[p.first] = p.second;
  }
}

} // namespace

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

RobotPtr Robots::robot(std::string_view name)
{
  return std::const_pointer_cast<Robot>(static_cast<const Robots *>(this)->robot(name));
}

ConstRobotPtr Robots::robot(std::string_view name) const
{
  auto it = getRobot(name);
  if(it != const_robots_.end())
  {
    return *it;
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

RobotPtr Robots::addRobot(RobotPtr robot)
{
  robots_.push_back(robot);
  const_robots_.push_back(robot);
  return robot;
}

RobotPtr Robots::robotCopy(const Robot & robot, std::string_view name)
{
  return addRobot(robot.copy(robot.module(), name));
}

RobotPtr Robots::load(const RobotModule & module,
                      std::string_view name,
                      const std::optional<sva::PTransformd> & base,
                      const std::optional<std::string_view> & bName)
{
  return addRobot(std::allocate_shared<Robot>(Eigen::aligned_allocator_indirection<Robot>{}, Robot::make_shared_token{},
                                              module, name, base, bName));
}

RobotPtr Robots::loadFromUrdf(std::string_view name,
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
  return robots_.rbegin();
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

MC_RTC_diagnostic_pop;

} // namespace mc_rbdyn
