#pragma once

#include <memory>

#include <Eigen/Core>

#include <mc_tasks/ImpedanceGains.h>
#include <mc_tasks/PostureTask.h>

namespace mc_tasks
{

template<typename T, typename... Args>
std::shared_ptr<T> make_shared_aligned(Args &&... args)
{
  using Allocator = Eigen::aligned_allocator<T>;
  return std::allocate_shared<T>(Allocator{}, std::forward<Args>(args)...);
}

template<typename T, typename U>
std::shared_ptr<T> cast(const std::shared_ptr<U> & p)
{
  return std::static_pointer_cast<T>(p);
}

void PostureTaskTarget(mc_tasks::PostureTask & task, const std::map<std::string, std::vector<double>> & target)
{
  mc_rtc::map<std::string, std::vector<double>> tgt;
  for(auto & t : target)
  {
    tgt[t.first] = t.second;
  }
  task.target(tgt);
}

namespace force
{

namespace details
{

using ImpedanceVecdStrictlyPositive = ImpedanceVecd<true>;
using ImpedanceVecdPositive = ImpedanceVecd<false>;

} // namespace details

} // namespace force

} // namespace mc_tasks
