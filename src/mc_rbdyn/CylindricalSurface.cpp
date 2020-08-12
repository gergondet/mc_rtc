/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/Robots.h>

namespace mc_rbdyn
{

CylindricalSurface::CylindricalSurface(std::string_view name, FramePtr frame, double radius, double width)
: Surface(name, frame), radius_(radius), width_(width)
{
  points_.clear();
  points_.push_back(sva::PTransformd(Eigen::Vector3d(-width_ / 2, 0, 0)) * frame->X_b_f());
  points_.push_back(sva::PTransformd(Eigen::Vector3d(width_ / 2, 0, 0)) * frame->X_b_f());
}

std::string CylindricalSurface::type() const noexcept
{
  return "cylindrical";
}

std::shared_ptr<Surface> CylindricalSurface::copy(Robot & to) const
{
  if(!to.hasFrame(name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No frame {} in destination robot {}", name(), to.name());
  }
  return std::make_shared<CylindricalSurface>(name(), to.frame(name()), radius_, width_);
}

} // namespace mc_rbdyn
