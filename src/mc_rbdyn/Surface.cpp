/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct SurfaceImpl
{
public:
  std::string name_;
  FramePtr frame_;
  std::vector<sva::PTransformd> points_;
};

Surface::Surface(std::string_view name, FramePtr frame) : impl(new SurfaceImpl{std::string(name), frame, {}}) {}

Surface::~Surface() noexcept {}

const std::string & Surface::name() const noexcept
{
  return impl->name_;
}

ConstFramePtr Surface::frame() const noexcept
{
  return impl->frame_;
}

FramePtr Surface::frame() noexcept
{
  return impl->frame_;
}

const std::vector<sva::PTransformd> & Surface::points() const noexcept
{
  return impl->points_;
}

std::vector<sva::PTransformd> & Surface::points() noexcept
{
  return impl->points_;
}

} // namespace mc_rbdyn
