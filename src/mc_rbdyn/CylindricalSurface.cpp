/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/contact_transform.h>

namespace mc_rbdyn
{

struct CylindricalSurfaceImpl
{
public:
  double radius;
  double width;
};

CylindricalSurface::CylindricalSurface(std::string_view name, FramePtr frame, double radius, double width)
: Surface(name, frame), impl(new CylindricalSurfaceImpl({radius, width}))
{
  points().clear();
  points().push_back(sva::PTransformd(Eigen::Vector3d(-impl->width / 2, 0, 0)) * frame->X_b_f());
  points().push_back(sva::PTransformd(Eigen::Vector3d(impl->width / 2, 0, 0)) * frame->X_b_f());
}

CylindricalSurface::~CylindricalSurface() noexcept {}

double CylindricalSurface::radius() const noexcept
{
  return impl->radius;
}

double CylindricalSurface::width() const noexcept
{
  return impl->width;
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
  return std::make_shared<CylindricalSurface>(name(), to.frame(name()), impl->radius, impl->width);
}

} // namespace mc_rbdyn
