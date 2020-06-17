/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/Robot.h>

namespace mc_rbdyn
{

struct PlanarSurfaceImpl
{
public:
  std::vector<std::pair<double, double>> planarPoints;
};

PlanarSurface::PlanarSurface(std::string_view name,
                             FramePtr frame,
                             const std::vector<std::pair<double, double>> & planarPoints)
: Surface(name, frame), impl(new PlanarSurfaceImpl({planarPoints}))
{
  points().clear();
  for(std::pair<double, double> & p : impl->planarPoints)
  {
    points().push_back(sva::PTransformd(Eigen::Vector3d(p.first, p.second, 0)) * frame->X_b_f());
  }
}

PlanarSurface::~PlanarSurface() noexcept {}

const std::vector<std::pair<double, double>> & PlanarSurface::planarPoints() const
{
  return impl->planarPoints;
}

std::string PlanarSurface::type() const noexcept
{
  return "planar";
}

std::shared_ptr<Surface> PlanarSurface::copy(Robot & to) const
{
  if(!to.hasFrame(name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No frame {} in destination robot {}", name(), to.name());
  }
  return std::make_shared<PlanarSurface>(name(), to.frame(name()), impl->planarPoints);
}

} // namespace mc_rbdyn
