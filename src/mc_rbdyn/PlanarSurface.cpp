/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/Robot.h>

namespace mc_rbdyn
{

PlanarSurface::PlanarSurface(std::string_view name,
                             RobotFramePtr frame,
                             const std::vector<std::pair<double, double>> & planarPoints)
: Surface(name, frame), planarPoints_(planarPoints)
{
  points_.clear();
  for(std::pair<double, double> & p : planarPoints_)
  {
    points_.push_back(sva::PTransformd(Eigen::Vector3d(p.first, p.second, 0)) * frame->X_b_f());
  }
  if(points_.size() == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot create planar surface {} for {} as it would have 0 points",
                                                     name, frame->robot().name());
  }
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
  return std::make_shared<PlanarSurface>(name(), to.frame(name()), planarPoints_);
}

} // namespace mc_rbdyn
