/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/contact_transform.h>

namespace mc_rbdyn
{

struct GripperSurfaceImpl
{
public:
  std::vector<sva::PTransformd> pointsFromOrigin;
  sva::PTransformd X_b_motor;
  double motorMaxTorque;
};

GripperSurface::GripperSurface(std::string_view name,
                               FramePtr frame,
                               const std::vector<sva::PTransformd> & pointsFromOrigin,
                               const sva::PTransformd & X_b_motor,
                               double motorMaxTorque)
: Surface(name, frame), impl(new GripperSurfaceImpl({pointsFromOrigin, X_b_motor, motorMaxTorque}))
{
  points().clear();
  for(sva::PTransformd & p : impl->pointsFromOrigin)
  {
    points().push_back(p * frame->X_b_f());
  }
}

GripperSurface::~GripperSurface() noexcept {}

std::string GripperSurface::type() const noexcept
{
  return "gripper";
}

const std::vector<sva::PTransformd> & GripperSurface::pointsFromOrigin() const noexcept
{
  return impl->pointsFromOrigin;
}

const sva::PTransformd & GripperSurface::X_b_motor() const noexcept
{
  return impl->X_b_motor;
}

double GripperSurface::motorMaxTorque() const noexcept
{
  return impl->motorMaxTorque;
}

std::shared_ptr<Surface> GripperSurface::copy(Robot & to) const
{
  if(!to.hasFrame(name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No frame {} in destination robot {}", name(), to.name());
  }
  return std::make_shared<GripperSurface>(name(), to.frame(name()), impl->pointsFromOrigin, impl->X_b_motor,
                                          impl->motorMaxTorque);
}

} // namespace mc_rbdyn
