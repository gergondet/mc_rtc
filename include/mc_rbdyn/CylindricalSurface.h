/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI CylindricalSurface : public Surface
{
  CylindricalSurface(std::string_view name, RobotFramePtr frame, double radius, double width);

  ~CylindricalSurface() noexcept override = default;

  inline double radius() const noexcept
  {
    return radius_;
  }

  inline double width() const noexcept
  {
    return width_;
  }

  std::string type() const noexcept override;

  std::shared_ptr<Surface> copy(Robot & to) const override;

private:
  double radius_;
  double width_;
};

} // namespace mc_rbdyn
