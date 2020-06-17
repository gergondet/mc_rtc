/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct CylindricalSurfaceImpl;

struct MC_RBDYN_DLLAPI CylindricalSurface : public Surface
{
  CylindricalSurface(std::string_view name, FramePtr frame, double radius, double width);

  ~CylindricalSurface() noexcept override;

  double radius() const noexcept;

  double width() const noexcept;

  std::string type() const noexcept override;

  std::shared_ptr<Surface> copy(Robot & to) const override;

private:
  std::unique_ptr<CylindricalSurfaceImpl> impl;
};

} // namespace mc_rbdyn
