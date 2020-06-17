/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct PlanarSurfaceImpl;

struct MC_RBDYN_DLLAPI PlanarSurface : public Surface
{
  PlanarSurface(std::string_view name, FramePtr frame, const std::vector<std::pair<double, double>> & planarPoints);

  ~PlanarSurface() noexcept override;

  const std::vector<std::pair<double, double>> & planarPoints() const;

  std::string type() const noexcept override;

  std::shared_ptr<Surface> copy(Robot & to) const override;

private:
  std::unique_ptr<PlanarSurfaceImpl> impl;
};

} // namespace mc_rbdyn
