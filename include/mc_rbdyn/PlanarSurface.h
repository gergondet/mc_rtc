/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI PlanarSurface : public Surface
{
  PlanarSurface(std::string_view name,
                RobotFramePtr frame,
                const std::vector<std::pair<double, double>> & planarPoints);

  ~PlanarSurface() noexcept override = default;

  inline const std::vector<std::pair<double, double>> & planarPoints() const noexcept
  {
    return planarPoints_;
  }

  std::string type() const noexcept override;

  std::shared_ptr<Surface> copy(Robot & to) const override;

private:
  std::vector<std::pair<double, double>> planarPoints_;
};

} // namespace mc_rbdyn
