/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct GripperSurfaceImpl;

struct MC_RBDYN_DLLAPI GripperSurface : public Surface
{
public:
  GripperSurface(std::string_view name,
                 FramePtr frame,
                 const std::vector<sva::PTransformd> & pointsFromOrigin,
                 const sva::PTransformd & X_b_motor,
                 double motorMaxTorque);

  ~GripperSurface() noexcept override;

  std::string type() const noexcept override;

  const std::vector<sva::PTransformd> & pointsFromOrigin() const noexcept;

  const sva::PTransformd & X_b_motor() const noexcept;

  double motorMaxTorque() const noexcept;

  std::shared_ptr<Surface> copy(Robot & to) const override;

private:
  std::unique_ptr<GripperSurfaceImpl> impl;
};

} // namespace mc_rbdyn
