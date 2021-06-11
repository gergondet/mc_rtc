/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI GripperSurface : public Surface
{
public:
  GripperSurface(std::string_view name,
                 RobotFramePtr frame,
                 const std::vector<sva::PTransformd> & pointsFromOrigin,
                 const sva::PTransformd & X_b_motor,
                 double motorMaxTorque);

  ~GripperSurface() noexcept override = default;

  std::string type() const noexcept override;

  inline const std::vector<sva::PTransformd> & pointsFromOrigin() const noexcept
  {
    return pointsFromOrigin_;
  }

  inline const sva::PTransformd & X_b_motor() const noexcept
  {
    return X_b_motor_;
  }

  inline double motorMaxTorque() const noexcept
  {
    return motorMaxTorque_;
  }

  std::shared_ptr<Surface> copy(Robot & to) const override;

private:
  std::vector<sva::PTransformd> pointsFromOrigin_;
  sva::PTransformd X_b_motor_;
  double motorMaxTorque_;
};

} // namespace mc_rbdyn
