/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Frame.h>

namespace mc_rbdyn
{

/** A surface is attached to a frame and provides a list of points in the surface frame */
struct MC_RBDYN_DLLAPI Surface : public mc_rtc::shared<Surface>
{
  Surface(std::string_view name, FramePtr frame);

  virtual ~Surface() noexcept = default;

  /** Name of the surface */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

  /** Frame to which this surface is attached */
  inline const Frame & frame() const noexcept
  {
    return *frame_;
  }

  /** Frame to which this surface is attached */
  inline Frame & frame() noexcept
  {
    return *frame_;
  }

  /** Points in the surface frame */
  inline const std::vector<sva::PTransformd> & points() const noexcept
  {
    return points_;
  }

  /** Type of surface */
  virtual std::string type() const noexcept = 0;

  /** Copy a surface from one robot to another */
  virtual std::shared_ptr<Surface> copy(Robot & to) const = 0;

  inline const sva::PTransformd & X_b_s() const noexcept
  {
    return frame_->X_b_f();
  }

private:
  std::string name_;
  FramePtr frame_;

protected:
  std::vector<sva::PTransformd> points_;
};

} // namespace mc_rbdyn
