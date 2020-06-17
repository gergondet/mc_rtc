/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Frame.h>

namespace mc_rbdyn
{

struct SurfaceImpl;

/** A surface is attached to a frame and provides a list of points in the surface frame */
struct MC_RBDYN_DLLAPI Surface
{
  Surface(std::string_view name, FramePtr frame);

  virtual ~Surface() noexcept;

  /** Name of the surface */
  const std::string & name() const noexcept;

  /** Frame to which this surface is attached */
  ConstFramePtr frame() const noexcept;

  /** Frame to which this surface is attached */
  FramePtr frame() noexcept;

  /** Points in the surface frame */
  const std::vector<sva::PTransformd> & points() const noexcept;

  /** Type of surface */
  virtual std::string type() const noexcept = 0;

  /** Copy a surface from one robot to another */
  virtual std::shared_ptr<Surface> copy(Robot & to) const = 0;

protected:
  std::vector<sva::PTransformd> & points() noexcept;

private:
  std::unique_ptr<SurfaceImpl> impl;
};

typedef std::shared_ptr<Surface> SurfacePtr;

} // namespace mc_rbdyn
