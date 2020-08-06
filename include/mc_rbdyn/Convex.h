/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <mc_rtc/shared.h>

#include <tvm/graph/abstract/Node.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

/** A convex is an SCH object associated to a Frame
 *
 * It defines a single output:
 * - Position: update the convex position according to the frame
 */
struct MC_RBDYN_DLLAPI Convex : public tvm::graph::abstract::Node<Convex>, mc_rtc::shared<Convex>
{
  friend struct Robot;

private:
  struct ctor_token
  {
  };

public:
  SET_OUTPUTS(Convex, Position)
  SET_UPDATES(Convex, Position)

  /** Construct a convex from associated S_Object and frame
   *
   * \param object Pointer to the loaded S_Object
   *
   * \param frame Frame of this convex
   *
   * \param X_f_c Transformation from the frame to the convex
   */
  Convex(ctor_token, S_ObjectPtr object, FramePtr frame, sva::PTransformd X_f_c = sva::PTransformd::Identity());

  /** Access the underlying frame object */
  inline const Frame & frame() const noexcept
  {
    return *frame_;
  }

  /** Access the underlying frame (non-const) */
  inline Frame & frame() noexcept
  {
    return *frame_;
  }

  /** Access the underlying SCH object */
  inline ConstS_ObjectPtr convex() const noexcept
  {
    return object_;
  }

  /** Access the underlying SCH object */
  inline S_ObjectPtr convex() noexcept
  {
    return object_;
  }

  /** Access the transformation from the frame to the convex */
  inline const sva::PTransformd & X_f_c() const noexcept
  {
    return X_f_c_;
  }

  /** Access the transformation from the frame to the convex */
  inline sva::PTransformd & X_f_c() noexcept
  {
    return X_f_c_;
  }

private:
  S_ObjectPtr object_;
  FramePtr frame_;
  sva::PTransformd X_f_c_;

  void updatePosition();
};

} // namespace mc_rbdyn
