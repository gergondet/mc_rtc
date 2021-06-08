/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/FreeFrame.h>

#include <mc_rbdyn/hat.h>

namespace mc_rbdyn
{

FreeFrame::FreeFrame(std::string_view name, const sva::PTransformd & pos, const sva::MotionVecd & vel)
: FreeFrame(nullptr, name, sva::PTransformd::Identity(), pos, vel)
{
}

FreeFrame::FreeFrame(std::string_view name, FreeFrame & frame, const sva::PTransformd & X_f1_f2)
: FreeFrame(frame, name, X_f1_f2)
{
}

FreeFrame::FreeFrame(FreeFramePtr parent,
                     std::string_view name,
                     const sva::PTransformd & X_p_f,
                     const sva::PTransformd & pos,
                     const sva::MotionVecd & vel)
: parent_(parent), X_p_f_(X_p_f), name_(name), position_(pos), velocity_(vel)
{
  if(!parent_)
  {
    return;
  }
  root_ = parent_->root_ ? parent_->root_ : parent_;
  // clang-format off
  registerUpdates(
                  Update::Position, &FreeFrame::updatePosition,
                  Update::Velocity, &FreeFrame::updateVelocity);
  // clang-format off

  addOutputDependency(Output::Position, Update::Position);
  addInputDependency(Update::Position, parent_, Output::Position);

  addOutputDependency(Output::Velocity, Update::Velocity);
  addInputDependency(Update::Velocity, parent_, Output::Velocity);

  addInternalDependency(Update::Velocity, Update::Position); // for h_

  updatePosition();
  updateVelocity();
}

void FreeFrame::updatePosition()
{
  X_r_f_ = X_p_f_ * parent_->X_r_f_;
  const auto & X_0_b = root_->position();
  position_ = X_r_f_ * X_0_b;
  h_ = -hat(X_0_b.rotation().transpose() * X_r_f_.translation());
}

void FreeFrame::updateVelocity()
{
  velocity_ = root_->velocity();
  velocity_.linear().noalias() += h_ * velocity_.angular();
}

} // namespace mc_rbdyn
