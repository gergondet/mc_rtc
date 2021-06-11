#include <mc_rbdyn/Convex.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/SCHAddon.h>

namespace mc_rbdyn
{

Convex::Convex(ctor_token, S_ObjectPtr object, FramePtr frame, sva::PTransformd X_f_c)
: object_(object), frame_(frame), X_f_c_(std::move(X_f_c))
{
  registerUpdates(Update::Position, &Convex::updatePosition);
  addInputDependency(Update::Position, frame, Frame::Output::Position);
  addOutputDependency(Output::Position, Update::Position);

  updatePosition();
}

void Convex::updatePosition()
{
  sch::mc_rbdyn::transform(*object_, X_f_c_ * frame_->position());
}

} // namespace mc_rbdyn
