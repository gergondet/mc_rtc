#include <mc_rbdyn/CollisionPair.h>

#include <mc_rbdyn/Convex.h>
#include <mc_rbdyn/SCHAddon.h>

namespace mc_rbdyn
{

CollisionPair::CollisionPair(ConvexPtr c1, ConvexPtr c2)
: c1_(c1), c2_(c2), pair_(c1_->convex().get(), c2_->convex().get())
{
  registerUpdates(Update::Distance, &CollisionPair::updateDistance);
  addOutputDependency(Output::Distance, Update::Distance);
  addInputDependency(Update::Distance, c1, Convex::Output::Position);
  addInputDependency(Update::Distance, c2, Convex::Output::Position);

  updateDistance();
}

void CollisionPair::updateDistance()
{
  distance_ = sch::mc_rbdyn::distance(pair_, p1_, p2_);
}

} // namespace mc_rbdyn
