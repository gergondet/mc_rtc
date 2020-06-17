/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/api.h>

#include <tvm/graph/abstract/Node.h>

namespace mc_rbdyn
{

/** A collision pair provides distance information between two convex
 *
 * It defines a single output:
 * - Distance: distance between the two convex and closest points
 *
 */
struct MC_RBDYN_DLLAPI CollisionPair : public tvm::graph::abstract::Node<CollisionPair>
{
  SET_OUTPUTS(CollisionPair, Distance)
  SET_UPDATES(CollisionPair, Distance)

  CollisionPair(ConvexPtr c1, ConvexPtr c2);

  /** Returns the distance between the two convex */
  inline double distance() const noexcept
  {
    return distance_;
  }

  /** Returns the point on c1 that is closes to c2 (in woorld coordinates) */
  inline const Eigen::Vector3d & p1() const noexcept
  {
    return p1_;
  }

  /** Returns the point on c2 that is closes to c1 (in woorld coordinates) */
  inline const Eigen::Vector3d & p2() const noexcept
  {
    return p2_;
  }

private:
  ConvexPtr c1_;
  ConvexPtr c2_;
  sch::CD_Pair pair_;
  double distance_;
  Eigen::Vector3d p1_;
  Eigen::Vector3d p2_;

  void updateDistance();
};

} // namespace mc_rbdyn
