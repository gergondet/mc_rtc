/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/Convex.h>
#include <mc_rbdyn/RobotFrame.h>
#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <sch/CD/CD_Pair.h>

#include <RBDyn/Jacobian.h>

namespace mc_tvm
{

/** This class implements a collision function for two given objects */
class MC_TVM_DLLAPI CollisionFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(CollisionFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * \param c1 First collision object
   *
   * \param c2 Second collision object
   *
   * \param dt Timestep
   *
   */
  CollisionFunction(mc_rbdyn::Convex & c1, mc_rbdyn::Convex & c2, double dt);

  /** Called *once* every iteration to advance the iteration counter */
  void tick();

  /** Distance between the two objects */
  inline double distance() const noexcept
  {
    return this->value()(0);
  }

  /** First convex involved in the collision */
  inline const mc_rbdyn::Convex & c1() const noexcept
  {
    return *c1_;
  }

  /** Closest point on c1 in inertial frame coordinates */
  inline const Eigen::Vector3d & p1() const noexcept
  {
    return p1_;
  }

  /** Second convex involved in the collision */
  inline const mc_rbdyn::Convex & c2() const noexcept
  {
    return *c2_;
  }

  /** Closest point on c2 in inertial frame coordinates */
  inline const Eigen::Vector3d & p2() const noexcept
  {
    return p2_;
  }

protected:
  /* Update functions */
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  uint64_t iter_ = 0;
  uint64_t prevIter_ = 0;

  mc_rbdyn::ConvexPtr c1_;
  mc_rbdyn::ConvexPtr c2_;
  double dt_;

  Eigen::Vector3d p1_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d p2_ = Eigen::Vector3d::Zero();

  sch::CD_Pair pair_;

  struct ObjectData
  {
    mc_rbdyn::RobotFramePtr frame_;
    Eigen::Vector3d nearestPoint_;
    rbd::Jacobian jac_;
  };
  std::vector<ObjectData> data_;

  Eigen::Vector3d normVecDist_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prevNormVecDist_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d speedVec_ = Eigen::Vector3d::Zero();

  /** Intermediate computation */
  Eigen::MatrixXd fullJac_;
  Eigen::MatrixXd distJac_;
};

using CollisionFunctionPtr = std::shared_ptr<CollisionFunction>;

} // namespace mc_tvm
