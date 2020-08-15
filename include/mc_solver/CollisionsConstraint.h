/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/Constraint.h>

#include <mc_tvm/CollisionFunction.h>

#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/fwd.h>

#include <mc_rtc/map.h>

#include <tvm/ControlProblem.h>

namespace mc_solver
{

struct QPSolver;

/** Collision constraint
 *
 * This manages a set of collision avoidance constraints between robots
 */
struct MC_SOLVER_DLLAPI CollisionsConstraint : public Constraint
{
public:
  /** Default value of damping offset */
  constexpr static double defaultDampingOffset = 0.1;

public:
  /** Constructor */
  CollisionsConstraint();

  ~CollisionsConstraint() override = default;

  /** Add a collision
   *
   * \param solver Solver where the constraint will be added
   *
   * \param collision Collision to be added
   */
  void addCollision(QPSolver & solver, const mc_rbdyn::Collision & collision);

  /** Add mutiple collisions
   *
   * \param solver Solver where the constraint will be added
   *
   * \param collisions Vector of collisions
   */
  void addCollisions(QPSolver & solver, const mc_rbdyn::CollisionVector & collisions);

  /** Remove a set of collisions
   *
   * \param solver The solver into which this constraint was added
   *
   * \param cols List of collisions to remove
   */
  void removeCollisions(QPSolver & solver, const mc_rbdyn::CollisionVector & cols);

  /** Remove all collisions between two robots
   *
   * Note: this will remove collision that were added for (r1, r2) as well as (r2, r1)
   *
   * \param r1 First robot involved
   *
   * \param r2 Second robot involved
   */
  void removeCollisions(QPSolver & solver, std::string_view r1, std::string_view r2);

  /** Remove all collisions from the constraint */
  void reset(QPSolver & solver);

  void addToSolver(QPSolver & solver) override;

  void removeFromSolver(QPSolver & solver) override;

  void update(QPSolver & solver) override;

private:
  bool inSolver_ = false;

  enum class Monitoring
  {
    OFF,
    AUTO,
    USER
  };

  struct CollisionData
  {
    CollisionData(uint64_t id, const mc_rbdyn::Collision & col);
    uint64_t id;
    mc_rbdyn::Collision collision;
    mc_tvm::CollisionFunctionPtr function;
    tvm::TaskWithRequirementsPtr task;
    Monitoring monitored;
  };
  /** All collisions handled by this constraint */
  std::vector<CollisionData> data_;
  /** Source for id generation */
  uint64_t nextId_ = 0;

  std::vector<CollisionData>::iterator getData(const mc_rbdyn::Collision & col);

  CollisionData & getData(uint64_t id);

  void addCollision(QPSolver & solver, CollisionData & data);

  void updateCollision(QPSolver & solver, CollisionData & data);

  void removeCollision(QPSolver & solver, CollisionData & data);

  void addMonitorButton(QPSolver & solver, CollisionData & data);

  void removeMonitorButton(QPSolver & solver, CollisionData & data);

  void toggleCollisionMonitor(QPSolver & solver, CollisionData & data, Monitoring monitoring);
};

using CollisionsConstraintPtr = std::shared_ptr<CollisionsConstraint>;

} // namespace mc_solver
