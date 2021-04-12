/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <string>
#include <vector>

namespace mc_rbdyn
{

/** \class CollisionDescription
 *
 * Light-weight description of a collision constraint between two robots
 *
 */
struct MC_RBDYN_DLLAPI CollisionDescription
{
  /** First collision object in the collision */
  std::string object1;
  /** Second collision object in the collision */
  std::string object2;
  /** Interaction distance */
  double iDist;
  /** Security distance */
  double sDist;
  /** Damping (0 is automatic */
  double damping;
};

/** \class Collision
 *
 * Specifiy a collision avoidance between two collision objects of two robots:
 * - the two robots can be the same (self-collision avoidance)
 * - '*' can be used as a wildcard character to specify multiple objects at once
 */
struct MC_RBDYN_DLLAPI Collision : public CollisionDescription
{
  /** General collision constructor */
  Collision(std::string_view r1,
            std::string_view r2,
            std::string_view o1,
            std::string_view o2,
            double i,
            double s,
            double d)
  : CollisionDescription{std::string(o1), std::string(o2), i, s, d}, robot1(r1), robot2(r2)
  {
  }
  /** Self-collision constructor */
  Collision(std::string_view r1, std::string_view o1, std::string_view o2, double i, double s, double d)
  : Collision(r1, r1, o1, o2, i, s, d)
  {
  }
  /** Constructor from a light description */
  Collision(std::string_view r1, std::string_view r2, const CollisionDescription & c)
  : Collision(r1, r2, c.object1, c.object2, c.iDist, c.sDist, c.damping)
  {
  }
  /** Self-collision constructor from a light description */
  Collision(std::string_view r1, const CollisionDescription & c) : Collision(r1, r1, c) {}
  /** First robot in the collision */
  std::string robot1;
  /** Second robot in the collision */
  std::string robot2;

  bool operator==(const Collision & rhs) const noexcept;
  bool operator!=(const Collision & rhs) const noexcept;

  /** Invalid collision constructor */
  Collision() = default;
};

/** \class CollisionVector
 *
 * Syntaxic sugar to specify many collisions at once without repeating the r1/r2 pair
 */
struct MC_RBDYN_DLLAPI CollisionVector : public std::vector<Collision>
{
  /** Specify a set of collisions for the r1/r2 pair */
  CollisionVector(std::string_view r1, std::string_view r2, const std::initializer_list<CollisionDescription> & cols);

  /** Specify a set of self-collisions */
  CollisionVector(std::string_view r, const std::initializer_list<CollisionDescription> & cols);

  /** Specify a set of collisions for the r1/r2 pair */
  CollisionVector(std::string_view r, const std::vector<CollisionDescription> & cols);

  /** Specify a set of collisions for the r1/r2 pair */
  CollisionVector(std::string_view r1, std::string_view r2, const std::vector<CollisionDescription> & cols);

  /** Copy from an existing vector of collisions */
  CollisionVector(const std::vector<Collision> & cols);

  /** Move construct from an existing vector of collisions */
  CollisionVector(std::vector<Collision> && cols);
};

} // namespace mc_rbdyn
