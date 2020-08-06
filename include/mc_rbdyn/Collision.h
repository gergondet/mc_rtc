/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <string>
#include <vector>

namespace mc_rbdyn
{

/** \class Collision
 *
 * Specifiy a collision avoidance between two collision objects of two robots:
 * - the two robots can be the same (self-collision avoidance)
 * - '*' can be used as a wildcard character to specify multiple objects at once
 */
struct MC_RBDYN_DLLAPI Collision
{
  /** General collision constructor */
  Collision(std::string_view r1,
            std::string_view r2,
            std::string_view o1,
            std::string_view o2,
            double i,
            double s,
            double d)
  : robot1(r1), robot2(r2), object1(o1), object2(o2), iDist(i), sDist(s), damping(d)
  {
  }
  /** Self-collision constructor */
  Collision(std::string_view r1, std::string_view o1, std::string_view o2, double i, double s, double d)
  : robot1(r1), robot2(r1), object1(o1), object2(o2), iDist(i), sDist(s), damping(d)
  {
  }
  /** First robot in the collision */
  std::string robot1;
  /** Second robot in the collision */
  std::string robot2;
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

  bool operator==(const Collision & rhs) const;
  bool operator!=(const Collision & rhs) const;
};

/** \class CollisionVector
 *
 * Syntaxic sugar to specify many collisions at once without repeating the r1/r2 pair
 */
struct MC_RBDYN_DLLAPI CollisionVector : public std::vector<Collision>
{
  struct CollisionDescription
  {
    std::string object1;
    std::string object2;
    double iDist;
    double sDist;
    double damping;
  };

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
