#pragma once

#include <mc_rbdyn/api.h>

#include <string>

/** \class Collision
 * \brief Used to define a collision constraint between two bodies
 */

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI Collision
{
  Collision() : body1("NONE"), body2("NONE") {}
  Collision(const std::string & b1, const std::string & b2, double i, double s, double d)
  : body1(b1), body2(b2), iDist(i), sDist(s), damping(d)
  {}
  std::string body1; /** First body in the constraint */
  std::string body2; /** Second body in the constraint */
  double iDist; /** Interaction distance */
  double sDist; /** Security distance */
  double damping; /** Damping (0 is automatic */
  inline bool isNone() { return body1 == "NONE" && body2 == "NONE"; }
};

MC_RBDYN_DLLAPI bool operator==(const Collision & lhs, const Collision & rhs);

MC_RBDYN_DLLAPI bool operator!=(const Collision & lhs, const Collision & rhs);

MC_RBDYN_DLLAPI std::ostream & operator<<(std::ostream & os, const Collision & c);

}
