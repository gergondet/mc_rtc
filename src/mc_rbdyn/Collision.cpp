/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Collision.h>

namespace mc_rbdyn
{

bool Collision::operator==(const Collision & rhs) const noexcept
{
  return robot1 == rhs.robot1 && robot2 == rhs.robot2 && object1 == rhs.object1 && object2 == rhs.object2;
}

bool Collision::operator!=(const Collision & rhs) const noexcept
{
  return !(*this == rhs);
}

CollisionVector::CollisionVector(std::string_view r1,
                                 std::string_view r2,
                                 const std::initializer_list<CollisionDescription> & cols)
{
  for(const auto & col : cols)
  {
    emplace_back(r1, r2, col.object1, col.object2, col.iDist, col.sDist, col.damping);
  }
}

CollisionVector::CollisionVector(std::string_view r, const std::initializer_list<CollisionDescription> & cols)
: CollisionVector(r, r, cols)
{
}

CollisionVector::CollisionVector(std::string_view r1,
                                 std::string_view r2,
                                 const std::vector<CollisionDescription> & cols)
{
  for(const auto & col : cols)
  {
    emplace_back(r1, r2, col.object1, col.object2, col.iDist, col.sDist, col.damping);
  }
}

CollisionVector::CollisionVector(std::string_view r, const std::vector<CollisionDescription> & cols)
: CollisionVector(r, r, cols)
{
}

CollisionVector::CollisionVector(const std::vector<Collision> & cols) : std::vector<Collision>(cols) {}

CollisionVector::CollisionVector(std::vector<Collision> && cols) : std::vector<Collision>(cols) {}

CollisionVector::CollisionVector() = default;

} // namespace mc_rbdyn
