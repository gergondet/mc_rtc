/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>
#include <mc_rbdyn/fwd.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

/** \class Contact
 *
 * A lightweight contact description
 *
 */
struct MC_RBDYN_DLLAPI Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr double defaultFriction = 0.7;

  Contact(std::string_view r1,
          std::string_view r2,
          std::string_view r1Surface,
          std::string_view r2Surface,
          double friction = defaultFriction,
          const Eigen::Vector6d & dof = Eigen::Vector6d::Ones())
  : r1(r1), r2(r2), r1Surface(r1Surface), r2Surface(r2Surface), friction(friction), dof(dof)
  {
  }

  std::string r1;
  std::string r2;
  std::string r1Surface;
  std::string r2Surface;
  mutable double friction;
  mutable Eigen::Vector6d dof;

  bool operator<(const Contact & rhs) const
  {
    return r1 < rhs.r1 || (r1 == rhs.r1 && r1Surface < rhs.r1Surface)
           || (r1 == rhs.r1 && r1Surface == rhs.r1Surface && r2 < rhs.r2)
           || (r1 == rhs.r1 && r1Surface == rhs.r1Surface && r2 == rhs.r2 && r2Surface < rhs.r2Surface);
  }

  bool operator==(const Contact & rhs) const
  {
    return r1 == rhs.r1 && r2 == rhs.r2 && r1Surface == rhs.r1Surface && r2Surface == rhs.r2Surface;
  }

  bool operator!=(const Contact & rhs) const
  {
    return !(*this == rhs);
  }

  /** Default constructor, invalid contact */
  Contact() = default;
};

} // namespace mc_rbdyn

namespace std
{

template<>
struct hash<mc_rbdyn::Contact>
{
  std::size_t operator()(const mc_rbdyn::Contact & c) const noexcept
  {
    auto h = std::hash<std::string>{}(c.r1);
    // Same as boost::hash_combine
    auto hash_combine = [&h](const std::string & value) {
      h ^= std::hash<std::string>{}(value) + 0x9e3779b9 + (h << 6) + (h >> 2);
    };
    hash_combine(c.r1Surface);
    hash_combine(c.r2);
    hash_combine(c.r2Surface);
    return h;
  }
};

} // namespace std
