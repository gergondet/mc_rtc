/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <sch/CD/CD_Pair.h>
#include <sch/STP-BV/STP_BV.h>
#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>

#include <mc_rbdyn/api.h>

namespace sch
{

namespace mc_rbdyn
{

MC_RBDYN_DLLAPI void transform(S_Object & obj, const sva::PTransformd & t);

MC_RBDYN_DLLAPI ::mc_rbdyn::S_ObjectPtr make_polyhedron(std::string_view filename);

MC_RBDYN_DLLAPI double distance(CD_Pair & pair, Eigen::Vector3d & p1, Eigen::Vector3d & p2);

} // namespace mc_rbdyn

} // namespace sch
