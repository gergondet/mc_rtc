/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/Constraint.h>
#include <mc_solver/GenericLoader.h>

namespace mc_solver
{

struct MC_SOLVER_DLLAPI ConstraintLoader : public mc_solver::GenericLoader<ConstraintLoader, Constraint>
{
  static storage_t & storage();
};

} // namespace mc_solver
