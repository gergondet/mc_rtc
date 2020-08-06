/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/ConstraintLoader.h>

namespace mc_solver
{

ConstraintLoader::storage_t & ConstraintLoader::storage()
{
  static storage_t storage_;
  return storage_;
}

} // namespace mc_solver
