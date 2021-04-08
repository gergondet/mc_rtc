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

namespace
{

struct DummyContactConstraint : public mc_solver::Constraint
{
  ~DummyContactConstraint() final {}

  DummyContactConstraint() : mc_solver::Constraint()
  {
    name_ = "ContactConstraint";
    mc_rtc::log::warning("Contact constraint is now an empty class, we keep an empty constraint around for backward "
                         "compatibility but you can safely remove it from your YAML/JSON configuration file");
  }

  void addToSolver(mc_solver::QPSolver &) final {}

  void removeFromSolver(mc_solver::QPSolver &) final {}
};

static auto registered = mc_solver::ConstraintLoader::register_load_function(
    "contact",
    [](mc_solver::QPSolver &, const mc_rtc::Configuration &) { return std::make_shared<DummyContactConstraint>(); });
} // namespace
