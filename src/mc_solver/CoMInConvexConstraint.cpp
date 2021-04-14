/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CoMInConvexConstraint.h>

#include <mc_solver/ConstraintLoader.h>

namespace mc_solver
{

CoMInConvexConstraint::CoMInConvexConstraint(mc_rbdyn::Robot & robot)
: function_(std::make_shared<mc_tvm::CoMInConvexFunction>(robot))
{
}

void CoMInConvexConstraint::addToSolver(QPSolver & solver)
{
  if(!constraint_)
  {
    constraint_ = solver.problem().add(function_ >= 0., tvm::task_dynamics::VelocityDamper(solver.dt(), config_),
                                       {tvm::requirements::PriorityLevel(0)});
  }
}

void CoMInConvexConstraint::removeFromSolver(QPSolver & solver)
{
  if(constraint_)
  {
    solver.problem().remove(*constraint_);
    constraint_.reset();
  }
}

void CoMInConvexConstraint::setPlanes(QPSolver & solver,
                                      const std::vector<tvm::geometry::PlanePtr> & planes,
                                      const std::optional<tvm::task_dynamics::VelocityDamper::Config> & config)
{
  bool inSolver = constraint_ != nullptr;
  if(inSolver)
  {
    removeFromSolver(solver);
  }
  if(config)
  {
    config_ = *config;
  }
  function_->reset();
  for(const auto & p : planes)
  {
    function_->addPlane(p);
  }
  if(inSolver)
  {
    addToSolver(solver);
  }
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintLoader::register_load_function(
    "CoMInConvex",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto ret = std::make_shared<mc_solver::CoMInConvexConstraint>(
          solver.robots().fromConfig(config, "CoMInConvexConstraint"));
      // FIXME Load planes from config
      return ret;
    });
}
