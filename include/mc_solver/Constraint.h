/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/api.h>

#include <mc_rtc/shared.h>

namespace mc_solver
{

struct QPSolver;

/** \class Constraint
 *
 * \brief This class is a basis to wrap TVM functions that will be added as constraints into the problem
 */
struct MC_SOLVER_DLLAPI Constraint : public mc_rtc::shared<Constraint>
{
public:
  /** Set the name for a constraint */
  inline void name(std::string_view name) noexcept
  {
    name_ = name;
  }

  /** Returns the constraint name */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

  /** This function is called when the constraint is added to the problem */
  virtual void addToSolver(QPSolver & solver) = 0;

  /** This function is called every iteration while the constraint is in the solver
   *
   * The default implementation does nothing
   */
  virtual void update(QPSolver &) {}

  /** This function is called when the constraint is removed from the problem */
  virtual void removeFromSolver(QPSolver & solver) = 0;

  /** Virtual destructor */
  virtual ~Constraint() {}

protected:
  /** Name of the constraint used for GUI and logging purposes */
  std::string name_;
};

using ConstraintPtr = std::shared_ptr<Constraint>;

} // namespace mc_solver
