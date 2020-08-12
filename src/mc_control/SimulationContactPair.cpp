/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/SimulationContactPair.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/Surface.h>

namespace mc_control
{

SimulationContactPair::SimulationContactPair(const mc_rbdyn::Surface & robotSurface,
                                             const mc_rbdyn::Surface & envSurface)
: robotSurface_(robotSurface), envSurface_(envSurface), robotSch_(mc_rbdyn::surface_to_sch(robotSurface, 0.005, 8)),
  envSch_(mc_rbdyn::surface_to_sch(envSurface, -0.001, 8)), pair(robotSch_.get(), envSch_.get())
{
}

double SimulationContactPair::update()
{
  updateSCH(robotSch_.get(), *robotSurface_);
  updateSCH(envSch_.get(), *envSurface_);
  return pair.getDistance();
}

void SimulationContactPair::updateSCH(sch::S_Object * obj, const mc_rbdyn::Surface & s)
{
  sch::mc_rbdyn::transform(*obj, s.robot().frame(s.frame().body()).position());
}

} // namespace mc_control
