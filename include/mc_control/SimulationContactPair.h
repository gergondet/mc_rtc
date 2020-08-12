/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_SIMULATIONCONTACTPAIR_H_
#define _H_SIMULATIONCONTACTPAIR_H_

#include <mc_control/api.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/surface_hull.h>

#include <memory>

namespace mc_control
{

struct MC_CONTROL_DLLAPI SimulationContactPair
{
public:
  SimulationContactPair(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface);

  double update();

private:
  mc_rbdyn::ConstSurfacePtr robotSurface_;
  mc_rbdyn::ConstSurfacePtr envSurface_;
  std::shared_ptr<sch::S_Object> robotSch_;
  std::shared_ptr<sch::S_Object> envSch_;
  sch::CD_Pair pair;

private:
  void updateSCH(sch::S_Object * obj, const mc_rbdyn::Surface & robotSurface);
};

} // namespace mc_control

#endif
