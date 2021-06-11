/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

Surface::Surface(std::string_view name, RobotFramePtr frame) : name_(name), frame_(frame) {}

} // namespace mc_rbdyn
