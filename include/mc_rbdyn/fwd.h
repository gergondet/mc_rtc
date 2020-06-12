/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/** Forward declarations for the mc_rbdyn library */

#pragma once

#include <memory>

namespace mc_rbdyn
{

struct Robots;

struct Robot;
using RobotPtr = std::shared_ptr<Robot>;
using ConstRobotPtr = std::shared_ptr<const Robot>;

struct Device;
using DevicePtr = std::unique_ptr<Device>;
using Sensor = Device;
using SensorPtr = DevicePtr;

struct Surface;
using SurfacePtr = std::shared_ptr<Surface>;
struct PlanarSurface;
struct CylindricalSurface;
struct GripperSurface;

struct Convex;
using ConvexPtr = std::shared_ptr<Convex>;

struct Frame;
using FramePtr = std::shared_ptr<Frame>;

} // namespace mc_rbdyn
