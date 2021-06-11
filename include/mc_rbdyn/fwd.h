/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/** Forward declarations for the mc_rbdyn library */

#pragma once

#include <memory>

namespace sch
{

class CD_Pair;

class S_Object;

} // namespace sch

namespace mc_rbdyn
{

struct Robots;
using RobotsPtr = std::shared_ptr<Robots>;

struct Robot;
using RobotPtr = std::shared_ptr<Robot>;
using ConstRobotPtr = std::shared_ptr<const Robot>;

struct Device;
using DevicePtr = std::unique_ptr<Device>;
using Sensor = Device;
using SensorPtr = DevicePtr;

struct Surface;
using SurfacePtr = std::shared_ptr<Surface>;
using ConstSurfacePtr = std::shared_ptr<const Surface>;

struct PlanarSurface;
struct CylindricalSurface;
struct GripperSurface;

struct Convex;
using ConvexPtr = std::shared_ptr<Convex>;
using ConstConvexPtr = std::shared_ptr<const Convex>;

struct RobotFrame;
using RobotFramePtr = std::shared_ptr<RobotFrame>;
using ConstRobotFramePtr = std::shared_ptr<const RobotFrame>;

struct FreeFrame;
using FreeFramePtr = std::shared_ptr<FreeFrame>;
using ConstFreeFramePtr = std::shared_ptr<const FreeFrame>;

struct CoM;
using CoMPtr = std::shared_ptr<CoM>;
using ConstCoMPtr = std::shared_ptr<const CoM>;

using S_ObjectPtr = std::shared_ptr<sch::S_Object>;
using ConstS_ObjectPtr = std::shared_ptr<const sch::S_Object>;

struct Momentum;
using MomentumPtr = std::shared_ptr<Momentum>;

struct Contact;

} // namespace mc_rbdyn
