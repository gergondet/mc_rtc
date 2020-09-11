/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_com_controller.h"

#include <mc_rbdyn/Surface.h>

#include <mc_filter/utils/clamp.h>

#include <mc_rtc/logging.h>

#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/VelocityTask.h>

#include <mc_solver/CoMInConvexConstraint.h>

/** Build a cube as a set of planes from a given origin and size */
static std::vector<tvm::geometry::PlanePtr> makeCube(const Eigen::Vector3d & origin, double size)
{
  return {std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{1, 0, 0}, origin + Eigen::Vector3d{-size, 0, 0}),
          std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{-1, 0, 0}, origin + Eigen::Vector3d{size, 0, 0}),
          std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{0, 1, 0}, origin + Eigen::Vector3d{0, -size, 0}),
          std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{0, -1, 0}, origin + Eigen::Vector3d{0, size, 0}),
          std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{0, 0, 1}, origin + Eigen::Vector3d{0, 0, -size}),
          std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{0, 0, -1}, origin + Eigen::Vector3d{0, 0, size})};
}

/** Build a cube to show in GUI */
static std::vector<std::vector<Eigen::Vector3d>> makeCubePolygon(const Eigen::Vector3d & origin, double size)
{
  return {// Back-face
          {origin + Eigen::Vector3d{-size, -size, -size}, origin + Eigen::Vector3d{-size, size, -size},
           origin + Eigen::Vector3d{-size, size, size}, origin + Eigen::Vector3d{-size, size, -size}},
          // Front-face
          {origin + Eigen::Vector3d{size, -size, -size}, origin + Eigen::Vector3d{size, size, -size},
           origin + Eigen::Vector3d{size, size, size}, origin + Eigen::Vector3d{size, -size, size}},
          // Left-face
          {origin + Eigen::Vector3d{-size, -size, -size}, origin + Eigen::Vector3d{size, -size, -size},
           origin + Eigen::Vector3d{size, -size, size}, origin + Eigen::Vector3d{-size, -size, size}},
          // Right-face
          {origin + Eigen::Vector3d{-size, size, -size}, origin + Eigen::Vector3d{size, size, -size},
           origin + Eigen::Vector3d{size, size, size}, origin + Eigen::Vector3d{-size, size, size}},
          // Bottom-face
          {origin + Eigen::Vector3d{-size, -size, -size}, origin + Eigen::Vector3d{size, -size, -size},
           origin + Eigen::Vector3d{size, size, -size}, origin + Eigen::Vector3d{size, -size, -size}},
          // Top-face
          {origin + Eigen::Vector3d{-size, -size, size}, origin + Eigen::Vector3d{size, -size, size},
           origin + Eigen::Vector3d{size, size, size}, origin + Eigen::Vector3d{size, -size, size}}};
}

namespace mc_control
{

MCCoMController::MCCoMController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    leftFootSurface_ = "LFullSole";
    rightFootSurface_ = "RFullSole";
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    leftFootSurface_ = "LeftFoot";
    rightFootSurface_ = "RightFoot";
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("MCCoMController does not support robot {}", robot().name());
  }

  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.frame(leftFootSurface_).position(), robot.frame(rightFootSurface_).position(), 0.5);
  });

  postureTask_->stiffness(1);
  postureTask_->weight(1);
  solver().addTask(postureTask_);

  mc_rtc::log::success("CoM sample controller initialized");
}

void MCCoMController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  comTask_ = std::make_shared<mc_tasks::CoMTask>(robot());
  comTask_->weight(1000);
  solver().addTask(comTask_);
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    solver().addContact({robot().name(), "ground", "LFullSole", "AllGround"});
    solver().addContact({robot().name(), "ground", "RFullSole", "AllGround"});
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    solver().addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    solver().addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("CoM sample controller does not support this robot {}",
                                                     robot().name());
  }
  solver().addTask(std::make_shared<mc_tasks::OrientationTask>(robot().frame("WAIST_R_S")));
  solver().addTask(std::make_shared<mc_tasks::TransformTask>(robot().frame("l_wrist")));
  solver().addTask(std::make_shared<mc_tasks::TransformTask>(robot().frame("r_wrist")));
  solver().addConstraint(dynamicsConstraint_);
  solver().addConstraint(collisionConstraint_);
  solver().addConstraint(compoundJointConstraint_);

  auto comInConvexCstr = std::make_shared<mc_solver::CoMInConvexConstraint>(robot());
  comInConvexCstr->setPlanes(solver(), makeCube(robot().com().com(), 0.1));
  solver().addConstraint(comInConvexCstr);
  // Draw the corresponding box
  auto poly = makeCubePolygon(robot().com().com(), 0.1);
  gui().addElement({"CoM constraint"},
                   mc_rtc::gui::Polygon("polygon", mc_rtc::gui::Color::Red, [p = std::move(poly)]() { return p; }));
  auto findLWristTask = [this]() -> mc_tasks::MetaTaskPtr {
    for(const auto & t : solver().tasks())
    {
      if(t->name() == fmt::format("transform_jvrc1_l_wrist"))
      {
        return t;
      }
    }
    return nullptr;
  };
  gui().addElement({}, mc_rtc::gui::Button("Remove l_wrist task", [=]() { solver().removeTask(findLWristTask()); }),
                   mc_rtc::gui::Button("Add l_wrist task", [=]() {
                     if(!findLWristTask())
                     {
                       solver().addTask(std::make_shared<mc_tasks::TransformTask>(robot().frame("l_wrist")));
                     }
                   }));
}

bool MCCoMController::run()
{
  return MCController::run();
}

} // namespace mc_control
