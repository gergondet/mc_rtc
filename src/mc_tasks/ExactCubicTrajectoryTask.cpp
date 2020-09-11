/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/ExactCubicTrajectoryTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_trajectory/ExactCubic.h>
#include <mc_trajectory/InterpolatedRotation.h>

#include <mc_rtc/gui/Arrow.h>

namespace mc_tasks
{

ExactCubicTrajectoryTask::ExactCubicTrajectoryTask(mc_rbdyn::Frame & frame,
                                                   double duration,
                                                   double stiffness,
                                                   double weight,
                                                   const sva::PTransformd & target,
                                                   const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                                                   const Eigen::Vector3d & initVel,
                                                   const Eigen::Vector3d & initAcc,
                                                   const Eigen::Vector3d & finalVel,
                                                   const Eigen::Vector3d & finalAcc,
                                                   const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: SplineTrajectoryTask<ExactCubicTrajectoryTask>(frame, duration, stiffness, weight, target.rotation(), oriWp),
  bspline_(duration, frame.position().translation(), target.translation(), posWp, initVel, initAcc, finalVel, finalAcc)
{
  type_ = "exact_cubic_trajectory";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
  bspline_.constraints(initVel, initAcc, finalVel, finalAcc);
  initialPose_ = frame.position();
}

void ExactCubicTrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  SplineTrajectoryBase::addToGUI(gui);

  bspline_.addToGUI(gui, {"Tasks", name_});

  gui.addElement(
      {"Tasks", name_, "Velocity Constraints"},
      mc_rtc::gui::Arrow("Initial", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 1., 1.)),
                         [this]() -> Eigen::Vector3d { return initialPose_.translation(); },
                         [this]() -> Eigen::Vector3d { return initialPose_.translation() + bspline_.init_vel(); }),
      mc_rtc::gui::Arrow(
          "Final", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 1., 1.)),
          [this]() -> Eigen::Vector3d { return SplineTrajectoryBase::target().translation(); },
          [this]() -> Eigen::Vector3d { return SplineTrajectoryBase::target().translation() + bspline_.end_vel(); }));
}

} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "exact_cubic_trajectory",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      sva::PTransformd finalTarget_;
      std::vector<std::pair<double, Eigen::Vector3d>> waypoints;
      std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
      auto & robot = solver.robots().fromConfig(config, "ExactCubicTrajectoryTask");
      Eigen::Vector3d init_vel, end_vel, init_acc, end_acc;
      if(config.has("targetSurface"))
      { // Target defined from a target surface, with an offset defined
        // in the surface coordinates
        const auto & c = config("targetSurface");
        const auto & targetSurfaceName = c("surface");
        const auto & robot = solver.robots().fromConfig(c, "ExactCubicTrajectoryTask::targetSurface");

        const auto & targetSurface = robot.frame(targetSurfaceName).position();
        const Eigen::Vector3d trans = c("translation", Eigen::Vector3d::Zero().eval());
        const Eigen::Matrix3d rot = c("rotation", Eigen::Matrix3d::Identity().eval());
        sva::PTransformd offset(rot, trans);
        finalTarget_ = offset * targetSurface;

        if(c.has("controlPoints"))
        {
          // Control points offsets defined wrt to the target surface frame
          std::vector<std::pair<double, Eigen::Vector3d>> controlPoints = c("controlPoints");
          waypoints.resize(controlPoints.size());
          for(unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            const Eigen::Vector3d wp = controlPoints[i].second;
            sva::PTransformd X_offset(wp);
            waypoints[i].first = controlPoints[i].first;
            waypoints[i].second = (X_offset * targetSurface).translation();
          }
        }

        if(c.has("oriWaypoints"))
        {
          std::vector<std::pair<double, Eigen::Matrix3d>> oriWaypoints = c("oriWaypoints");
          for(const auto & wp : oriWaypoints)
          {
            const sva::PTransformd offset{wp.second};
            const sva::PTransformd ori = offset * targetSurface;
            oriWp.push_back(std::make_pair(wp.first, ori.rotation()));
          }
        }
        init_vel = targetSurface.rotation().inverse() * c("init_vel", Eigen::Vector3d::Zero().eval());
        init_acc = targetSurface.rotation().inverse() * c("init_acc", Eigen::Vector3d::Zero().eval());
        end_vel = targetSurface.rotation().inverse() * c("end_vel", Eigen::Vector3d::Zero().eval());
        end_acc = targetSurface.rotation().inverse() * c("end_acc", Eigen::Vector3d::Zero().eval());
      }
      else
      { // Absolute target pose
        finalTarget_ = config("target");

        if(config.has("controlPoints"))
        {
          // Control points defined in world coordinates
          std::vector<std::pair<double, Eigen::Vector3d>> controlPoints = config("controlPoints");
          waypoints.resize(controlPoints.size());
          for(unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            waypoints[i] = controlPoints[i];
          }
        }
        init_vel = config("init_vel", Eigen::Vector3d::Zero().eval());
        init_acc = config("init_acc", Eigen::Vector3d::Zero().eval());
        end_vel = config("end_vel", Eigen::Vector3d::Zero().eval());
        end_acc = config("end_acc", Eigen::Vector3d::Zero().eval());
        oriWp = config("oriWaypoints", std::vector<std::pair<double, Eigen::Matrix3d>>{});
      }

      std::shared_ptr<mc_tasks::ExactCubicTrajectoryTask> t = std::make_shared<mc_tasks::ExactCubicTrajectoryTask>(
          robot.frame(config("surface")), config("duration", 10.), config("stiffness", 100.), config("weight", 500.),
          finalTarget_, waypoints, init_vel, init_acc, end_vel, end_acc, oriWp);
      t->load(solver, config);
      const auto displaySamples = config("displaySamples", t->displaySamples());
      t->displaySamples(displaySamples);
      t->pause(config("paused", false));
      return t;
    });
}
