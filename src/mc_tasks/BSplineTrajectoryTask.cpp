/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/BSplineTrajectoryTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_trajectory/BSpline.h>
#include <mc_trajectory/InterpolatedRotation.h>

#include <mc_rbdyn/Surface.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

using BSpline = mc_trajectory::BSpline;

BSplineTrajectoryTask::BSplineTrajectoryTask(mc_rbdyn::Frame & frame,
                                             double duration,
                                             double stiffness,
                                             double weight,
                                             const sva::PTransformd & target,
                                             const waypoints_t & posWp,
                                             const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: SplineTrajectoryTask<BSplineTrajectoryTask>(frame, duration, stiffness, weight, target.rotation(), oriWp),
  bspline_(duration, frame.position().translation(), target.translation(), posWp)
{
  type_ = "bspline_trajectory";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
}

void BSplineTrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  SplineTrajectoryBase::addToGUI(gui);
  bspline_.addToGUI(gui, {"Tasks", name_});
}

} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "bspline_trajectory",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & configIn) {
      auto config = configIn;
      sva::PTransformd finalTarget_;
      mc_tasks::BSplineTrajectoryTask::waypoints_t waypoints;
      std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
      auto & robot = solver.robots().fromConfig(config, "BSplineTrajectoryTask");
      if(config.has("targetSurface"))
      {
        mc_rtc::log::deprecated("BSplineTrajectoryTask", "targetSurface", "targetFrame");
        config.add("targetFrame", config("targetSurface"));
        config("targetFrame").add("frame", config("targetSurface")("surface"));
      }
      if(config.has("targetFrame"))
      { // Target defined from a target frame, with an offset defined
        // in the frame coordinates
        const auto & c = config("targetFrame");
        const auto & targetSurfaceName = c("frame");
        const auto & robot = solver.robots().fromConfig(c, "BSplineTrajectoryTask::targetFrame");

        const auto & targetSurface = robot.frame(targetSurfaceName).position();
        const Eigen::Vector3d trans = c("translation", Eigen::Vector3d::Zero().eval());
        const Eigen::Matrix3d rot = c("rotation", Eigen::Matrix3d::Identity().eval());
        sva::PTransformd offset(rot, trans);
        finalTarget_ = offset * targetSurface;

        if(c.has("controlPoints"))
        {
          // Control points offsets defined wrt to the target surface frame
          const auto & controlPoints = c("controlPoints");
          waypoints.resize(controlPoints.size());
          for(unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            const Eigen::Vector3d wp = controlPoints[i];
            sva::PTransformd X_offset(wp);
            waypoints[i] = (X_offset * targetSurface).translation();
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
      }
      else
      { // Absolute target pose
        finalTarget_ = config("target");

        if(config.has("controlPoints"))
        {
          // Control points defined in world coordinates
          const auto & controlPoints = config("controlPoints");
          waypoints.resize(controlPoints.size());
          for(unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            const Eigen::Vector3d wp = controlPoints[i];
            waypoints[i] = wp;
          }
        }

        oriWp = config("oriWaypoints", std::vector<std::pair<double, Eigen::Matrix3d>>{});
      }

      if(config.has("surface"))
      {
        mc_rtc::log::deprecated("BSplineTrajectoryTask", "surface", "frame");
        config.add("frame", config("surface"));
      }
      auto t = std::make_shared<mc_tasks::BSplineTrajectoryTask>(robot.frame(config("frame")), config("duration", 10.),
                                                                 config("stiffness", 100.), config("weight", 500.),
                                                                 finalTarget_, waypoints, oriWp);
      t->load(solver, config);
      const auto displaySamples = config("displaySamples", t->displaySamples());
      t->displaySamples(displaySamples);
      t->pause(config("paused", false));
      return t;
    });
}
