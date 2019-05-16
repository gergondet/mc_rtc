#include <mc_tasks/ExactCubicTrajectoryTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_trajectory/ExactCubic.h>
#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_tasks
{

ExactCubicTrajectoryTask::ExactCubicTrajectoryTask(const mc_rbdyn::Robots & robots,
                                                   unsigned int robotIndex,
                                                   const std::string & surfaceName,
                                                   double duration,
                                                   double stiffness,
                                                   double posW,
                                                   double oriW,
                                                   const sva::PTransformd & target,
                                                   const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                                                   const Eigen::Vector3d & init_vel,
                                                   const Eigen::Vector3d & init_acc,
                                                   const Eigen::Vector3d & end_vel,
                                                   const Eigen::Vector3d & end_acc,
                                                   const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: TrajectoryTask(robots, robotIndex, surfaceName, duration, stiffness, posW, oriW)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  type_ = "exact_cubic_trajectory";
  name_ = "exact_cubic_trajectory_" + robot.name() + "_" + surfaceName;
  this->target(target);
  posWaypoints(posWp, init_vel, init_acc, end_vel, end_acc);
  oriWaypoints(oriWp);
}

void ExactCubicTrajectoryTask::posWaypoints(const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                                            const Eigen::Vector3d & init_vel,
                                            const Eigen::Vector3d & init_acc,
                                            const Eigen::Vector3d & end_vel,
                                            const Eigen::Vector3d & end_acc)
{
  std::vector<std::pair<double, Eigen::Vector3d>> waypoints;
  waypoints.reserve(posWp.size() + 2);
  const auto & robot = robots.robot(rIndex);
  const auto & X_0_s = robot.surface(surfaceName).X_0_s(robot);
  waypoints.push_back(std::make_pair(0., X_0_s.translation()));
  for(const auto & wp : posWp)
  {
    waypoints.push_back(wp);
  }
  waypoints.push_back(std::make_pair(duration, X_0_t.translation()));
  bspline.reset(new mc_trajectory::ExactCubic(waypoints, init_vel, init_acc, end_vel, end_acc));
}

void ExactCubicTrajectoryTask::oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
{
  const auto & robot = robots.robot(rIndex);
  const auto & X_0_s = robot.surface(surfaceName).X_0_s(robot);
  oriWp_.push_back(std::make_pair(0., X_0_s.rotation()));
  for(const auto & wp : oriWp)
  {
    oriWp_.push_back(wp);
  }
  oriWp_.push_back(std::make_pair(duration, X_0_t.rotation()));
  orientation_spline.reset(new mc_trajectory::InterpolatedRotation(oriWp_));
}

void ExactCubicTrajectoryTask::update()
{
  // Interpolate position
  auto res = bspline->splev({t}, 2);
  Eigen::Vector3d & pos = res[0][0];
  Eigen::Vector3d & vel = res[0][1];
  Eigen::Vector3d & acc = res[0][2];

  // Interpolate orientation
  Eigen::Matrix3d ori_target = orientation_spline->eval(t);
  sva::PTransformd target(ori_target, pos);

  // Set the trajectory tracking task targets from the trajectory.
  Eigen::VectorXd refVel(6);
  Eigen::VectorXd refAcc(6);
  refVel.head<3>() = Eigen::Vector3d::Zero();
  refVel.tail<3>() = vel;
  refAcc.head<3>() = Eigen::Vector3d::Zero();
  refAcc.tail<3>() = acc;
  this->refVel(refVel);
  this->refAcc(refAcc);
  this->refPose(target);

  TrajectoryTask::update();
}

void ExactCubicTrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTask::addToGUI(gui);
  bspline->addToGUI(gui, {"Tasks", name_});

  gui.addElement({"Tasks", name_, "Target"}, mc_rtc::gui::Transform("target", [this]() { return target(); },
                                                                    [this](const sva::PTransformd & t) {
                                                                      target(t);
                                                                      // XXX inefficient
                                                                      auto waypoints = bspline->waypoints();
                                                                      waypoints.back().second = t.translation();
                                                                      bspline->waypoints(waypoints);
                                                                      orientation_spline->waypoints().back().second =
                                                                          t.rotation();
                                                                    }));
  gui.addElement(
      {"Tasks", name_, "Velocity Constraints"},
      mc_rtc::gui::Arrow("Initial", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 1., 1.)),
                         [this]() -> Eigen::Vector3d { return X_0_start.translation(); },
                         [this]() -> Eigen::Vector3d { return X_0_start.translation() + bspline->init_vel(); }),
      mc_rtc::gui::Arrow("Final", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 1., 1.)),
                         [this]() -> Eigen::Vector3d { return target().translation(); },
                         [this]() -> Eigen::Vector3d { return target().translation() + bspline->end_vel(); }));

  // XXX different style for rotation element
  for(unsigned i = 1; i < orientation_spline->waypoints().size() - 1; ++i)
  {
    gui.addElement({"Tasks", name_, "Orientation Control Points"},
                   mc_rtc::gui::Rotation("control_point_ori_" + std::to_string(i),
                                         [this, i]() {
                                           const auto & wp = orientation_spline->waypoints()[i];

                                           // Get position of orientation waypoint along the spline
                                           const auto & res = bspline->splev({wp.first}, 2);
                                           const Eigen::Vector3d pos = res[0][0];
                                           return sva::PTransformd(wp.second, pos);
                                         },
                                         [this, i](const Eigen::Quaterniond & ori) {
                                           auto & wp = orientation_spline->waypoints()[i];
                                           wp.second = ori.toRotationMatrix();
                                         }));
  }
}

void ExactCubicTrajectoryTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTask::removeFromGUI(gui);
  gui.removeCategory({"Tasks", name_, "Orientation Control Points"});
  gui.removeCategory({"Tasks", name_, "Position Control Points"});
}

} // namespace mc_tasks

namespace
{
static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "exact_cubic_trajectory",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      sva::PTransformd X_0_t;
      std::vector<std::pair<double, Eigen::Vector3d>> waypoints;
      std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
      const auto robotIndex = config("robotIndex");
      Eigen::Vector3d init_vel, end_vel, init_acc, end_acc;

      if(config.has("targetSurface"))
      { // Target defined from a target surface, with an offset defined
        // in the surface coordinates
        const auto & c = config("targetSurface");
        const auto & targetSurfaceName = c("surface");
        const auto & robot = solver.robot(c("robotIndex"));

        const sva::PTransformd & targetSurface = robot.surface(targetSurfaceName).X_0_s(robot);
        const Eigen::Vector3d trans = c("offset_translation", Eigen::Vector3d::Zero().eval());
        const Eigen::Matrix3d rot = c("offset_rotation", Eigen::Matrix3d::Identity().eval());
        sva::PTransformd offset(rot, trans);
        X_0_t = offset * targetSurface;

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
        X_0_t = config("target");

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
          solver.robots(), robotIndex, config("surface"), config("duration"), config("stiffness"), config("posWeight"),
          config("oriWeight"), X_0_t, waypoints, init_vel, init_acc, end_vel, end_acc, oriWp);
      t->load(solver, config);
      const auto displaySamples = config("displaySamples", t->displaySamples());
      t->displaySamples(displaySamples);
      return t;
    });
}
