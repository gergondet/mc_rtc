/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/ExactCubicTrajectoryTask.h>
#include <mc_tasks/GazeTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/PositionBasedVisServoTask.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/VectorOrientationTask.h>

// FIXME Missing StabilizerTask and most force tasks

#include <boost/mpl/list.hpp>
#include <boost/test/unit_test.hpp>

#include "utils.h"

static auto solver_ptr = makeSolver();
static auto & solver = *solver_ptr;
static auto & robots = solver.robots();
static auto & robot = robots.robot();
static auto & rm = robot.module();

template<typename T>
struct fail : public std::false_type
{
};

template<typename T>
struct TaskTester
{
  static_assert(fail<T>::value, "This should be specialized");
  mc_tasks::MetaTaskPtr make_ref()
  {
    return nullptr;
  }

  std::string json()
  {
    return "";
  }

  void check(const mc_tasks::MetaTaskPtr & /*ref*/, const mc_tasks::MetaTaskPtr & /*loaded*/) {}
};

template<>
struct TaskTester<mc_tasks::CoMTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::CoMTask>(robot, stiffness, weight);
    ret->com(com);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "com");
    config.add("robotIndex", 0);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("com", com);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::CoMTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::CoMTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->com().isApprox(loaded->com(), 1e-9));
  }

  Eigen::Vector3d com = Eigen::Vector3d::Random();
  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
};

#define AddRemoveContactTaskTester(T, IsAddContactTask, typeStr)                                                     \
  struct T : public mc_tasks::AddRemoveContactTask                                                                   \
  {                                                                                                                  \
    using AddRemoveContactTask::AddRemoveContactTask;                                                                \
  };                                                                                                                 \
  template<>                                                                                                         \
  struct TaskTester<T>                                                                                               \
  {                                                                                                                  \
    mc_tasks::MetaTaskPtr make_ref()                                                                                 \
    {                                                                                                                \
      auto ret = std::make_shared<T>(robot.frame("LeftFoot"), IsAddContactTask ? -speed : speed, stiffness, weight); \
      return ret;                                                                                                    \
    }                                                                                                                \
                                                                                                                     \
    std::string json()                                                                                               \
    {                                                                                                                \
      mc_rtc::Configuration config;                                                                                  \
      config.add("type", typeStr);                                                                                   \
      config.add("contact", contact);                                                                                \
      config.add("stiffness", stiffness);                                                                            \
      config.add("weight", weight);                                                                                  \
      config.add("speed", speed);                                                                                    \
      auto ret = getTmpFile();                                                                                       \
      config.save(ret);                                                                                              \
      return ret;                                                                                                    \
    }                                                                                                                \
                                                                                                                     \
    void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)                          \
    {                                                                                                                \
      auto ref = std::dynamic_pointer_cast<mc_tasks::AddRemoveContactTask>(ref_p);                                   \
      auto loaded = std::dynamic_pointer_cast<mc_tasks::AddRemoveContactTask>(loaded_p);                             \
      BOOST_REQUIRE(ref);                                                                                            \
      BOOST_REQUIRE(loaded);                                                                                         \
      BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);                                                \
      BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);                                                      \
      BOOST_CHECK_CLOSE(ref->desiredSpeed(), loaded->desiredSpeed(), 1e-6);                                          \
      BOOST_CHECK(ref->frame().name() == loaded->frame().name());                                                    \
    }                                                                                                                \
                                                                                                                     \
    mc_rbdyn::Contact contact = mc_rbdyn::Contact(robot.name(), "ground", "LeftFoot", "AllGround");                  \
    double speed = fabs(rnd());                                                                                      \
    double stiffness = fabs(rnd());                                                                                  \
    double weight = fabs(rnd());                                                                                     \
  }
AddRemoveContactTaskTester(AddContactTask, true, "addContact");
AddRemoveContactTaskTester(RemoveContactTask, false, "removeContact");

template<>
struct TaskTester<mc_tasks::force::ComplianceTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::shared_ptr<mc_tasks::force::ComplianceTask>(new mc_tasks::force::ComplianceTask(
        robot.frame("R_WRIST_Y_S"), dof, stiffness, weight, forceThresh, torqueThresh, forceGain, torqueGain));
    t->setTargetWrench(wrench);
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "compliance");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
    config.add("dof", dof);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("forceThresh", forceThresh);
    config.add("torqueThresh", torqueThresh);
    config.add("forceGain", forceGain);
    config.add("torqueGain", torqueGain);
    config.add("wrench", wrench);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::force::ComplianceTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::force::ComplianceTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK(ref->dof().isApprox(loaded->dof(), 1e-6));
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK_CLOSE(ref->forceThreshold(), loaded->forceThreshold(), 1e-6);
    BOOST_CHECK_CLOSE(ref->torqueThreshold(), loaded->torqueThreshold(), 1e-6);
    BOOST_CHECK_CLOSE(ref->forceGain().first, loaded->forceGain().first, 1e-6);
    BOOST_CHECK_CLOSE(ref->forceGain().second, loaded->forceGain().second, 1e-6);
    BOOST_CHECK_CLOSE(ref->torqueGain().first, loaded->torqueGain().first, 1e-6);
    BOOST_CHECK_CLOSE(ref->torqueGain().second, loaded->torqueGain().second, 1e-6);
    BOOST_CHECK(ref->getTargetWrench().vector().isApprox(loaded->getTargetWrench().vector()));
  }

  Eigen::Vector6d dof = random_dof();
  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  double forceThresh = fabs(rnd());
  double torqueThresh = fabs(rnd());
  std::pair<double, double> forceGain = {fabs(rnd()), fabs(rnd())};
  std::pair<double, double> torqueGain = {fabs(rnd()), fabs(rnd())};
  sva::ForceVecd wrench = {Eigen::Vector6d::Random()};
};

template<>
struct TaskTester<mc_tasks::OrientationTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::make_shared<mc_tasks::OrientationTask>(robot.frame("R_WRIST_Y_S"), stiffness, weight);
    t->orientation(ori);
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "orientation");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("orientation", ori);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::OrientationTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::OrientationTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->orientation().isApprox(loaded->orientation(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  Eigen::Matrix3d ori = Eigen::Matrix3d::Random();
};

template<>
struct TaskTester<mc_tasks::PositionTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::make_shared<mc_tasks::PositionTask>(robot.frame("R_WRIST_Y_S"), stiffness, weight);
    t->position(pos);
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "position");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("position", pos);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::PositionTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::PositionTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->position().isApprox(loaded->position(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  Eigen::Vector3d pos = Eigen::Vector3d::Random();
};

template<>
struct TaskTester<mc_tasks::TransformTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::make_shared<mc_tasks::TransformTask>(robot.frame("R_WRIST_Y_S"), stiffness, weight);
    t->target({ori, pos});
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "body6d");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("position", pos);
    config.add("orientation", ori);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::TransformTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::TransformTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->target().rotation().isApprox(loaded->target().rotation(), 1e-6));
    BOOST_CHECK(ref->target().translation().isApprox(loaded->target().translation(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  Eigen::Matrix3d ori = Eigen::Matrix3d::Random();
  Eigen::Vector3d pos = Eigen::Vector3d::Random();
};

template<>
struct TaskTester<mc_tasks::GazeTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::GazeTask>(robot.frame("NECK_P_S"), stiffness, weight);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "gaze");
    config.add("robotIndex", 0);
    config.add("frame", "NECK_P_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::GazeTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::GazeTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
};

template<>
struct TaskTester<mc_tasks::PositionBasedVisServoTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::PositionBasedVisServoTask>(robot.frame("NECK_P_S"), stiffness, weight);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "pbvs");
    config.add("robotIndex", 0);
    config.add("frame", "NECK_P_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::PositionBasedVisServoTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::PositionBasedVisServoTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  sva::PTransformd X_b_s = random_pt();
};

template<>
struct TaskTester<mc_tasks::VectorOrientationTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret =
        std::make_shared<mc_tasks::VectorOrientationTask>(robot.frame("R_WRIST_Y_S"), frameVector, stiffness, weight);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "vectorOrientation");
    config.add("body", "R_WRIST_Y_S");
    config.add("frameVector", frameVector);
    config.add("targetVector", targetVector);
    config.add("robotIndex", 0);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::VectorOrientationTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::VectorOrientationTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->frame().name() == loaded->frame().name());
    BOOST_CHECK(ref->frameVector().isApprox(loaded->frameVector()));
  }

  Eigen::Vector3d frameVector = Eigen::Vector3d::Random();
  Eigen::Vector3d targetVector = Eigen::Vector3d::Random();
  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
};

template<>
struct TaskTester<mc_tasks::PostureTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::PostureTask>(robot, stiffness, weight);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "posture");
    config.add("robotIndex", 0);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::PostureTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::PostureTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
};

template<>
struct TaskTester<mc_tasks::BSplineTrajectoryTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::BSplineTrajectoryTask>(robot.frame("LeftFoot"), d, stiffness, weight, target);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "bspline_trajectory");
    config.add("robotIndex", 0);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("duration", d);
    config.add("surface", "LeftFoot");
    config.add("target", target);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::BSplineTrajectoryTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::BSplineTrajectoryTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK_CLOSE(ref->duration(), loaded->duration(), 1e-6);
    BOOST_CHECK(ref->target().rotation().isApprox(loaded->target().rotation(), 1e-6));
    BOOST_CHECK(ref->target().translation().isApprox(loaded->target().translation(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  double d = fabs(rnd());
  sva::PTransformd target = random_pt();
};

template<>
struct TaskTester<mc_tasks::ExactCubicTrajectoryTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret =
        std::make_shared<mc_tasks::ExactCubicTrajectoryTask>(robot.frame("LeftFoot"), d, stiffness, weight, target);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "exact_cubic_trajectory");
    config.add("robotIndex", 0);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("duration", d);
    config.add("surface", "LeftFoot");
    config.add("target", target);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::ExactCubicTrajectoryTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::ExactCubicTrajectoryTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK_CLOSE(ref->duration(), loaded->duration(), 1e-6);
    BOOST_CHECK(ref->target().rotation().isApprox(loaded->target().rotation(), 1e-6));
    BOOST_CHECK(ref->target().translation().isApprox(loaded->target().translation(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  double d = fabs(rnd());
  sva::PTransformd target = random_pt();
};

typedef boost::mpl::list<mc_tasks::CoMTask,
                         AddContactTask,
                         RemoveContactTask,
                         mc_tasks::force::ComplianceTask,
                         mc_tasks::OrientationTask,
                         mc_tasks::PositionTask,
                         mc_tasks::GazeTask,
                         mc_tasks::PositionBasedVisServoTask,
                         mc_tasks::TransformTask,
                         mc_tasks::VectorOrientationTask,
                         mc_tasks::PostureTask,
                         mc_tasks::BSplineTrajectoryTask,
                         mc_tasks::ExactCubicTrajectoryTask>
    test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE(TestMetaTaskLoader, T, test_types)
{
  auto tester = TaskTester<T>();
  auto ref = tester.make_ref();
  auto conf = tester.json();
  auto loaded = mc_tasks::MetaTaskLoader::load(solver, conf);
  tester.check(ref, loaded);
}
