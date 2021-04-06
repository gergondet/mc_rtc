/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// FIXME Add CompoundJointConstraint

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_solver/CoMInConvexConstraint.h>
#include <mc_solver/CollisionsConstraint.h>
#include <mc_solver/ConstraintLoader.h>
#include <mc_solver/DynamicsConstraint.h>

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
struct ConstraintTester
{
  static_assert(fail<T>::value, "This should be specialized");
  mc_solver::ConstraintPtr make_ref()
  {
    return nullptr;
  }

  std::string json()
  {
    return "";
  }

  void check(const mc_solver::ConstraintPtr & /*ref*/, const mc_solver::ConstraintPtr & /*loaded*/) {}
};

template<>
struct ConstraintTester<mc_solver::BoundedSpeedConstr>
{
  ConstraintTester<mc_solver::BoundedSpeedConstr>()
  {
    for(int i = 0; i < 3; ++i)
    {
      if(lS(i) > 0)
      {
        lS(i) = -lS(i);
      }
      uS(i) = -lS(i);
    }
  }

  mc_solver::ConstraintPtr make_ref()
  {
    auto ret = std::make_shared<mc_solver::BoundedSpeedConstr>();
    ret->addBoundedSpeed(solver, robot.frame("R_WRIST_Y_S"), Eigen::Vector6d::Ones(), s);
    ret->addBoundedSpeed(solver, robot.frame("L_WRIST_Y_S"), Eigen::Vector6d::Ones(), lS, uS);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "boundedSpeed");
    config.add("robotIndex", 0);
    auto cs = config.array("constraints");
    {
      mc_rtc::Configuration c;
      c.add("body", "R_WRIST_Y_S");
      c.add("speed", s);
      cs.push(c);
    }
    {
      mc_rtc::Configuration c;
      c.add("body", "L_WRIST_Y_S");
      c.add("lowerSpeed", lS);
      c.add("upperSpeed", lS);
      cs.push(c);
    }
    {
      // No speed entry so shouldn't matter
      mc_rtc::Configuration c;
      c.add("body", "R_WRIST_Y_S");
      cs.push(c);
    }
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_solver::ConstraintPtr & ref_p, const mc_solver::ConstraintPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_solver::BoundedSpeedConstr>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_solver::BoundedSpeedConstr>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK(ref->nrBoundedSpeeds() == loaded->nrBoundedSpeeds());
    BOOST_CHECK(loaded->removeBoundedSpeed(solver, robot.frame("R_WRIST_Y_S")));
    BOOST_CHECK(loaded->removeBoundedSpeed(solver, robot.frame("L_WRIST_Y_S")));
  }

  Eigen::Vector6d s = Eigen::Vector6d::Random();
  Eigen::Vector6d lS = Eigen::Vector6d::Random();
  Eigen::Vector6d uS = Eigen::Vector6d::Zero();
};

template<>
struct ConstraintTester<mc_solver::CollisionsConstraint>
{
  mc_solver::ConstraintPtr make_ref()
  {
    auto ret = std::make_shared<mc_solver::CollisionsConstraint>();
    BOOST_REQUIRE(rm.commonSelfCollisions().size() > 0);
    ret->addCollisions(solver, {robot.name(), rm.commonSelfCollisions()});
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "collision");
    config.add("r1Index", 0);
    config.add("r2Index", 0);
    auto cv = config.array("collisions");
    for(const auto & c : rm.commonSelfCollisions())
    {
      cv.push(c);
    }
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_solver::ConstraintPtr & ref_p, const mc_solver::ConstraintPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_solver::CollisionsConstraint>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_solver::CollisionsConstraint>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK(ref->collisions() == loaded->collisions());
  }
};

template<>
struct ConstraintTester<mc_solver::CoMInConvexConstraint>
{
  mc_solver::ConstraintPtr make_ref()
  {
    return std::make_shared<mc_solver::CoMInConvexConstraint>(robot);
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "CoMInConvex");
    config.add("robotIndex", 0);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_solver::ConstraintPtr & ref_p, const mc_solver::ConstraintPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_solver::CoMInConvexConstraint>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_solver::CoMInConvexConstraint>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
  }
};

template<>
struct ConstraintTester<mc_solver::KinematicsConstraint>
{
  mc_solver::ConstraintPtr make_ref()
  {
    return std::make_shared<mc_solver::KinematicsConstraint>(robot, std::array<double, 3>{0.1, 0.01, 0.5});
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "kinematics");
    config.add("robotIndex", 0);
    config.add("damper", std::array<double, 3>{{0.1, 0.01, 0.5}});
    config.add("velocityPercent", 0.5);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_solver::ConstraintPtr & ref_p, const mc_solver::ConstraintPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_solver::KinematicsConstraint>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_solver::KinematicsConstraint>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
  }
};

template<>
struct ConstraintTester<mc_solver::DynamicsConstraint>
{
  mc_solver::ConstraintPtr make_ref()
  {
    return std::make_shared<mc_solver::DynamicsConstraint>(robot, std::array<double, 3>{0.1, 0.01, 0.5});
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "dynamics");
    config.add("robotIndex", 0);
    config.add("damper", std::array<double, 3>{{0.1, 0.01, 0.5}});
    config.add("velocityPercent", 0.5);
    config.add("infTorque", true);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_solver::ConstraintPtr & ref_p, const mc_solver::ConstraintPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_solver::DynamicsConstraint>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_solver::DynamicsConstraint>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
  }
};

typedef boost::mpl::list<mc_solver::BoundedSpeedConstr,
                         mc_solver::CollisionsConstraint,
                         mc_solver::CoMInConvexConstraint,
                         mc_solver::KinematicsConstraint,
                         mc_solver::DynamicsConstraint>
    test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE(TestConstraintLoader, T, test_types)
{
  auto tester = ConstraintTester<T>();
  auto ref = tester.make_ref();
  auto conf = tester.json();
  auto loaded = mc_solver::ConstraintLoader::load(solver, conf);
  tester.check(ref, loaded);
}
