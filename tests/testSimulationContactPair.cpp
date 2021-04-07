/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/SimulationContactPair.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/config.h>

#include <boost/test/unit_test.hpp>

#include "utils.h"

mc_rbdyn::Robots & get_robots()
{
  static std::shared_ptr<mc_rbdyn::Robots> robots_ptr = nullptr;
  if(robots_ptr)
  {
    return *robots_ptr;
  }
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                     std::string("ground"));
  robots_ptr = std::make_shared<mc_rbdyn::Robots>();
  robots_ptr->load(*rm, rm->name);
  robots_ptr->load(*env, env->name);
  return *robots_ptr;
}

BOOST_AUTO_TEST_CASE(TestSimulationContactPair)
{
  auto & robots = get_robots();
  auto & robot = robots.robot();
  auto & env = robots.robot("ground");

  mc_control::SimulationContactPair pair(robot.surface("LeftFoot"), env.surface("AllGround"));

  // In its default configuration the robot's foot should be at the ground
  // level
  BOOST_REQUIRE_SMALL(pair.update(), 1e-6);
  // 1 m above the ground
  robot.mbc().q[0].back() += 1.0;
  robot.updateKinematics();
  BOOST_REQUIRE(pair.update() > 0.99);
  // ~1 mm below the ground
  robot.mbc().q[0].back() -= 1.0 + 1e-3;
  robot.updateKinematics();
  BOOST_REQUIRE(pair.update() <= 0);
}
