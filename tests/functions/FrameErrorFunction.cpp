/* Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL  */

#include <mc_tvm/FrameErrorFunction.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <tvm/utils/checkFunction.h>

#include <boost/test/unit_test.hpp>

#include "utils.h"

std::tuple<rbd::MultiBody, rbd::MultiBodyGraph, rbd::MultiBodyConfig> XYZRobot()
{
  rbd::Body b0(1, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), "b0");
  rbd::Body b1(1, Eigen::Vector3d::UnitX() / 2, Eigen::Matrix3d::Identity(), "b1");
  rbd::Body b2(1, Eigen::Vector3d::UnitX() / 2, Eigen::Matrix3d::Identity(), "b2");
  rbd::Body b3(1, Eigen::Vector3d::UnitX() / 2, Eigen::Matrix3d::Identity(), "b3");
  rbd::Body b4(1, Eigen::Vector3d::UnitX() / 2, Eigen::Matrix3d::Identity(), "b4");

  rbd::Joint j1(rbd::Joint::Rev, Eigen::Vector3d::UnitX(), true, "j1");
  rbd::Joint j2(rbd::Joint::Rev, Eigen::Vector3d::UnitY(), true, "j2");
  rbd::Joint j3(rbd::Joint::Rev, Eigen::Vector3d::UnitZ(), true, "j3");
  rbd::Joint j4(rbd::Joint::Fixed, Eigen::Vector3d::UnitX(), true, "j4");

  rbd::MultiBodyGraph mbg;
  mbg.addBody(b0);
  mbg.addBody(b1);
  mbg.addBody(b2);
  mbg.addBody(b3);
  mbg.addBody(b4);
  mbg.addJoint(j1);
  mbg.addJoint(j2);
  mbg.addJoint(j3);
  mbg.addJoint(j4);
  mbg.linkBodies("b0", sva::PTransformd(Eigen::Vector3d(0, 0, 0)), "b1", sva::PTransformd::Identity(), "j1");
  mbg.linkBodies("b1", sva::PTransformd(Eigen::Vector3d(0, 1, 0)), "b2", sva::PTransformd::Identity(), "j2");
  mbg.linkBodies("b2", sva::PTransformd(Eigen::Vector3d(0, 1, 0)), "b3", sva::PTransformd::Identity(), "j3");
  mbg.linkBodies("b3", sva::PTransformd(Eigen::Vector3d(0, 1, 0)), "b4", sva::PTransformd::Identity(), "j4");

  auto mb = mbg.makeMultiBody("b0", true);
  rbd::MultiBodyConfig mbc(mb);
  mbc.q = {{}, {0}, {0}, {0}, {}};
  return std::make_tuple(mb, mbg, mbc);
}

Eigen::Vector6d toDof(int v)
{
  Eigen::Vector6d ret;
  for(int i = 0; i < 6; ++i)
  {
    ret[i] = v % 2;
    v /= 2;
  }
  return ret;
}

BOOST_AUTO_TEST_CASE(FrameErrorFunctionTest)
{
  // We are repeatedly creating new function so that the log becomes large. We deactivate it .
  tvm::graph::internal::Logger::logger().disable();

  configureRobotLoader();
  mc_rbdyn::Robots robots;
  auto module = mc_rbdyn::RobotLoader::get_robot_module("env/ground");
  auto & world = robots.load(*module, "world");
  auto & f0 =
      world.makeFrame("fixed", "ground", sva::PTransformd(Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random()));

  rbd::parsers::ParserResult pr1;
  std::tie(pr1.mb, pr1.mbg, pr1.mbc) = XYZRobot();
  rbd::parsers::ParserResult pr2;
  std::tie(pr2.mb, pr2.mbg, pr2.mbc) = XYZRobot();

  auto & xyz1 = robots.load({"XYZ1", pr1}, "XYZ1");
  auto & f1 =
      xyz1.makeFrame("ee1", "b4", sva::PTransformd(Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random()));

  auto & xyz2 =
      robots.load({"XYZ2", pr2}, "XYZ2", sva::PTransformd(Eigen::Matrix3d::Identity(), 2 * Eigen::Vector3d::UnitX()));
  auto & f2 =
      xyz2.makeFrame("ee2", "b4", sva::PTransformd(Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random()));

  // These tests are using the derivatives of the log on SO(3) whose implementation in sva is known to
  // become less precise when the rotation between two frames gets close to pi.
  // For this reason, we ignore the failures when the angle is nearing pi (which we evaluate with the
  // trace of the error rotation becoming close to -1).
  for(int i = 0; i < 64; ++i)
  {
    for(int k = 0; k < 10; ++k)
    {
      auto dof = toDof(i);
      {
        // Only left frame depends on a variable
        auto e = std::make_shared<mc_tvm::FrameErrorFunction>(f1, f0, dof);
        bool b = tvm::utils::checkFunction(e, tvm::utils::CheckOptions(1e-7, 1e-4, true));
        bool c = (f0.position().rotation() * f1.position().rotation().transpose()).trace() < -0.95;
        BOOST_CHECK((b || c));
      }
      {
        // Only right frame depends on a variable
        auto e = std::make_shared<mc_tvm::FrameErrorFunction>(f0, f2, dof);
        bool b = tvm::utils::checkFunction(e, tvm::utils::CheckOptions(1e-7, 1e-4, true));
        bool c = (f2.position().rotation() * f0.position().rotation().transpose()).trace() < -0.95;
        BOOST_CHECK((b || c));
      }
      {
        // Both frames depend on a variable
        auto e = std::make_shared<mc_tvm::FrameErrorFunction>(f1, f2, dof);
        bool b = tvm::utils::checkFunction(e, tvm::utils::CheckOptions(1e-7, 1e-4, true));
        bool c = (f2.position().rotation() * f1.position().rotation().transpose()).trace() < -0.95;
        BOOST_CHECK((b || c));
      }
    }
  }
}