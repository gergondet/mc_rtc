/* Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL  */

#include <mc_tvm/FrameErrorFunction.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <tvm/utils/checkFunction.h>

#include <boost/test/unit_test.hpp>

using namespace Eigen;
using namespace rbd;
using namespace sva;

std::tuple<MultiBody, MultiBodyGraph, MultiBodyConfig> XYZRobot()
{
  Body b0(1, Vector3d::Zero(), Matrix3d::Identity(), "b0");
  Body b1(1, Vector3d::UnitX() / 2, Matrix3d::Identity(), "b1");
  Body b2(1, Vector3d::UnitX() / 2, Matrix3d::Identity(), "b2");
  Body b3(1, Vector3d::UnitX() / 2, Matrix3d::Identity(), "b3");
  Body b4(1, Vector3d::UnitX() / 2, Matrix3d::Identity(), "b4");

  Joint j1(Joint::Rev, Vector3d::UnitX(), true, "j1");
  Joint j2(Joint::Rev, Vector3d::UnitY(), true, "j2");
  Joint j3(Joint::Rev, Vector3d::UnitZ(), true, "j3");
  Joint j4(Joint::Fixed, Vector3d::UnitX(), true, "j4");

  MultiBodyGraph mbg;
  mbg.addBody(b0);
  mbg.addBody(b1);
  mbg.addBody(b2);
  mbg.addBody(b3);
  mbg.addBody(b4);
  mbg.addJoint(j1);
  mbg.addJoint(j2);
  mbg.addJoint(j3);
  mbg.addJoint(j4);
  mbg.linkBodies("b0", sva::PTransformd(Vector3d(0, 0, 0)), "b1", sva::PTransformd::Identity(), "j1");
  mbg.linkBodies("b1", sva::PTransformd(Vector3d(0, 1, 0)), "b2", sva::PTransformd::Identity(), "j2");
  mbg.linkBodies("b2", sva::PTransformd(Vector3d(0, 1, 0)), "b3", sva::PTransformd::Identity(), "j3");
  mbg.linkBodies("b3", sva::PTransformd(Vector3d(0, 1, 0)), "b4", sva::PTransformd::Identity(), "j4");

  auto mb = mbg.makeMultiBody("b0", true);
  MultiBodyConfig mbc(mb);
  mbc.q = {{}, {0}, {0}, {0}, {}};
  return std::make_tuple(mb, mbg, mbc);
}

BOOST_AUTO_TEST_CASE(FrameErrorFunctionTest)
{
  mc_rbdyn::Robots robots;
  auto module = mc_rbdyn::RobotLoader::get_robot_module("env/ground");
  auto & world = robots.load(*module, "world");
  auto & f0 = world.makeFrame("fixed", "ground", sva::PTransformd(Quaterniond::UnitRandom(), Vector3d::Random()));

  rbd::parsers::ParserResult pr1;
  std::tie(pr1.mb, pr1.mbg, pr1.mbc) = XYZRobot();
  rbd::parsers::ParserResult pr2;
  std::tie(pr2.mb, pr2.mbg, pr2.mbc) = XYZRobot();

  auto & xyz1 = robots.load({"XYZ1", pr1}, "XYZ1");
  auto & f1 = xyz1.makeFrame("ee1", "b4", sva::PTransformd(Quaterniond::UnitRandom(), Vector3d::Random()));

  auto & xyz2 = robots.load({"XYZ2", pr2}, "XYZ2", PTransformd(Matrix3d::Identity(), 2 * Vector3d::UnitX()));
  auto & f2 = xyz2.makeFrame("ee2", "b4", sva::PTransformd(Quaterniond::UnitRandom(), Vector3d::Random()));

  {
    auto e = std::make_shared<mc_tvm::FrameErrorFunction>(f1, f0);
    BOOST_CHECK(tvm::utils::checkFunction(e, tvm::utils::CheckOptions(1e-7, 5e-6, true)));
  }
  {
    auto e = std::make_shared<mc_tvm::FrameErrorFunction>(f0, f2);
    BOOST_CHECK(tvm::utils::checkFunction(e, tvm::utils::CheckOptions(1e-7, 5e-6, true)));
  }
  {
    auto e = std::make_shared<mc_tvm::FrameErrorFunction>(f1, f2);
    BOOST_CHECK(tvm::utils::checkFunction(e, tvm::utils::CheckOptions(1e-7, 5e-6, true)));
  }
}