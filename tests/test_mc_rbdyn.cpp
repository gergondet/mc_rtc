#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/rpy_utils.h>
#include <boost/test/unit_test.hpp>
#include "utils.h"
#include <chrono>
#include <random>

BOOST_AUTO_TEST_CASE(TestRobotLoading)
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto envrm = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                       std::string("ground"));
  // Non-unique names
  mc_rbdyn::Robots robots;
  BOOST_REQUIRE_NO_THROW(robots.load(*rm, rm->name));
  BOOST_REQUIRE_NO_THROW(robots.load(*envrm, envrm->name));
  BOOST_REQUIRE_THROW(robots.load(*rm, rm->name), std::runtime_error);
  BOOST_REQUIRE(robots.hasRobot(rm->name));
  BOOST_REQUIRE(robots.hasRobot(envrm->name));
  auto & robot = robots.robot(rm->name);
  auto & env = robots.robot(envrm->name);
  BOOST_REQUIRE_EQUAL(robot.name(), rm->name);
  BOOST_REQUIRE_EQUAL(env.name(), envrm->name);

  BOOST_REQUIRE_NO_THROW(robots.robotCopy(robot, "robotCopy"));
  BOOST_REQUIRE(robots.hasRobot("robotCopy"));
  auto & robotCopy = *robots.robots().back();
  BOOST_REQUIRE(robotCopy.name() != rm->name);
  BOOST_REQUIRE_EQUAL(robots.robot("robotCopy").name(), "robotCopy");

  robots.removeRobot("robotCopy");
  BOOST_REQUIRE(!robots.hasRobot("robotCopy"));
  BOOST_REQUIRE(robots.hasRobot(rm->name));
  BOOST_REQUIRE(robots.hasRobot(envrm->name));
}

BOOST_AUTO_TEST_CASE(TestRobotPosWVelWAccW)
{
  auto robots_ptr = makeRobots();
  auto & robots = *robots_ptr;

  for(int i = 0; i < 100; ++i)
  {
    Eigen::Vector3d oriRPYRef = Eigen::Vector3d::Random();
    Eigen::Vector3d posRPYRef = Eigen::Vector3d::Random();
    sva::PTransformd refPosW(mc_rbdyn::rpyToMat(oriRPYRef), posRPYRef);
    robots.robot().posW(refPosW);
    BOOST_CHECK(robots.robot().posW().matrix().isApprox(refPosW.matrix()));
  }

  auto checkVelocity = [](const sva::MotionVecd & actual, const sva::MotionVecd & refVal) {
    BOOST_CHECK_MESSAGE(actual.vector().isApprox(refVal.vector()), "Error in Robot::velW"
                                                                       << "\nExpected:"
                                                                       << "\nangular:" << refVal.angular().transpose()
                                                                       << "\nlinear :" << refVal.linear().transpose()
                                                                       << "\nGot:"
                                                                       << "\nangular:" << actual.angular().transpose()
                                                                       << "\nlinear :" << actual.linear().transpose());
  };

  for(int i = 0; i < 100; ++i)
  {
    auto refVal = sva::MotionVecd{Eigen::Vector3d::Random(), Eigen::Vector3d::Random()};
    robots.robot().velW(refVal);
    robots.robot().accW(refVal);
    checkVelocity(robots.robot().velW(), refVal);
    checkVelocity(robots.robot().accW(), refVal);
  }
}

BOOST_AUTO_TEST_CASE(TestRobotZMPSimple)
{
  auto robots_ptr = makeRobots();
  auto & robots = *robots_ptr;
  auto & robot = robots.robot();

  // Put all mass on the left foot, ZMP should be under the sensor
  const auto normalForce = robot.mass() * 10;
  const auto sensorNames = std::vector<std::string>{"LeftFootForceSensor", "RightFootForceSensor"};
  auto & lfs = robot.forceSensor("LeftFootForceSensor");
  auto & rfs = robot.forceSensor("RightFootForceSensor");
  // Prevent using the JVRC1 calibration files (when they exist)
  // for the ZMP computation to obtain repeatable results here.
  // Resetting the calibrator makes sure that it has no effect.
  //
  // This test assumes that the sensor is at the model position and that there
  // is no force offset.
  //
  // XXX We should investigate the effect of calibrator on ZMP measurement,
  // and write a test that checks this case as well
  lfs.resetCalibrator();
  rfs.resetCalibrator();

  {
    // ZMP under left sensor
    const auto forceLeftSurface = sva::ForceVecd{Eigen::Vector3d::Zero(), {0., 0., normalForce}};
    sva::PTransformd X_0_ls = lfs.X_0_f(robot);
    X_0_ls.translation().z() = 0;
    auto X_ls_f = lfs.X_0_f(robot) * X_0_ls.inv();
    lfs.wrench(X_ls_f.dualMul(forceLeftSurface));
    rfs.wrench(sva::ForceVecd::Zero());

    auto zmpIdeal = X_0_ls.translation();
    auto zmpComputed = robot.zmp(sensorNames, Eigen::Vector3d::Zero(), {0., 0., 1.});
    BOOST_CHECK_MESSAGE(zmpComputed.isApprox(zmpIdeal, 1e-10), "Error in Robot::zmp computation with leftFootRatio="
                                                                   << "\nExpected: " << zmpIdeal.transpose()
                                                                   << "\nGot: " << zmpComputed.transpose());
  }

  {
    // ZMP under right sensor
    const auto forceRightSurface = sva::ForceVecd{Eigen::Vector3d::Zero(), {0., 0., normalForce}};
    sva::PTransformd X_0_rs = rfs.X_0_f(robot);
    X_0_rs.translation().z() = 0;
    auto X_rs_f = lfs.X_0_f(robot) * X_0_rs.inv();
    lfs.wrench(X_rs_f.dualMul(forceRightSurface));
    rfs.wrench(sva::ForceVecd::Zero());

    auto zmpIdeal = X_0_rs.translation();
    auto zmpComputed = robot.zmp(sensorNames, Eigen::Vector3d::Zero(), {0., 0., 1.});
    BOOST_CHECK_MESSAGE(zmpComputed.isApprox(zmpIdeal, 1e-10), "Error in Robot::zmp computation with leftFootRatio="
                                                                   << "\nExpected: " << zmpIdeal.transpose()
                                                                   << "\nGot: " << zmpComputed.transpose());
  }

  { // checks that zmp throws if used with null force
    rfs.wrench(sva::ForceVecd::Zero());
    lfs.wrench(sva::ForceVecd::Zero());
    BOOST_CHECK_THROW(robot.zmp(sensorNames, Eigen::Vector3d::Zero(), {0., 0., 1.}), std::runtime_error);
  }
}

using namespace rbd;
using namespace Eigen;
using namespace sva;
using namespace tvm;

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

BOOST_AUTO_TEST_CASE(FrameTest)
{
  rbd::parsers::ParserResult pr;
  std::tie(pr.mb, pr.mbg, pr.mbc) = XYZRobot();

  mc_rbdyn::Robots robots;
  robots.load({"XYZ1", pr}, "XYZ1");

  auto & xyz = robots.robot("XYZ1");
  sva::PTransformd Xs(Quaterniond::UnitRandom(), Vector3d::Random());
  auto & f1 = xyz.makeFrame("ee1", "b4", Xs);

  graph::CallGraph callgraph;
  auto inputs = std::make_shared<graph::internal::Inputs>();
  inputs->addInput(f1, mc_rbdyn::Frame::Output::Position);
  inputs->addInput(f1, mc_rbdyn::Frame::Output::Jacobian);
  inputs->addInput(f1, mc_rbdyn::Frame::Output::Velocity);
  inputs->addInput(f1, mc_rbdyn::Frame::Output::NormalAcceleration);
  inputs->addInput(f1, mc_rbdyn::Frame::Output::JDot);
  callgraph.add(inputs);
  callgraph.update();

  for(int i = 0; i < 100; ++i)
  {
    double h = 1e-8;
    Vector3d q = Vector3d::Random();
    xyz.q()->set(q);
    callgraph.execute();
    PTransformd X0 = f1.position();
    MatrixXd J0 = f1.jacobian();

    // test Jacobian matrix by finite differences
    MatrixXd Jd(6, 3);
    for(int i = 0; i < 3; ++i)
    {
      q[i] += h;
      xyz.q()->set(q);
      callgraph.execute();
      PTransformd Xi = f1.position();
      Jd.col(i).head<3>() = rotationError(X0.rotation(), Xi.rotation()) / h;
      Jd.col(i).tail<3>() = (Xi.translation() - X0.translation()) / h;

      q[i] -= h;
    }
    BOOST_CHECK_SMALL((J0 - Jd).norm(), 1e-6);

    // Check velocity (is it equal to J dq ?)
    Vector3d dq = Vector3d::Random();
    xyz.q()->set(q);
    dot(xyz.q())->set(dq);
    callgraph.execute();
    BOOST_CHECK_SMALL((f1.velocity().vector() - f1.jacobian() * dq).norm(), 1e-6);

    // Checking normal acceleration (is it equal to dv/dt - J ddq, with dv/dt approximated by finite forward differences
    // ?)
    Vector3d ddq = Vector3d::Random().normalized();

    // we consider a constant-acceleration trajectory in variable space
    Vector3d dq1 = dq + ddq * h;
    Vector3d q1 = q + dq * h + 0.5 * ddq.cwiseProduct(ddq) * h * h;

    Vector6d v0 = f1.velocity().vector();
    xyz.q()->set(q1);
    dot(xyz.q())->set(dq1);
    callgraph.execute();
    Vector6d v1 = f1.velocity().vector();
    Vector6d a = (v1 - v0) / h - J0 * ddq;

    const auto & na = f1.normalAcceleration().vector();
    BOOST_CHECK_SMALL((na - a).norm(), 1e-6);

    // test JDot
    BOOST_CHECK_SMALL((na - f1.JDot() * dq).norm(), 1e-6);
  }
}
