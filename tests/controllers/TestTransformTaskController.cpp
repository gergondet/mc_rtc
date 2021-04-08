/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/TransformTask.h>

#include <boost/test/unit_test.hpp>

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestTransformTaskController : public MCController
{
public:
  TestTransformTaskController(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    BOOST_REQUIRE(robots().hasRobot("ground"));
    solver().addConstraint(dynamicsConstraint_);
    postureTask_->stiffness(1);
    postureTask_->weight(1);
    solver().addTask(postureTask_);
    solver().addContact({"jvrc1", "ground", "LeftFoot", "AllGround"});
    solver().addContact({"jvrc1", "ground", "RightFoot", "AllGround"});

    /* Create and add the position task with the default stiffness/weight */
    efTask = std::make_shared<mc_tasks::TransformTask>(robot().frame("R_WRIST_Y_S"));
    efTask->stiffness(5);
    solver().addTask(efTask);

    comTask = std::make_shared<mc_tasks::CoMTask>(robot());
    solver().addTask(comTask);

    mc_rtc::log::success("Created TestEndEffectorTaskController");
  }

  virtual bool run() override
  {
    bool ret = MCController::run();
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 1500)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(efTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(efTask->speed().norm(), 1e-3);

      /* Apply dimWeight and give a "crazy" position target */
      postureTask_->posture(robot().mbc().q);
      Eigen::VectorXd dimW(6);
      dimW << 1., 1., 1., 1., 1., 0.;
      efTask->dimWeight(dimW);
      efTask->target(sva::PTransformd{Eigen::Vector3d(0., 0., 100.)} * efTask->target());
    }
    if(nrIter == 2000)
    {
      BOOST_CHECK_SMALL(efTask->eval().norm(), 0.015);
      BOOST_CHECK_SMALL(efTask->speed().norm(), 0.015);

      /* Reset the task and ask to raise the hand by 15 cm using only the
       * right arm joints */
      efTask->reset();
      Eigen::VectorXd dimW(6);
      dimW << 1., 1., 1., 1., 1., 1.;
      efTask->dimWeight(dimW);
      efTask->selectActiveJoints(solver(), active_joints);
      efTask->target(sva::PTransformd{Eigen::Vector3d(0., 0., 0.15)} * efTask->target());
    }
    if(nrIter == 3000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(efTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(efTask->speed().norm(), 1e-2);

      /* Now move the hand down again, forbid elbow pitch movement in the task */
      efTask->reset();
      efTask->selectInactiveJoints(solver(), {"R_ELBOW_P"});
      orig_rep = robot().mbc().q[robot().jointIndexByName("R_ELBOW_P")][0];
      efTask->target(sva::PTransformd{Eigen::Vector3d(0., 0., -0.15)} * efTask->target());

      comTask->selectInactiveJoints(solver(), {"R_ELBOW_P"});

      /* Also reset the joint target in posture task */
      postureTask_->target({{"R_ELBOW_P", {orig_rep}}});
    }
    if(nrIter == 4000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(efTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(efTask->speed().norm(), 1e-2);

      /* And that RARM_JOINT3 didn't move. Note that the error is not so
       * small because of other tasks' interaction */
      double current_rep = robot().mbc().q[robot().jointIndexByName("R_ELBOW_P")][0];
      BOOST_CHECK_SMALL(fabs(orig_rep - current_rep), 1e-2);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    /* Reset the task to the current end-effector position */
    efTask->reset();
    comTask->reset();
    /* Move the end-effector 10cm forward, 10 cm to the right and 10 cm upward */
    efTask->target(sva::PTransformd(sva::RotY<double>(-M_PI / 2),
                                    efTask->target().translation() + Eigen::Vector3d(0.3, -0.1, 0.2)));
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::TransformTask> efTask = nullptr;
  std::shared_ptr<mc_tasks::CoMTask> comTask = nullptr;
  std::vector<std::string> active_joints = {"R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                            "R_ELBOW_Y",    "R_WRIST_R",    "R_WRIST_Y"};
  double orig_rep = 0;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestTransformTaskController", mc_control::TestTransformTaskController)
