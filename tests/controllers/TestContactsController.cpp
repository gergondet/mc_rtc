/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestContactsController : public MCController
{
  TestContactsController(mc_rbdyn::RobotModulePtr rm, double dt) : MCController(rm, dt)
  {
    solver().addConstraint(dynamicsConstraint_);
    postureTask_->stiffness(1.0);
    postureTask_->weight(1.0);
    solver().addTask(postureTask_);
    solver().addContact({"jvrc1", "ground", "LeftFoot", "AllGround"});
    solver().addContact({"jvrc1", "ground", "RightFoot", "AllGround"});

    comTask_ = std::make_shared<mc_tasks::CoMTask>(robot());
    solver().addTask(comTask_);
  }

  void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    comTask_->reset();
  }

  bool run() override
  {
    bool ret = MCController::run();
    BOOST_CHECK(ret);
    nrIter_++;
    if(nrIter_ < 500)
    {
      // Check that the feet contacts are well maintained
      static auto X_0_lf_init = robot("jvrc1").frame("LeftFoot").position();
      auto X_0_lf = robot("jvrc1").frame("LeftFoot").position();
      BOOST_CHECK(sva::transformError(X_0_lf, X_0_lf_init).vector().norm() < 1e-10);
      static auto X_0_rf_init = robot("jvrc1").frame("RightFoot").position();
      auto X_0_rf = robot("jvrc1").frame("RightFoot").position();
      BOOST_CHECK(sva::transformError(X_0_rf, X_0_rf_init).vector().norm() < 1e-10);
    }
    return ret;
  }

private:
  size_t nrIter_ = 0;
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestContactsController", mc_control::TestContactsController)
