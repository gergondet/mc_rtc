/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/TransformTask.h>

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

    lfTask_ = std::make_shared<mc_tasks::TransformTask>(robot().frame("LeftFoot"));
    solver().addTask(lfTask_);
  }

  void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    comTask_->reset();
    lfTask_->reset();
    lfTask_->target(sva::PTransformd(Eigen::Vector3d(0, 0, 0.2)) * lfTask_->target());
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
      BOOST_CHECK(sva::transformError(X_0_lf, X_0_lf_init).vector().norm() < 1e-8);
      static auto X_0_rf_init = robot("jvrc1").frame("RightFoot").position();
      auto X_0_rf = robot("jvrc1").frame("RightFoot").position();
      BOOST_CHECK(sva::transformError(X_0_rf, X_0_rf_init).vector().norm() < 1e-8);
      BOOST_CHECK(lfTask_->error().value().norm() > 0.1999);
    }
    if(nrIter_ == 500)
    {
      solver().removeContact({"jvrc1", "ground", "LeftFoot", "AllGround"});
      solver().removeContact({"jvrc1", "ground", "RightFoot", "AllGround"});
      solver().addContact({"ground", "jvrc1", "AllGround", "LeftFoot"});
      solver().addContact({"ground", "jvrc1", "AllGround", "RightFoot"});
    }
    if(nrIter_ >= 500 && nrIter_ < 1000)
    {
      // Check that the feet contacts are well maintained
      static auto X_0_lf_init = robot("jvrc1").frame("LeftFoot").position();
      auto X_0_lf = robot("jvrc1").frame("LeftFoot").position();
      BOOST_CHECK(sva::transformError(X_0_lf, X_0_lf_init).vector().norm() < 1e-8);
      static auto X_0_rf_init = robot("jvrc1").frame("RightFoot").position();
      auto X_0_rf = robot("jvrc1").frame("RightFoot").position();
      BOOST_CHECK(sva::transformError(X_0_rf, X_0_rf_init).vector().norm() < 1e-8);
      BOOST_CHECK(lfTask_->error().value().norm() > 0.1999);
    }
    if(nrIter_ == 1000)
    {
      solver().removeContact({"ground", "jvrc1", "AllGround", "LeftFoot"});
      solver().removeContact({"ground", "jvrc1", "AllGround", "RightFoot"});
      Eigen::Vector6d dof = Eigen::Vector6d::Ones();
      dof(5) = 0.0;
      solver().addContact({"jvrc1", "ground", "LeftFoot", "AllGround", mc_rbdyn::Contact::defaultFriction, dof});
      solver().addContact({"jvrc1", "ground", "RightFoot", "AllGround"});
    }
    if(nrIter_ >= 1000 && nrIter_ < 1500)
    {
      static double prev_error = lfTask_->error().value().norm();
      if(nrIter_ > 1001)
      {
        double error = lfTask_->error().value().norm();
        BOOST_CHECK(error < prev_error);
        prev_error = error;
      }
      static auto X_0_lf_init = robot("jvrc1").frame("LeftFoot").position();
      auto X_0_lf = robot("jvrc1").frame("LeftFoot").position();
      auto X_lfi_lf = X_0_lf * X_0_lf_init.inv();
      X_lfi_lf.translation().z() = 0.0;
      BOOST_CHECK(sva::transformVelocity(X_lfi_lf).vector().norm() < 1e-4);
      static auto X_0_rf_init = robot("jvrc1").frame("RightFoot").position();
      auto X_0_rf = robot("jvrc1").frame("RightFoot").position();
      BOOST_CHECK(sva::transformError(X_0_rf, X_0_rf_init).vector().norm() < 1e-4);
    }
    if(nrIter_ == 1500)
    {
      Eigen::Vector6d dof = Eigen::Vector6d::Ones();
      dof(2) = 0.0;
      solver().addContact({"jvrc1", "ground", "LeftFoot", "AllGround", mc_rbdyn::Contact::defaultFriction, dof});
      lfTask_->reset();
      lfTask_->target(sva::PTransformd(sva::RotZ(M_PI / 4)) * lfTask_->target());
    }
    if(nrIter_ >= 1500 && nrIter_ < 2000)
    {
      static double prev_error = lfTask_->error().value().norm();
      double error = lfTask_->error().value().norm();
      if(nrIter_ > 1501) // At iter 1500, prev_error is not updated yet
      {
        BOOST_CHECK(error < prev_error);
      }
      prev_error = error;
      static auto X_0_lf_init = robot("jvrc1").frame("LeftFoot").position();
      auto X_0_lf = robot("jvrc1").frame("LeftFoot").position();
      auto X_lfi_lf = X_0_lf * X_0_lf_init.inv();
      auto lfi_lf_error = sva::transformVelocity(X_lfi_lf).vector();
      lfi_lf_error(2) = 0.0;
      BOOST_CHECK(lfi_lf_error.norm() < 1e-4);
      static auto X_0_rf_init = robot("jvrc1").frame("RightFoot").position();
      auto X_0_rf = robot("jvrc1").frame("RightFoot").position();
      BOOST_CHECK(sva::transformError(X_0_rf, X_0_rf_init).vector().norm() < 1e-4);
    }
    return ret;
  }

private:
  size_t nrIter_ = 0;
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
  std::shared_ptr<mc_tasks::TransformTask> lfTask_;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestContactsController", mc_control::TestContactsController)
