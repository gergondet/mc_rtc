/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rtc/logging.h>
#include <mc_solver/KinematicsConstraint.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/MetaTask.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Force.h>
#include <mc_rtc/gui/Form.h>

#include <tvm/solver/defaultLeastSquareSolver.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_solver
{
QPSolver::QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots,
                   std::shared_ptr<mc_rtc::Logger> logger,
                   std::shared_ptr<mc_rtc::gui::StateBuilder> gui,
                   double timeStep)
: robots_(robots), dt_(timeStep), solver_(tvm::solver::DefaultLSSolverOptions{}), logger_(logger), gui_(gui)
{
  if(timeStep <= 0)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("dt has to be > 0! dt = {}", timeStep);
  }
  if(!gui_)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("GUI cannot be null");
  }
  if(!logger_)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("Logger cannot be null");
  }
  realRobots_ = std::make_shared<mc_rbdyn::Robots>(*robots_);
}

std::vector<ConstraintPtr>::iterator QPSolver::getConstraint(ConstraintPtr & c)
{
  return std::find(constraints_.begin(), constraints_.end(), c);
}

std::vector<mc_tasks::MetaTaskPtr>::iterator QPSolver::getTask(mc_tasks::MetaTaskPtr & t)
{
  return std::find(tasks_.begin(), tasks_.end(), t);
}

void QPSolver::addConstraint(ConstraintPtr cs)
{
  auto it = getConstraint(cs);
  if(cs && it == constraints_.end())
  {
    cs->addToSolver(*this);
    constraints_.push_back(cs);
    mc_rtc::log::info("Added constraint {}", cs->name());
  }
}

void QPSolver::removeConstraint(ConstraintPtr cs)
{
  auto it = getConstraint(cs);
  if(it != constraints_.end())
  {
    cs->removeFromSolver(*this);
    constraints_.erase(it);
    mc_rtc::log::info("Removed constraint {}", cs->name());
  }
}

void QPSolver::addTask(mc_tasks::MetaTaskPtr task)
{
  if(task && getTask(task) == tasks_.end())
  {
    task->addToSolver(*this);
    task->addToLogger(*logger_);
    task->addToGUI(*gui_);
    gui_->addElement({"Tasks", task->name()},
                     mc_rtc::gui::Button("Remove from solver", [this, task]() { this->removeTask(task); }));
    tasks_.push_back(task);
    mc_rtc::log::info("Added task {}", task->name());
  }
}

void QPSolver::removeTask(mc_tasks::MetaTaskPtr task)
{
  auto it = getTask(task);
  if(it != tasks_.end())
  {
    task->removeFromSolver(*this);
    task->removeFromLogger(*logger_);
    task->removeFromGUI(*gui_);
    tasks_.erase(it);
    mc_rtc::log::info("Removed task {}", task->name());
  }
}

bool QPSolver::run(FeedbackType fType)
{
  bool success = false;
  switch(fType)
  {
    case FeedbackType::None:
      success = runOpenLoop();
      break;
    case FeedbackType::Joints:
      success = runJointsFeedback(false);
      break;
    case FeedbackType::JointsWVelocity:
      success = runJointsFeedback(true);
      break;
    case FeedbackType::ObservedRobots:
      success = runClosedLoop();
      break;
    default:
      mc_rtc::log::error("FeedbackType set to unknown value");
      break;
  }
  return success;
}

bool QPSolver::runCommon()
{
  for(auto & c : constraints_)
  {
    c->update(*this);
  }
  for(auto & t : tasks_)
  {
    t->update(*this);
  }
  return solver_.solve(problem_);
}

bool QPSolver::runOpenLoop()
{
  if(runCommon())
  {
    for(auto & robot : *robots_)
    {
      auto & mb = robot->mb();
      auto & mbc = robot->mbc();
      if(mb.nrDof() > 0)
      {
        rbd::vectorToParam(robot->tau()->value(), robot->mbc().jointTorque);
        rbd::vectorToParam(tvm::dot(robot->q(), 2).value(), robot->mbc().alphaD);
        rbd::eulerIntegration(mb, mbc, dt_);
        // FIXME We update the kinematics here for logging and display purpose mostly
        robot->updateKinematics();
      }
    }
    return true;
  }
  return false;
}

bool QPSolver::runJointsFeedback(bool wVelocity)
{
  if(control_q_.size() < robots().size())
  {
    prev_encoders_.resize(robots().size());
    encoders_alpha_.resize(robots().size());
    control_q_.resize(robots().size());
    control_alpha_.resize(robots().size());
  }
  for(size_t i = 0; i < robots().size(); ++i)
  {
    auto & robot = *(robots_->robots()[i]);
    control_q_[i] = robot.mbc().q;
    control_alpha_[i] = robot.mbc().alpha;
    const auto & encoders = robot.encoderValues();
    if(encoders.size())
    {
      // FIXME Not correct for every joint types
      if(prev_encoders_[i].size() == 0)
      {
        prev_encoders_[i] = robot.encoderValues();
        encoders_alpha_[i].resize(prev_encoders_[i].size());
        if(i == 0)
        {
          logger_->addLogEntry("alphaIn", [this]() -> const std::vector<double> & { return encoders_alpha_[0]; });
        }
        else
        {
          logger_->addLogEntry(robot.name() + "_alphaIn",
                               [this, i]() -> const std::vector<double> & { return encoders_alpha_[i]; });
        }
      }
      for(size_t j = 0; j < encoders.size(); ++j)
      {
        encoders_alpha_[i][j] = (encoders[j] - prev_encoders_[i][j]) / dt_;
        prev_encoders_[i][j] = encoders[j];
      }
      const auto & rjo = robot.module().ref_joint_order();
      for(size_t j = 0; j < rjo.size(); ++j)
      {
        const auto & jN = rjo[j];
        if(!robot.hasJoint(jN))
        {
          continue;
        }
        auto jI = robot.jointIndexByName(jN);
        robot.mbc().q[jI][0] = encoders[j];
        if(wVelocity)
        {
          robot.mbc().alpha[jI][0] = encoders_alpha_[i][j];
        }
      }
      rbd::forwardKinematics(robot.mb(), robot.mbc());
      robot.forwardKinematics();
      robot.forwardVelocity();
      robot.forwardAcceleration();
    }
  }
  if(runCommon())
  {
    for(size_t i = 0; i < robots_->size(); ++i)
    {
      auto & robot = robots_->robots()[i];
      auto & mb = robot->mb();
      auto & mbc = robot->mbc();
      mbc.q = control_q_[i];
      mbc.alpha = control_alpha_[i];
      if(mb.nrDof() > 0)
      {
        rbd::vectorToParam(robot->tau()->value(), robot->mbc().jointTorque);
        rbd::vectorToParam(tvm::dot(robot->q(), 2).value(), robot->mbc().alphaD);
        rbd::eulerIntegration(mb, mbc, dt_);
        // FIXME We update the kinematics here for logging and display purpose mostly
        robot->updateKinematics();
      }
    }
    return true;
  }
  return false;
}

bool QPSolver::runClosedLoop()
{
  if(control_q_.size() < robots().size())
  {
    control_q_.resize(robots().size());
    control_alpha_.resize(robots().size());
  }

  for(size_t i = 0; i < robots().size(); ++i)
  {
    auto & robot = *(robots_->robots()[i]);
    auto & realRobot = *(realRobots_->robots()[i]);

    // Save old integrator state
    control_q_[i] = robot.mbc().q;
    control_alpha_[i] = robot.mbc().alpha;

    // Set robot state from estimator
    robot.mbc().q = realRobot.mbc().q;
    robot.mbc().alpha = realRobot.mbc().alpha;
    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();
  }

  // Solve QP and integrate
  if(runCommon())
  {
    for(size_t i = 0; i < robots_->size(); ++i)
    {
      auto & robot = robots_->robots()[i];
      auto & mb = robot->mb();
      auto & mbc = robot->mbc();
      mbc.q = control_q_[i];
      mbc.alpha = control_alpha_[i];
      if(mb.nrDof() > 0)
      {
        updateRobot(*robot);
      }
    }
    return true;
  }
  return false;
}

} // namespace mc_solver
