/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/KinematicsConstraint.h>
#include <mc_solver/QPSolver.h>

#include <mc_tasks/MetaTask.h>

#include <mc_tvm/ContactFunction.h>

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/surface_utils.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Force.h>
#include <mc_rtc/gui/Form.h>
#include <mc_rtc/logging.h>

#include <tvm/solver/defaultLeastSquareSolver.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_solver
{

namespace
{

inline static Eigen::MatrixXd discretizedFrictionCone(double muI)
{
  Eigen::MatrixXd C(4, 3);
  double mu = muI / std::sqrt(2);
  C << Eigen::Matrix2d::Identity(), Eigen::Vector2d::Constant(mu), -Eigen::Matrix2d::Identity(),
      Eigen::Vector2d::Constant(mu);
  return C;
}

inline static void addRemoveContactToGUI(mc_solver::QPSolver & solver, const std::vector<mc_rbdyn::Contact> & contacts)
{
  auto & gui = solver.gui();
  gui.removeCategory({"Contacts", "Remove"});
  if(contacts.empty())
  {
    return;
  }
  for(size_t i = 0; i < contacts.size(); ++i)
  {
    const auto & c = contacts[i];
    auto button = fmt::format("Remove {}::{}/{}::{} contact", c.r1, c.r1Surface, c.r2, c.r2Surface);
    gui.addElement({"Contacts", "Remove"}, mc_rtc::gui::Button(button, [&c, &solver]() { solver.removeContact(c); }));
  }
}

} // namespace

QPSolver::QPSolver(mc_rbdyn::RobotsPtr robots,
                   mc_rbdyn::RobotsPtr realRobots,
                   std::shared_ptr<mc_rtc::Logger> logger,
                   std::shared_ptr<mc_rtc::gui::StateBuilder> gui,
                   double timeStep)
: robots_(robots), realRobots_(realRobots), dt_(timeStep), solver_(tvm::solver::DefaultLSSolverOptions{}),
  logger_(logger), gui_(gui)
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
    auto dynamics = std::dynamic_pointer_cast<DynamicsConstraint>(cs);
    if(addDynamicsConstraint(dynamics))
    {
      cs->addToSolver(*this);
      constraints_.push_back(cs);
      mc_rtc::log::info("Added constraint {}", cs->name());
    }
  }
}

bool QPSolver::addDynamicsConstraint(const DynamicsConstraintPtr & cs)
{
  if(!cs)
  {
    return true;
  }
  if(dynamics_.count(cs->robot().name()))
  {
    mc_rtc::log::error("A DynamicsConstraint has already been added for {}", cs->robot().name());
    return false;
  }
  dynamics_[cs->robot().name()] = cs;
  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    const auto & contact = contacts_[i];
    auto & data = contactsData_[i];
    bool isR1 = contact.r1 == cs->robot().name();
    bool isR2 = contact.r2 == cs->robot().name();
    if(isR1 || isR2)
    {
      auto & r1 = robots_->robot(contact.r1);
      auto & r2 = robots_->robot(contact.r2);
      auto & s1 = r1.surface(contact.r1Surface);
      auto & s2 = r2.surface(contact.r2Surface);
      auto & f1 = s1.frame();
      auto & f2 = s2.frame();
      auto C = discretizedFrictionCone(contact.friction);
      // FIXME Debug mc_rbdyn::intersection
      // auto s1Points = mc_rbdyn::intersection(s1, s2);
      auto s1Points = s1.points();
      if(isR1)
      {
        addContactToDynamics(contact.r1, f1, s1Points, data.f1_, data.f1Constraints_, C, 1.0);
      }
      if(isR2)
      {
        std::vector<sva::PTransformd> s2Points;
        s2Points.reserve(s1Points.size());
        auto X_b2_b1 = r1.mbc().bodyPosW[r1.bodyIndexByName(s1.frame().body())]
                       * r2.mbc().bodyPosW[r2.bodyIndexByName(s2.frame().body())].inv();
        std::transform(s1Points.begin(), s1Points.end(), std::back_inserter(s2Points),
                       [&](const auto & X_b1_p) { return X_b1_p * X_b2_b1; });
        addContactToDynamics(contact.r2, f2, s2Points, data.f2_, data.f2Constraints_, C, 2.0);
      }
    }
  }
  return true;
}

void QPSolver::removeConstraint(ConstraintPtr cs)
{
  auto it = getConstraint(cs);
  if(it != constraints_.end())
  {
    cs->removeFromSolver(*this);
    constraints_.erase(it);
    mc_rtc::log::info("Removed constraint {}", cs->name());
    auto dynamics = std::dynamic_pointer_cast<DynamicsConstraint>(cs);
    if(dynamics)
    {
      removeDynamicsConstraint(dynamics);
    }
  }
}

void QPSolver::removeDynamicsConstraint(const DynamicsConstraintPtr & cs)
{
  dynamics_.erase(cs->robot().name());
  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    const auto & contact = contacts_[i];
    auto & data = contactsData_[i];
    auto clearContacts = [&](const std::string & robot, tvm::VariableVector & forces,
                             std::vector<tvm::TaskWithRequirementsPtr> & constraints) {
      if(robot != cs->robot().name())
      {
        return;
      }
      for(auto & c : constraints)
      {
        problem_.remove(*c);
      }
      constraints.clear();
      forces = tvm::VariableVector();
    };
    clearContacts(contact.r1, data.f1_, data.f1Constraints_);
    clearContacts(contact.r2, data.f2_, data.f2Constraints_);
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

size_t QPSolver::getContactIdx(const mc_rbdyn::Contact & contact)
{
  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(contacts_[i] == contact)
    {
      return i;
    }
  }
  return contacts_.size();
}

void QPSolver::addContactToDynamics(const std::string & robot,
                                    mc_rbdyn::Frame & frame,
                                    const std::vector<sva::PTransformd> & points,
                                    tvm::VariableVector & forces,
                                    std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                                    const Eigen::MatrixXd & frictionCone,
                                    double dir)
{
  auto it = dynamics_.find(robot);
  if(it == dynamics_.end())
  {
    return;
  }
  if(constraints.size())
  {
    // FIXME Instead of this we should be able to change C
    for(const auto & c : constraints)
    {
      problem_.remove(*c);
    }
    constraints.clear();
    gui_->removeCategory({"Forces", robot, std::string(frame.name())});
  }
  else
  {
    it->second->removeFromSolver(*this);
    forces = it->second->dynamic().addContact(frame, points, dir);
    it->second->addToSolver(*this);
  }
  for(int i = 0; i < forces.numberOfVariables(); ++i)
  {
    auto & f = forces[i];
    auto point = points[static_cast<size_t>(i)];
    constraints.push_back(problem_.add(dir * frictionCone * f >= 0.0, {tvm::requirements::PriorityLevel(0)}));
    gui_->addElement({"Forces", robot, std::string(frame.name())},
                     mc_rtc::gui::Force(
                         f->name(), mc_rtc::gui::ForceConfig(mc_rtc::gui::Color::Red),
                         [f]() { return sva::ForceVecd(Eigen::Vector3d::Zero(), f->value()); },
                         [&frame, point]() { return point * frame.robot().frame(frame.body()).position(); }));
  }
}

void QPSolver::addVirtualContact(const mc_rbdyn::Contact & contact)
{
  bool hasWork = false;
  std::tie(std::ignore, hasWork) = addVirtualContactImpl(contact);
  if(hasWork)
  {
    mc_rtc::log::info("Added virtual contact {}::{}/{}::{} (dof: {})", contact.r1, contact.r1Surface, contact.r2,
                      contact.r2Surface, contact.dof.transpose());
  }
}

auto QPSolver::addVirtualContactImpl(const mc_rbdyn::Contact & contact) -> std::tuple<size_t, bool>
{
  bool hasWork = false;
  auto idx = getContactIdx(contact);
  if(idx < contacts_.size())
  {
    const auto & oldContact = contacts_[idx];
    if(oldContact.dof == contact.dof && oldContact.friction == contact.friction)
    {
      return std::make_tuple(idx, hasWork);
    }
    hasWork = contact.friction != oldContact.friction;
    contacts_[idx] = contact;
  }
  else
  {
    hasWork = true;
    contacts_.push_back(contact);
    addRemoveContactToGUI(*this, contacts_);
  }
  auto & data = idx < contactsData_.size() ? contactsData_[idx] : contactsData_.emplace_back();
  auto & r1 = robots_->robot(contact.r1);
  auto & r2 = robots_->robot(contact.r2);
  auto & f1 = r1.frame(contact.r1Surface);
  auto & f2 = r2.frame(contact.r2Surface);
  if(!data.contactConstraint_) // New contact
  {
    auto contact_fn = std::make_shared<mc_tvm::ContactFunction>(f1, f2, contact.dof);
    data.contactConstraint_ = problem_.add(contact_fn == 0., tvm::task_dynamics::PD(1.0 / dt_, 1.0 / dt_),
                                           {tvm::requirements::PriorityLevel(0)});
  }
  else
  {
    auto contact_fn = std::static_pointer_cast<mc_tvm::ContactFunction>(data.contactConstraint_->task.function());
    contact_fn->dof(contact.dof);
  }
  return std::make_tuple(idx, hasWork);
}

void QPSolver::addContact(const mc_rbdyn::Contact & contact)
{
  size_t idx = contacts_.size();
  bool hasWork = false;
  std::tie(idx, hasWork) = addVirtualContactImpl(contact);
  if(!hasWork)
  {
    return;
  }
  auto & data = contactsData_[idx];
  auto & r1 = robots_->robot(contact.r1);
  auto & r2 = robots_->robot(contact.r2);
  auto & s1 = r1.surface(contact.r1Surface);
  auto & s2 = r2.surface(contact.r2Surface);
  auto & f1 = s1.frame();
  auto & f2 = s2.frame();
  // FIXME Let the user decide how much the friction cone should be discretized
  auto C = discretizedFrictionCone(contact.friction);
  auto addContactForce = [&](const std::string & robot, mc_rbdyn::Frame & frame,
                             const std::vector<sva::PTransformd> & points, tvm::VariableVector & forces,
                             std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                             double dir) { addContactToDynamics(robot, frame, points, forces, constraints, C, dir); };
  // FIXME These points computation are a waste of time if they are not needed
  // FIXME Debug mc_rbdyn::intersection
  // auto s1Points = mc_rbdyn::intersection(s1, s2);
  auto s1Points = s1.points();
  addContactForce(contact.r1, f1, s1Points, data.f1_, data.f1Constraints_, 1.0);
  std::vector<sva::PTransformd> s2Points;
  s2Points.reserve(s1Points.size());
  auto X_b2_b1 = r1.mbc().bodyPosW[r1.bodyIndexByName(s1.frame().body())]
                 * r2.mbc().bodyPosW[r2.bodyIndexByName(s2.frame().body())].inv();
  std::transform(s1Points.begin(), s1Points.end(), std::back_inserter(s2Points),
                 [&](const auto & X_b1_p) { return X_b1_p * X_b2_b1; });
  addContactForce(contact.r2, f2, s2Points, data.f2_, data.f2Constraints_, -1.0);
  mc_rtc::log::info("Added contact {}::{}/{}::{} (dof: {}, friction: {})", contact.r1, contact.r1Surface, contact.r2,
                    contact.r2Surface, contact.dof.transpose(), contact.friction);
}

void QPSolver::removeContact(const mc_rbdyn::Contact & contact)
{
  auto idx = getContactIdx(contact);
  if(idx >= contacts_.size())
  {
    return;
  }
  auto & data = contactsData_[idx];
  auto r1DynamicsIt = dynamics_.find(contact.r1);
  if(r1DynamicsIt != dynamics_.end())
  {
    gui_->removeCategory({"Forces", r1DynamicsIt->second->robot().name(), contact.r1Surface});
    r1DynamicsIt->second->removeFromSolver(*this);
    r1DynamicsIt->second->dynamic().removeContact(robots_->robot(contact.r1).surface(contact.r1Surface).frame());
    r1DynamicsIt->second->addToSolver(*this);
  }
  auto r2DynamicsIt = dynamics_.find(contact.r2);
  if(r2DynamicsIt != dynamics_.end())
  {
    gui_->removeCategory({"Forces", r2DynamicsIt->second->robot().name(), contact.r2Surface});
    r2DynamicsIt->second->removeFromSolver(*this);
    r2DynamicsIt->second->dynamic().removeContact(robots_->robot(contact.r2).surface(contact.r2Surface).frame());
    r2DynamicsIt->second->addToSolver(*this);
  }
  for(const auto & c : data.f1Constraints_)
  {
    problem_.remove(*c);
  }
  for(const auto & c : data.f2Constraints_)
  {
    problem_.remove(*c);
  }
  if(data.contactConstraint_)
  {
    problem_.remove(*data.contactConstraint_);
    data.contactConstraint_.reset();
  }
  contacts_.erase(contacts_.begin() + static_cast<decltype(contacts_)::difference_type>(idx));
  contactsData_.erase(contactsData_.begin() + static_cast<decltype(contacts_)::difference_type>(idx));
  mc_rtc::log::info("Removed contact {}::{}/{}::{}", contact.r1, contact.r1Surface, contact.r2, contact.r2Surface);
  addRemoveContactToGUI(*this, contacts_);
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
  auto start_t = mc_rtc::clock::now();
  auto r = solver_.solve(problem_);
  solve_dt_ = mc_rtc::clock::now() - start_t;
  return r;
}

bool QPSolver::runOpenLoop()
{
  if(runCommon())
  {
    for(auto & robot : *robots_)
    {
      auto & mb = robot->mb();
      if(mb.nrDof() > 0)
      {
        updateRobot(*robot);
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
    control_q_[i] = robot.q()->value();
    control_alpha_[i] = robot.alpha()->value();
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
        auto jI = robot.refJointIndexToQIndex(j);
        if(jI == -1)
        {
          continue;
        }
        robot.q()->set(jI, encoders[j]);
        if(wVelocity)
        {
          jI = robot.refJointIndexToQDotIndex(j);
          robot.alpha()->set(jI, encoders_alpha_[i][j]);
        }
      }
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
      robot->q()->set(control_q_[i]);
      robot->alpha()->set(control_alpha_[i]);
      if(robot->mb().nrDof() > 0)
      {
        updateRobot(*robot);
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
    control_q_[i] = robot.q()->value();
    control_alpha_[i] = robot.alpha()->value();

    // Set robot state from estimator
    robot.q()->set(realRobot.q()->value());
    robot.alpha()->set(realRobot.alpha()->value());
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
      robot->q()->set(control_q_[i]);
      robot->alpha()->set(control_alpha_[i]);
      if(robot->mb().nrDof() > 0)
      {
        updateRobot(*robot);
      }
    }
    return true;
  }
  return false;
}

void QPSolver::updateRobot(mc_rbdyn::Robot & robot)
{
  auto & mbc = robot.mbc();
  rbd::vectorToParam(robot.tau()->value(), robot.controlTorque());
  rbd::vectorToParam(robot.alphaD()->value(), robot.controlAcceleration());
  robot.eulerIntegration(dt_);
  Eigen::VectorXd alpha = tvm::dot(robot.q())->value();
  rbd::paramToVector(mbc.alpha, alpha);
  tvm::dot(robot.q())->set(alpha);
  Eigen::VectorXd q = robot.q()->value();
  rbd::paramToVector(mbc.q, q);
  robot.q()->set(q);
  robot.forwardKinematics();
  robot.forwardVelocity();
  robot.forwardAcceleration();
}

} // namespace mc_solver
