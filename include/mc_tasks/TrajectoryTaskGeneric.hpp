/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <cmath>

namespace mc_tasks
{

template<typename T>
TrajectoryTaskGeneric<T>::TrajectoryTaskGeneric(mc_rbdyn::Robot & robot, double stiffness, double w)
: robot_(robot), stiffness_(Eigen::VectorXd::Constant(1, stiffness)),
  damping_(Eigen::VectorXd::Constant(1, 2 * std::sqrt(stiffness))), weight_(w)
{
}

template<typename T>
TrajectoryTaskGeneric<T>::~TrajectoryTaskGeneric()
{
}

template<typename T>
template<typename... Args>
void TrajectoryTaskGeneric<T>::finalize(Args &&... args)
{
  errorT_ = std::make_shared<T>(args...);
  stiffness_ = Eigen::VectorXd::Constant(errorT_->size(), 1, stiffness_(0));
  damping_ = Eigen::VectorXd::Constant(errorT_->size(), 1, damping_(0));
  dimWeight_ = Eigen::VectorXd::Ones(errorT_->size(), 1);
}

template<typename T>
void TrajectoryTaskGeneric<T>::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(task_)
  {
    solver.problem().remove(task_.get());
    task_.reset();
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::addToSolver(mc_solver::QPSolver & solver)
{
  if(!task_)
  {
    auto addTask = [&, this](auto & error) {
      task_ = solver.problem().add(error == 0., tvm::task_dynamics::PD(stiffness_, damping_),
                                   {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(weight_)});
    };
    if(selectorT_)
    {
      addTask(selectorT_);
    }
    else
    {
      addTask(errorT_);
    }
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::reset()
{
  errorT_->reset();
  if constexpr(hasRefVel)
  {
    refVel(Eigen::VectorXd::Zero(refVel().size()));
  }
  if constexpr(hasRefAccel)
  {
    refAccel(Eigen::VectorXd::Zero(refAccel().size()));
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::update(mc_solver::QPSolver &)
{
}

template<typename T>
void TrajectoryTaskGeneric<T>::setGains(const Eigen::VectorXd & stiffness, const Eigen::VectorXd & damping)
{
  stiffness_ = stiffness;
  damping_ = damping;
  if(task_)
  {
    auto & PDImpl = static_cast<tvm::task_dynamics::PD::Impl &>(*task_->task.taskDynamics());
    PDImpl.gains(stiffness_, damping_);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::weight(double w)
{
  weight_ = w;
  if(task_)
  {
    // FIXME Currently not possible
    // task_->requirements.weight(weight_);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::dimWeight(const Eigen::VectorXd & w)
{
  dimWeight_ = w;
  if(task_)
  {
    // FIXME Currently not possible
    // task_->requirements.anisotropicWeight(w);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectActiveJoints(
    const std::vector<std::string> & activeJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & /*activeDofs*/,
    bool checkJoints)
{
  if(task_)
  {
    mc_rtc::log::warning("selectActiveJoints(names) ignored: use selectActiveJoints(solver, names) for a task already "
                         "added to the solver");
    return;
  }
  if(checkJoints)
  {
    ensureHasJoints(*robot_, activeJointsName, "[" + name() + "::selectActiveJoints]");
  }
  selectorT_ = mc_tvm::JointsSelectorFunction::ActiveJoints(errorT_, *robot_, activeJointsName);
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectActiveJoints(
    mc_solver::QPSolver & solver,
    const std::vector<std::string> & activeJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  ensureHasJoints(*robot_, activeJointsName, "[" + name() + "::selectActiveJoints]");
  if(task_)
  {
    removeFromSolver(solver);
    selectActiveJoints(activeJointsName, activeDofs, false);
    addToSolver(solver);
  }
  else
  {
    selectActiveJoints(activeJointsName, activeDofs, false);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectInactiveJoints(
    const std::vector<std::string> & inactiveJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & /*unactiveDofs*/,
    bool checkJoints)
{
  if(task_)
  {
    mc_rtc::log::warning(
        "{}::selectInactiveJoints(names) ignored: use selectInactiveJoints(solver, names) for a task already added "
        "to the solver",
        name());
    return;
  }
  if(checkJoints)
  {
    ensureHasJoints(*robot_, inactiveJointsName, "[" + name() + "::selectInactiveJoints]");
  }
  selectorT_ = mc_tvm::JointsSelectorFunction::InactiveJoints(errorT_, *robot_, inactiveJointsName);
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectInactiveJoints(
    mc_solver::QPSolver & solver,
    const std::vector<std::string> & inactiveJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & inactiveDofs)
{
  ensureHasJoints(*robot_, inactiveJointsName, "[" + name() + "::selectInactiveJoints]");
  if(task_)
  {
    removeFromSolver(solver);
    selectInactiveJoints(inactiveJointsName, inactiveDofs, false);
    addToSolver(solver);
  }
  else
  {
    selectInactiveJoints(inactiveJointsName, inactiveDofs, false);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::resetJointsSelector()
{
  if(task_)
  {
    mc_rtc::log::warning(
        "{}::resetJointsSelector() ignored: use resetJointsSelector(solver) for a task already added to the solver",
        name());
    return;
  }
  selectorT_.reset();
}

template<typename T>
void TrajectoryTaskGeneric<T>::resetJointsSelector(mc_solver::QPSolver & solver)
{
  if(task_)
  {
    removeFromSolver(solver);
    resetJointsSelector();
    addToSolver(solver);
  }
  else
  {
    resetJointsSelector();
  }
}

template<typename T>
Eigen::VectorXd TrajectoryTaskGeneric<T>::eval() const
{
  return errorT_->value().cwiseProduct(dimWeight_);
}

template<typename T>
Eigen::VectorXd TrajectoryTaskGeneric<T>::speed() const
{
  return errorT_->velocity().cwiseProduct(dimWeight_);
}

template<typename T>
Eigen::VectorXd TrajectoryTaskGeneric<T>::normalAcc() const
{
  return errorT_->normalAcceleration().cwiseProduct(dimWeight_);
}

template<typename T>
void TrajectoryTaskGeneric<T>::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("stiffness"))
  {
    auto s = config("stiffness");
    if(s.size())
    {
      Eigen::VectorXd stiff = s;
      stiffness(stiff);
    }
    else
    {
      stiffness(static_cast<double>(s));
    }
  }
  if(config.has("damping"))
  {
    auto d = config("damping");
    if(d.size())
    {
      setGains(dimStiffness(), d);
    }
    else
    {
      setGains(stiffness(), d);
    }
  }
  if(config.has("weight"))
  {
    weight(config("weight"));
  }
  if constexpr(hasRefVel)
  {
    if(config.has("refVel"))
    {
      refVel(config("refVel"));
    }
  }
  if constexpr(hasRefAccel)
  {
    if(config.has("refAccel"))
    {
      refAccel(config("refAccel"));
    }
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  if constexpr(hasRefVel)
  {
    gui.addElement({"Tasks", name_},
                   mc_rtc::gui::ArrayInput("refVel", [this]() -> refVel_return_t { return this->refVel(); },
                                           [this](const refVel_t & v) { this->refVel(v); }));
  }
  if constexpr(hasRefAccel)
  {
    gui.addElement({"Tasks", name_},
                   mc_rtc::gui::ArrayInput("refAccel", [this]() -> refAccel_return_t { return this->refAccel(); },
                                           [this](const refAccel_t & v) { this->refAccel(v); }));
  }
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput("stiffness", [this]() { return this->stiffness(); },
                                          [this](const double & s) { this->setGains(s, this->damping()); }),
                 mc_rtc::gui::NumberInput("damping", [this]() { return this->damping(); },
                                          [this](const double & d) { this->setGains(this->stiffness(), d); }),
                 mc_rtc::gui::NumberInput("stiffness & damping", [this]() { return this->stiffness(); },
                                          [this](const double & g) { this->stiffness(g); }),
                 mc_rtc::gui::NumberInput("weight", [this]() { return this->weight(); },
                                          [this](const double & w) { this->weight(w); }));
  gui.addElement(
      {"Tasks", name_, "Gains", "Dimensional"},
      mc_rtc::gui::ArrayInput("stiffness", [this]() { return this->dimStiffness(); },
                              [this](const Eigen::VectorXd & v) { this->setGains(v, this->dimDamping()); }),
      mc_rtc::gui::ArrayInput("damping", [this]() { return this->dimDamping(); },
                              [this](const Eigen::VectorXd & v) { this->setGains(this->dimStiffness(), v); }),
      mc_rtc::gui::ArrayInput("stiffness & damping", [this]() { return this->dimStiffness(); },
                              [this](const Eigen::VectorXd & v) { this->stiffness(v); }));
}

template<typename T>
void TrajectoryTaskGeneric<T>::addToLogger(mc_rtc::Logger & logger)
{
  // clang-format off
  logger.addLogEntries(this,
                       name_ + "_damping", [this]() { return damping_(0); },
                       name_ + "_stiffness", [this]() { return stiffness_(0); });
  // clang-format on
  MC_RTC_LOG_HELPER(name_ + "_dimWeight", dimWeight_);
  MC_RTC_LOG_HELPER(name_ + "_dimDamping", damping_);
  MC_RTC_LOG_HELPER(name_ + "_dimStiffness", stiffness_);
  if constexpr(hasRefVel)
  {
    logger.addLogEntry(name_ + "_refVel", this, [this]() -> refVel_return_t { return refVel(); });
  }
  if constexpr(hasRefAccel)
  {
    logger.addLogEntry(name_ + "_refAccel", this, [this]() -> refAccel_return_t { return refAccel(); });
  }
}

} // namespace mc_tasks
