/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/PostureTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/NumberSlider.h>

namespace mc_tasks
{

PostureTask::PostureTask(mc_rbdyn::Robot & robot, double stiffness, double weight)
: TrajectoryBase(robot, stiffness, weight)
{
  finalize(robot);
  type_ = "posture";
  name_ = "posture_" + robot.name();
  for(const auto & j : robot.mb().joints())
  {
    if(j.isMimic())
    {
      mimics_[j.mimicName()].push_back(robot.jointIndexByName(j.name()));
    }
  }
}

void PostureTask::reset()
{
  TrajectoryBase::reset();
  errorT_->posture(robot_->mbc().q);
}

void PostureTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("posture"))
  {
    this->posture(config("posture"));
  }
  if(config.has("jointGains"))
  {
    // FIXME Adapt
  }
  if(config.has("target"))
  {
    this->target(config("target"));
  }
  if(config.has("stiffness"))
  {
    this->stiffness(static_cast<double>(config("stiffness")));
  }
  if(config.has("damping"))
  {
    this->damping(static_cast<double>(config("damping")));
  }
  if(config.has("weight"))
  {
    this->weight(config("weight"));
  }
}

void PostureTask::target(const mc_rtc::map<std::string, std::vector<double>> & joints)
{
  const auto & mb = robot_->mb();
  auto q = posture();
  for(const auto & j : joints)
  {
    if(robot_->hasJoint(j.first))
    {
      auto jIdx = robot_->jointIndexByName(j.first);
      if(static_cast<size_t>(mb.joint(jIdx).dof()) == j.second.size())
      {
        q[jIdx] = j.second;
        if(mimics_.count(j.first))
        {
          for(const auto & m : mimics_[j.first])
          {
            const auto & mimic = mb.joint(m);
            std::transform(q[jIdx].begin(), q[jIdx].end(), q[m].begin(),
                           [&](const auto & ji) { return mimic.mimicMultiplier() * ji + mimic.mimicOffset(); });
          }
        }
      }
      else
      {
        mc_rtc::log::error("{}::target dof missmatch for {}", name_, j.first);
      }
    }
    else
    {
      mc_rtc::log::error("{}::target no joint named {}", name_, j.first);
    }
  }
  posture(q);
}

void PostureTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_eval", this, [this]() -> const Eigen::VectorXd & { return errorT_->value(); });
  logger.addLogEntry(name_ + "_speed", this, [this]() -> const Eigen::VectorXd & { return errorT_->velocity(); });
}

void PostureTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  std::vector<std::string> active_gripper_joints;
  for(const auto & g : robot_->grippers())
  {
    for(const auto & n : g.get().activeJoints())
    {
      active_gripper_joints.push_back(n);
    }
  }
  auto isActiveGripperJoint = [&](const std::string & j) {
    return std::find(active_gripper_joints.begin(), active_gripper_joints.end(), j) != active_gripper_joints.end();
  };
  for(const auto & j : robot_->mb().joints())
  {
    if(j.dof() != 1 || j.isMimic() || isActiveGripperJoint(j.name()))
    {
      continue;
    }
    auto jIndex = robot_->jointIndexByName(j.name());
    auto jIndexInParam = robot_->mb().jointPosInParam(jIndex);
    bool isContinuous = robot_->limits().ql[jIndexInParam] == -std::numeric_limits<double>::infinity();
    auto updatePosture = [this](unsigned int jIndex, double v) {
      auto posture_ = this->posture();
      posture_[jIndex][0] = v;
      const auto & jName = robot_->mb().joint(static_cast<int>(jIndex)).name();
      if(mimics_.count(jName))
      {
        for(auto ji : mimics_.at(jName))
        {
          const auto & mimic = robot_->mb().joint(ji);
          posture_[static_cast<size_t>(ji)][0] = mimic.mimicMultiplier() * v + mimic.mimicOffset();
        }
      }
      posture(posture_);
    };
    if(isContinuous)
    {
      gui.addElement({"Tasks", name_, "Target"},
                     mc_rtc::gui::NumberInput(j.name(), [this, jIndex]() { return posture()[jIndex][0]; },
                                              [jIndex, updatePosture](double v) { updatePosture(jIndex, v); }));
    }
    else
    {
      gui.addElement({"Tasks", name_, "Target"},
                     mc_rtc::gui::NumberSlider(j.name(), [this, jIndex]() { return posture()[jIndex][0]; },
                                               [jIndex, updatePosture](double v) { updatePosture(jIndex, v); },
                                               robot_->limits().ql[jIndexInParam], robot_->limits().qu[jIndexInParam]));
    }
  }
}

void PostureTask::setJointGains(const std::string & name, double stiffness, double damping)
{
  if(!robot().hasJoint(name))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Cannot specify gains for non-existing joint {}",
                                                     this->name(), name);
  }
  auto jIdx = robot().jointIndexByName(name);
  auto gainIdx = robot().mb().jointPosInDof(static_cast<int>(jIdx)) - robot().mb().joint(0).dof();
  auto stiff = dimStiffness();
  stiff(gainIdx) = stiffness;
  auto damp = dimDamping();
  damp(gainIdx) = damping;
  setGains(stiff, damp);
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "posture",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::PostureTask>(solver.robots().fromConfig(config, "PostureTask"));
      t->load(solver, config);
      return t;
    });
}
