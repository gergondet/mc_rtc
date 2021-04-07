/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/BoundedSpeedConstr.h>

#include <mc_solver/ConstraintLoader.h>
#include <mc_solver/QPSolver.h>

#include <mc_rtc/deprecated.h>

#include <tvm/task_dynamics/Proportional.h>

namespace mc_solver
{

BoundedSpeedConstr::BoundedSpeedData::BoundedSpeedData(mc_tvm::FrameVelocityPtr fn,
                                                       const Eigen::Vector6d & lowerSpeed,
                                                       const Eigen::Vector6d & upperSpeed)
: fn(fn), lowerSpeed(lowerSpeed), upperSpeed(upperSpeed)
{
}

BoundedSpeedConstr::BoundedSpeedConstr()
{
  name_ = "BoundedSpeedConstr";
}

void BoundedSpeedConstr::addToSolver(QPSolver & solver)
{
  if(!inSolver_)
  {
    inSolver_ = true;
    for(auto & d : data_)
    {
      addBoundedSpeed(solver, d);
    }
  }
}

void BoundedSpeedConstr::removeFromSolver(QPSolver & solver)
{
  if(inSolver_)
  {
    inSolver_ = false;
    for(auto & d : data_)
    {
      removeBoundedSpeed(solver, d);
    }
  }
}

void BoundedSpeedConstr::reset(QPSolver & solver)
{
  if(inSolver_)
  {
    for(auto & d : data_)
    {
      removeBoundedSpeed(solver, d);
    }
    data_.clear();
  }
}

void BoundedSpeedConstr::addBoundedSpeed(QPSolver & solver,
                                         mc_rbdyn::Frame & frame,
                                         const Eigen::Vector6d & dof,
                                         const Eigen::Vector6d & speed)
{
  addBoundedSpeed(solver, frame, dof, speed, speed);
}

void BoundedSpeedConstr::addBoundedSpeed(QPSolver & solver,
                                         mc_rbdyn::Frame & frame,
                                         const Eigen::Vector6d & dof,
                                         const Eigen::Vector6d & lowerSpeed,
                                         const Eigen::Vector6d & upperSpeed)
{
  auto it = getData(frame);
  if(it == data_.end())
  {
    auto & d = data_.emplace_back(std::make_shared<mc_tvm::FrameVelocity>(frame, dof), lowerSpeed, upperSpeed);
    if(inSolver_)
    {
      addBoundedSpeed(solver, d);
    }
  }
  else
  {
    auto & d = *it;
    if(d.fn->dof() != dof || d.lowerSpeed != lowerSpeed || d.upperSpeed != upperSpeed)
    {
      d.fn->dof() = dof;
      d.lowerSpeed = lowerSpeed;
      d.upperSpeed = upperSpeed;
      if(inSolver_)
      {
        updateBoundedSpeed(solver, d);
      }
    }
  }
}

bool BoundedSpeedConstr::removeBoundedSpeed(QPSolver & solver, mc_rbdyn::Frame & frame)
{
  auto it = getData(frame);
  if(it == data_.end())
  {
    return false;
  }
  if(inSolver_)
  {
    removeBoundedSpeed(solver, *it);
  }
  data_.erase(it);
  return true;
}

void BoundedSpeedConstr::addBoundedSpeed(QPSolver & solver, BoundedSpeedData & data)
{
  auto lower = data.fn->dof().cwiseProduct(data.lowerSpeed) / solver.dt();
  auto upper = data.fn->dof().cwiseProduct(data.upperSpeed) / solver.dt();
  data.task = solver.problem().add(lower <= data.fn <= upper, tvm::task_dynamics::Proportional(1 / solver.dt()),
                                   {tvm::requirements::PriorityLevel(0)});
  mc_rtc::log::info("Added bounded speed constraint: {} <= {} <= {} (dof: {}", data.lowerSpeed.transpose(),
                    data.fn->frame().name(), data.upperSpeed.transpose(), data.fn->dof().transpose());
}

void BoundedSpeedConstr::updateBoundedSpeed(QPSolver & solver, BoundedSpeedData & data)
{
  // FIXME Later we can update data.task
  removeBoundedSpeed(solver, data);
  addBoundedSpeed(solver, data);
}

void BoundedSpeedConstr::removeBoundedSpeed(QPSolver & solver, BoundedSpeedData & data)
{
  solver.problem().remove(data.task.get());
}

auto BoundedSpeedConstr::getData(const mc_rbdyn::Frame & frame) -> std::vector<BoundedSpeedData>::iterator
{
  return std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.fn->frame() == frame; });
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintLoader::register_load_function(
    "boundedSpeed",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto ret = std::make_shared<mc_solver::BoundedSpeedConstr>();
      if(config.has("constraints"))
      {
        for(const auto & c : config("constraints"))
        {
          auto & robot = solver.robots().fromConfig(c, "BoundedSpeedConstr");
          auto getFrame = [&]() -> mc_rbdyn::Frame & {
            if(c.has("body"))
            {
              mc_rtc::log::deprecated("BoundedSpeedConstraint", "body", "frame");
              std::string bodyName = c("body");
              auto bPoint = c("bodyPoint", Eigen::Vector3d::Zero().eval());
              if(bPoint != Eigen::Vector3d::Zero())
              {
                size_t i = 0;
                while(robot.hasFrame(fmt::format("{}_bSpeed_{}", bodyName, ++i)))
                  ;
                return robot.makeFrame(fmt::format("{}_bSpeed_{}", bodyName, i), bodyName, bPoint);
              }
              return robot.frame(bodyName);
            }
            return robot.frame(c("frame"));
          };
          auto & frame = getFrame();
          Eigen::Vector6d dof = c("dof", Eigen::Vector6d::Ones().eval());
          if(c.has("speed"))
          {
            ret->addBoundedSpeed(solver, frame, dof, c("speed"));
          }
          else if(c.has("lowerSpeed"))
          {
            assert(c.has("upperSpeed"));
            ret->addBoundedSpeed(solver, frame, dof, c("lowerSpeed"), c("upperSpeed"));
          }
          else
          {
            mc_rtc::log::error("No speed or lowerSpeed/upperSpeed entry for bounded speed constraint on {}",
                               frame.name());
          }
        }
      }
      return ret;
    });
}
