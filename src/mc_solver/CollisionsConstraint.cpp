/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CollisionsConstraint.h>

#include <mc_solver/ConstraintLoader.h>
#include <mc_solver/QPSolver.h>

#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/StateBuilder.h>

#include <tvm/task_dynamics/VelocityDamper.h>

namespace mc_solver
{

CollisionsConstraint::CollisionData::CollisionData(uint64_t id, const mc_rbdyn::Collision & col)
: id(id), collision(col), function(nullptr), task(nullptr), monitored(Monitoring::OFF)
{
}

CollisionsConstraint::CollisionsConstraint()
{
  name_ = "CollisionConstraint";
}

void CollisionsConstraint::addCollision(QPSolver & solver, const mc_rbdyn::Collision & col)
{
  auto & robots = solver.robots();
  auto & r1 = robots.robot(col.robot1);
  auto & r2 = robots.robot(col.robot2);

  auto handle_wildcard = [&, this](const mc_rbdyn::Robot & robot, std::string_view convex, bool is_c1) {
    if(!convex.size())
    {
      return false;
    }
    if(convex.back() != '*')
    {
      return false;
    }
    std::string_view search = convex.substr(0, convex.size() - 1);
    bool match = false;
    for(const auto & c : robot.convexes())
    {
      const auto & cName = c.first;
      if(cName.size() < search.size())
      {
        continue;
      }
      if(cName.substr(0, search.size()) == search)
      {
        match = true;
        auto nCol = col;
        auto & nObject = is_c1 ? nCol.object1 : nCol.object2;
        nObject = cName;
        addCollision(solver, nCol);
      }
    }
    if(!match)
    {
      mc_rtc::log::warning("No match found for collision wildcard {} in {}", convex, robot.name());
    }
    return true;
  };
  if(handle_wildcard(r1, col.object1, true) || handle_wildcard(r2, col.object2, false))
  {
    return;
  }
  auto & c1 = r1.convex(col.object1);
  auto & c2 = r2.convex(col.object2);
  auto it = getData(col);
  if(it != data_.end())
  {
    auto & data = *it;
    data.collision.iDist = col.iDist;
    data.collision.sDist = col.sDist;
    data.collision.damping = col.damping;
    if(inSolver_)
    {
      updateCollision(solver, data);
    }
  }
  else
  {
    auto & data = data_.emplace_back(nextId_++, col);
    data.function = std::make_shared<mc_tvm::CollisionFunction>(c1, c2, solver.dt());
    if(inSolver_)
    {
      addCollision(solver, data);
    }
  }
}

void CollisionsConstraint::addMonitorButton(QPSolver & solver, CollisionData & data)
{
  const auto & col = data.collision;
  auto cat = fmt::format("{}/{}", col.robot1, col.robot2);
  auto name = fmt::format("Monitor {}::{}", col.object1, col.object2);
  auto id = data.id;
  solver.gui().addElement(
      {"Collisions", cat, "Monitors"},
      mc_rtc::gui::Checkbox(
          name, [id, this]() { return getData(id).monitored != Monitoring::OFF; },
          [id, &solver, this]() { toggleCollisionMonitor(solver, getData(id), Monitoring::USER); }));
}

void CollisionsConstraint::removeMonitorButton(QPSolver & solver, CollisionData & data)
{
  const auto & col = data.collision;
  auto cat = fmt::format("{}/{}", col.robot1, col.robot2);
  auto name = fmt::format("Monitor {}::{}", col.object1, col.object2);
  solver.gui().removeElement({"Collisions", cat, "Monitors"}, name);
}

void CollisionsConstraint::toggleCollisionMonitor(QPSolver & solver, CollisionData & data, Monitoring m)
{
  const auto & col = data.collision;
  auto & gui = solver.gui();
  auto cat = fmt::format("{}/{}", col.robot1, col.robot2);
  auto label = fmt::format("{}::{}", col.object1, col.object2);
  if(data.monitored > Monitoring::OFF)
  {
    // Remove the monitor
    gui.removeElement({"Collisions", cat}, label);
    gui.removeElement({"Collisions", cat, "Arrows"}, label);
    data.monitored = Monitoring::OFF;
  }
  else
  {
    if(data.monitored != Monitoring::OFF)
    {
      data.monitored = m;
      return;
    }
    // Add the monitor
    auto collId = data.id;
    gui.addElement({"Collisions", cat}, mc_rtc::gui::Label(label, [this, collId]() {
                     auto & data = getData(collId);
                     return fmt::format("{:0.2f} cm", data.function->distance() * 100);
                   }));
    gui.addElement({"Collisions", cat, "Arrows"},
                   mc_rtc::gui::Arrow(
                       label, [this, collId]() -> const Eigen::Vector3d & { return getData(collId).function->p1(); },
                       [this, collId]() -> const Eigen::Vector3d & { return getData(collId).function->p2(); }));
    data.monitored = m;
  }
}

void CollisionsConstraint::addCollisions(QPSolver & solver, const mc_rbdyn::CollisionVector & collisions)
{
  for(const auto & col : collisions)
  {
    addCollision(solver, col);
  }
}

void CollisionsConstraint::removeCollisions(QPSolver & solver, const mc_rbdyn::CollisionVector & cols)
{
  for(const auto & col : cols)
  {
    auto it = getData(col);
    if(it == data_.end())
    {
      continue;
    }
    removeCollision(solver, *it);
    data_.erase(it);
  }
}

void CollisionsConstraint::removeCollisions(QPSolver & solver, std::string_view r1, std::string_view r2)
{
  auto remove = [&](auto & d) {
    auto & col = d.collision;
    if((col.robot1 == r1 && col.robot2 == r2) || (col.robot1 == r2 && col.robot2 == r1))
    {
      removeCollision(solver, d);
      return true;
    }
    return false;
  };
  data_.erase(std::remove_if(data_.begin(), data_.end(), [&](auto & d) { return remove(d); }), data_.end());
}

void CollisionsConstraint::reset(QPSolver & solver)
{
  for(auto & col : data_)
  {
    removeCollision(solver, col);
  }
  data_.clear();
}

void CollisionsConstraint::addToSolver(QPSolver & solver)
{
  if(!inSolver_)
  {
    inSolver_ = true;
    for(auto & d : data_)
    {
      addCollision(solver, d);
    }
  }
}

void CollisionsConstraint::removeFromSolver(QPSolver & solver)
{
  if(inSolver_)
  {
    inSolver_ = false;
    for(auto & d : data_)
    {
      removeCollision(solver, d);
    }
  }
}

void CollisionsConstraint::update(QPSolver & solver)
{
  for(auto & d : data_)
  {
    d.function->tick();
    if(d.monitored == Monitoring::OFF && d.function->distance() <= d.collision.iDist)
    {
      toggleCollisionMonitor(solver, d, Monitoring::AUTO);
    }
    if(d.monitored == Monitoring::AUTO && d.function->distance() > d.collision.iDist)
    {
      toggleCollisionMonitor(solver, d, Monitoring::OFF);
    }
  }
}

auto CollisionsConstraint::getData(const mc_rbdyn::Collision & col) -> std::vector<CollisionData>::iterator
{
  return std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.collision == col; });
}

auto CollisionsConstraint::getData(uint64_t id) -> CollisionData &
{
  return *std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.id == id; });
}

void CollisionsConstraint::addCollision(QPSolver & solver, CollisionData & data)
{
  const auto & col = data.collision;
  data.task = solver.problem().add(
      data.function >= 0.,
      tvm::task_dynamics::VelocityDamper(solver.dt(), {col.iDist, col.sDist, col.damping, defaultDampingOffset},
                                         tvm::constant::big_number),
      {tvm::requirements::PriorityLevel(0)});
  mc_rtc::log::info("Added collision {}::{}/{}::{} (iDist: {}, sDist: {})", col.robot1, col.object1, col.robot2,
                    col.object2, col.iDist, col.sDist);
  addMonitorButton(solver, data);
}

void CollisionsConstraint::updateCollision(QPSolver & solver, CollisionData & data)
{
  const auto & col = data.collision;
  if(data.task)
  {
    solver.problem().remove(*data.task);
  }
  data.task = solver.problem().add(
      data.function >= 0.,
      tvm::task_dynamics::VelocityDamper(solver.dt(), {col.iDist, col.sDist, col.damping, defaultDampingOffset},
                                         tvm::constant::big_number),
      {tvm::requirements::PriorityLevel(0)});
  mc_rtc::log::info("Updated collision {}::{}/{}::{} (iDist: {}, sDist: {})", col.robot1, col.object1, col.robot2,
                    col.object2, col.iDist, col.sDist);
}

void CollisionsConstraint::removeCollision(QPSolver & solver, CollisionData & data)
{
  const auto & col = data.collision;
  if(data.monitored > Monitoring::OFF)
  {
    toggleCollisionMonitor(solver, data, Monitoring::OFF);
  }
  removeMonitorButton(solver, data);
  if(data.task)
  {
    solver.problem().remove(*data.task);
  }
  data.task.reset();
  mc_rtc::log::info("Removed collision {}::{}/{}::{}", col.robot1, col.object1, col.robot2, col.object2);
}

std::vector<mc_rbdyn::Collision> CollisionsConstraint::collisions() const noexcept
{
  std::vector<mc_rbdyn::Collision> out;
  for(const auto & d : data_)
  {
    out.push_back(d.collision);
  }
  return out;
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintLoader::register_load_function(
    "collision",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto ret = std::make_shared<mc_solver::CollisionsConstraint>();
      auto & r1 = solver.robots().fromConfig(config, "CollisionsConstraint", false, "r1Index", "r1");
      auto & r2 = solver.robots().fromConfig(config, "CollisionsConstraint", false, "r2Index", "r2");
      if(r1.name() == r2.name())
      {
        bool useCommon = config("useCommon", false);
        if(useCommon)
        {
          ret->addCollisions(solver, {r1.name(), r1.name(), r1.module().commonSelfCollisions()});
        }
        bool useMinimal = config("useMinimal", false);
        if(useMinimal)
        {
          ret->addCollisions(solver, {r1.name(), r1.name(), r1.module().minimalSelfCollisions()});
        }
      }
      {
        auto collisions = config("collisions", std::vector<mc_rbdyn::CollisionDescription>{});
        ret->addCollisions(solver, {r1.name(), r2.name(), collisions});
      }
      return ret;
    });
}
