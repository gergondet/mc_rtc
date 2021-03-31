#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include <spdlog/spdlog.h>

#include "benchmark/benchmark.h"

class AllocTaskFixture : public benchmark::Fixture
{
public:
  AllocTaskFixture()
  {
    spdlog::set_level(spdlog::level::err);
    auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
    solver.robots().load(*rm, rm->name);
    solver.realRobots().load(*rm, rm->name);
  }

  void SetUp(const ::benchmark::State &) {}

  void TearDown(const ::benchmark::State &) {}

  mc_solver::QPSolver solver{std::make_shared<mc_rbdyn::Robots>(), std::make_shared<mc_rbdyn::Robots>(),
                             std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED, "", ""),
                             std::make_shared<mc_rtc::GUI>(), 0.005};
};

BENCHMARK_F(AllocTaskFixture, AllocTransformTask)(benchmark::State & state)
{
  for(auto _ : state)
  {
    auto task = std::make_shared<mc_tasks::TransformTask>(solver.robots().robot().frame("LeftFoot"));
  }
}

BENCHMARK_F(AllocTaskFixture, TransformTaskFromConfig)(benchmark::State & state)
{
  mc_rtc::Configuration config("@CMAKE_CURRENT_SOURCE_DIR@/config.yaml");
  for(auto _ : state)
  {
    auto task = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(solver, config);
  }
}

BENCHMARK_F(AllocTaskFixture, StabilizerTask)(benchmark::State & state)
{
  for(auto _ : state)
  {
    auto & robot = solver.robots().robot();
    const auto & real = solver.realRobots().robot();
    auto task = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(robot, real, "LeftFoot", "RightFoot",
                                                                            "WAIST_R_S", solver.dt());
  }
}

BENCHMARK_F(AllocTaskFixture, StabilizerTaskFromConfig)(benchmark::State & state)
{
  mc_rtc::Configuration config("@CMAKE_CURRENT_SOURCE_DIR@/config_lipm.yaml");
  for(auto _ : state)
  {
    auto task = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(solver, config);
  }
}

BENCHMARK_MAIN();
