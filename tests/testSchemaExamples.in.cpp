/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <boost/filesystem.hpp>
#include <boost/mpl/list.hpp>
#include <boost/test/unit_test.hpp>
namespace bfs = boost::filesystem;

#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/ExactCubicTrajectoryTask.h>
#include <mc_tasks/GazeTask.h>
#include <mc_tasks/LookAtFrameTask.h>
#include <mc_tasks/LookAtTask.h>
#include <mc_tasks/MomentumTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/PositionBasedVisServoTask.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/VectorOrientationTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include "utils.h"

static auto solver_ptr = makeSolver();
static auto & solver = *solver_ptr;

static const bfs::path EXAMPLE_PATH = "@EXAMPLE_PATH@";
static const bfs::path JSON_EXAMPLES = EXAMPLE_PATH / "json" / "MetaTask";
static const bfs::path YAML_EXAMPLES = EXAMPLE_PATH / "yaml" / "MetaTask";

#define TEST_TASK(TaskT, TaskN)                                        \
  BOOST_AUTO_TEST_CASE(TaskN)                                          \
  {                                                                    \
    const bfs::path jsonIn = JSON_EXAMPLES / #TaskN ".json";           \
    {                                                                  \
      mc_rtc::Configuration json(jsonIn.string());                     \
      auto task = mc_tasks::MetaTaskLoader::load<TaskT>(solver, json); \
      size_t nLogEntriesBefore = solver.logger().size();               \
      size_t nGUIEntriesBefore = solver.gui().size();                  \
      solver.addTask(task);                                            \
      solver.removeTask(task);                                         \
      BOOST_REQUIRE(nLogEntriesBefore == solver.logger().size());      \
      BOOST_REQUIRE(nGUIEntriesBefore == solver.gui().size());         \
    }                                                                  \
    const bfs::path yamlIn = YAML_EXAMPLES / #TaskN ".yaml";           \
    {                                                                  \
      mc_rtc::Configuration yaml(yamlIn.string());                     \
      auto task = mc_tasks::MetaTaskLoader::load<TaskT>(solver, yaml); \
    }                                                                  \
  }

TEST_TASK(mc_tasks::AddRemoveContactTask, AddContactTask)
TEST_TASK(mc_tasks::AddRemoveContactTask, RemoveContactTask)
TEST_TASK(mc_tasks::force::AdmittanceTask, AdmittanceTask)
TEST_TASK(mc_tasks::BSplineTrajectoryTask, BSplineTrajectoryTask)
TEST_TASK(mc_tasks::force::ComplianceTask, ComplianceTask)
TEST_TASK(mc_tasks::CoMTask, CoMTask)
TEST_TASK(mc_tasks::force::CoPTask, CoPTask)
TEST_TASK(mc_tasks::ExactCubicTrajectoryTask, ExactCubicTrajectoryTask)
TEST_TASK(mc_tasks::GazeTask, GazeTask)
TEST_TASK(mc_tasks::lipm_stabilizer::StabilizerTask, LIPMStabilizerTask)
TEST_TASK(mc_tasks::LookAtFrameTask, LookAtFrameTask)
TEST_TASK(mc_tasks::LookAtTask, LookAtTask)
TEST_TASK(mc_tasks::MomentumTask, MomentumTask)
TEST_TASK(mc_tasks::OrientationTask, OrientationTask)
TEST_TASK(mc_tasks::PositionBasedVisServoTask, PBVSTask)
TEST_TASK(mc_tasks::PositionTask, PositionTask)
TEST_TASK(mc_tasks::PostureTask, PostureTask)
TEST_TASK(mc_tasks::TransformTask, TransformTask)
TEST_TASK(mc_tasks::VectorOrientationTask, VectorOrientationTask)
