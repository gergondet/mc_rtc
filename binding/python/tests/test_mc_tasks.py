#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

import mc_tasks
import mc_rbdyn
import mc_rtc
import eigen as e

from nose import with_setup

mc_rtc.Loader.debug_suffix("")

class TestMCTasks():

  @classmethod
  def setup_class(self):
    self.robots = mc_rbdyn.Robots()
    mc_rbdyn.RobotLoader.clear()
    mc_rbdyn.RobotLoader.update_robot_module_path(["$<TARGET_FILE_DIR:jvrc1>"])
    self.rm = mc_rbdyn.get_robot_module("JVRC1")
    self.robots.load(self.rm, self.rm.name)
    self.robot = self.robots.robot()

  @classmethod
  def teardown_class(self):
    pass

  def test_comTask(self):
    comTask1 = mc_tasks.CoMTask(self.robot)
    comTask2 = mc_tasks.CoMTask(self.robot, 5.0, 100)
    comTask3 = mc_tasks.CoMTask(self.robot, weight=100.0, stiffness=5.0)

    assert(comTask1.stiffness() == comTask2.stiffness())
    assert(comTask1.stiffness() == comTask3.stiffness())
    assert(comTask1.weight() == comTask2.weight())
    assert(comTask1.weight() == comTask3.weight())

  def test_posTask(self):
    posTask1 = mc_tasks.PositionTask(self.robot.frame("r_wrist"))
    posTask2 = mc_tasks.PositionTask(self.robot.frame("r_wrist"), 2.0, 500)
    posTask3 = mc_tasks.PositionTask(self.robot.frame("r_wrist"), weight=500.0, stiffness=2.0)

    assert(posTask1.stiffness() == posTask2.stiffness())
    assert(posTask1.stiffness() == posTask3.stiffness())
    assert(posTask1.weight() == posTask2.weight())
    assert(posTask1.weight() == posTask3.weight())

  def test_oriTask(self):
    oriTask1 = mc_tasks.OrientationTask(self.robot.frame("r_wrist"))
    oriTask2 = mc_tasks.OrientationTask(self.robot.frame("r_wrist"), 2.0, 500)
    oriTask3 = mc_tasks.OrientationTask(self.robot.frame("r_wrist"), weight=500.0, stiffness=2.0)

    assert(oriTask1.stiffness() == oriTask2.stiffness())
    assert(oriTask1.stiffness() == oriTask3.stiffness())
    assert(oriTask1.weight() == oriTask2.weight())
    assert(oriTask1.weight() == oriTask3.weight())

  def test_vecOriTask(self):
    vecTask1 = mc_tasks.VectorOrientationTask(self.robot.frame("r_wrist"), e.Vector3d(0., 0., 1.))
    vecTask2 = mc_tasks.VectorOrientationTask(self.robot.frame("r_wrist"), e.Vector3d(0., 0., 1.), 2.0, 500)
    vecTask3 = mc_tasks.VectorOrientationTask(self.robot.frame("r_wrist"), e.Vector3d(0., 0., 1.), weight=500.0, stiffness=2.0)

    assert(vecTask1.stiffness() == vecTask2.stiffness())
    assert(vecTask1.stiffness() == vecTask3.stiffness())
    assert(vecTask1.weight() == vecTask2.weight())
    assert(vecTask1.weight() == vecTask3.weight())

  def test_complianceTask(self):
    dof = e.Vector6d(*6*[1])
    compTask1 = mc_tasks.force.ComplianceTask(self.robot.frame("R_WRIST_Y_S"))
    compTask2 = mc_tasks.force.ComplianceTask(self.robot.frame("R_WRIST_Y_S"), dof, 2.0, 1000, 3., 1., mc_tasks.force.ComplianceTask.defaultFGain, mc_tasks.force.ComplianceTask.defaultTGain)
    compTask3 = mc_tasks.force.ComplianceTask(self.robot.frame("R_WRIST_Y_S"), weight=1000.0, stiffness=2.0, forceGain = mc_tasks.force.ComplianceTask.defaultFGain, torqueThreshold = 1., torqueGain = mc_tasks.force.ComplianceTask.defaultTGain, forceThreshold = 3.)

  def test_transformTask(self):
    transformTask1 = mc_tasks.TransformTask(self.robot.frame("LeftFoot"))
    transformTask2 = mc_tasks.TransformTask(self.robot.frame("LeftFoot"), 2.0, 500.0)

    assert(transformTask1.stiffness() == transformTask2.stiffness())
    assert(transformTask1.weight() == transformTask2.weight())

  def test_admittanceTask(self):
    admTask1 = mc_tasks.force.AdmittanceTask(self.robot.frame("LeftFoot"))
    admTask2 = mc_tasks.force.AdmittanceTask(self.robot.frame("LeftFoot"), 5.0, 1000.0)

    assert(admTask1.stiffness() == admTask2.stiffness())
    assert(admTask1.weight() == admTask2.weight())
