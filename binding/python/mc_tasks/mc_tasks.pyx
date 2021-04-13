# distutils: language = c++

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_tasks.c_mc_tasks as c_mc_tasks

cimport eigen.c_eigen as c_eigen
cimport eigen.eigen as eigen

cimport sva.sva as sva

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

cimport mc_solver.c_mc_solver as c_mc_solver
cimport mc_solver.mc_solver as mc_solver

cimport mc_tvm.c_mc_tvm as c_mc_tvm

from mc_rtc.c_mc_rtc cimport map as cppmap

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.memory cimport make_shared
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from libcpp cimport bool as cppbool

cdef class MetaTask(object):
  def __cinit__(self):
    pass
  def reset(self):
    assert(self.mt_base.get())
    deref(self.mt_base).reset()
  def dimWeight(self, eigen.VectorXd dimW = None):
    assert(self.mt_base.get())
    if dimW is None:
      return eigen.VectorXdFromC(deref(self.mt_base).dimWeight())
    else:
      deref(self.mt_base).dimWeight(dimW.impl)
  def selectActiveJoints(self, mc_solver.QPSolver solver, joints):
    assert(self.mt_base.get())
    deref(self.mt_base).selectActiveJoints(deref(solver.impl), joints)
  def selectInactiveJoints(self, mc_solver.QPSolver solver, joints):
    assert(self.mt_base.get())
    deref(self.mt_base).selectInactiveJoints(deref(solver.impl), joints)
  def resetJointsSelector(self, mc_solver.QPSolver solver):
    assert(self.mt_base.get())
    deref(self.mt_base).resetJointsSelector(deref(solver.impl))
  def eval(self):
    assert(self.mt_base.get())
    return eigen.VectorXdFromC(deref(self.mt_base).eval())
  def speed(self):
    assert(self.mt_base.get())
    return eigen.VectorXdFromC(deref(self.mt_base).speed())
  property name:
    def __get__(self):
      assert(self.mt_base.get())
      return deref(self.mt_base).name()
    def __set__(self, value):
      assert(self.mt_base.get())
      if isinstance(value, unicode):
        value = value.encode(u'ascii')
      deref(self.mt_base).name(value)

include "com_trajectory_task.pxi"
include "position_trajectory_task.pxi"
include "posture_trajectory_task.pxi"
include "orientation_trajectory_task.pxi"
include "spline_trajectory_task.pxi"
include "transform_trajectory_task.pxi"
include "vector_orientation_trajectory_task.pxi"

cdef class CoMTask(_CoMTrajectoryTask):
  def __cinit__(self, mc_rbdyn.Robot robot, double stiffness = 5.0, double weight = 100.0):
    self.impl = make_shared[c_mc_tasks.CoMTask](deref(robot.impl), stiffness, weight)
    self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.CoMFunction]](self.impl)
    self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)

  def com(self, eigen.Vector3d com = None):
    assert(self.impl.get())
    if com is None:
      return eigen.Vector3dFromC(deref(self.impl).com())
    else:
      deref(self.impl).com(com.impl)

cdef class PositionTask(_PositionTrajectoryTask):
  def __cinit__(self, mc_rbdyn.Frame frame, double stiffness = 2.0, double weight = 500.0):
    self.impl = make_shared[c_mc_tasks.PositionTask](deref(frame.impl), stiffness, weight)
    self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.PositionFunction]](self.impl)
    self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)

  def position(self, eigen.Vector3d pos = None):
    assert(self.impl.get())
    if pos is None:
      return eigen.Vector3dFromC(deref(self.impl).position())
    else:
      deref(self.impl).position(pos.impl)

cdef class OrientationTask(_OrientationTrajectoryTask):
  def __cinit__(self, mc_rbdyn.Frame frame, double stiffness = 2.0, double weight = 500.0):
    self.impl = make_shared[c_mc_tasks.OrientationTask](deref(frame.impl), stiffness, weight)
    self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.OrientationFunction]](self.impl)
    self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
  def orientation(self, eigen.Matrix3d ori = None):
    assert(self.impl.get())
    if ori is None:
      return eigen.Matrix3dFromC(deref(self.impl).orientation())
    else:
      deref(self.impl).orientation(ori.impl)

cdef class VectorOrientationTask(_VectorOrientationTrajectoryTask):
  def __cinit__(self, mc_rbdyn.Frame frame, eigen.Vector3d frameVector, double stiffness = 2.0, double weight = 500.0):
    self.impl = make_shared[c_mc_tasks.VectorOrientationTask](deref(frame.impl), frameVector.impl, stiffness, weight)
    self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.VectorOrientationFunction]](self.impl)
    self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
  def frameVector(self, eigen.Vector3d ori = None):
    assert(self.impl.get())
    if ori is None:
      return eigen.Vector3dFromC(deref(self.impl).frameVector())
    else:
      deref(self.impl).frameVector(ori.impl)
  def targetVector(self, eigen.Vector3d ori = None):
    assert(self.impl.get())
    if ori is None:
      return eigen.Vector3dFromC(deref(self.impl).targetVector())
    else:
      deref(self.impl).targetVector(ori.impl)

cdef class TransformTask(_TransformTrajectoryTask):
  def __cinit__(self, mc_rbdyn.Frame frame, double stiffness = 2.0, double weight = 500.0):
    self.impl = make_shared[c_mc_tasks.TransformTask](deref(frame.impl), stiffness, weight)
    self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]](self.impl)
    self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
  def target(self, pos = None):
    if pos is None:
      return sva.PTransformdFromC(deref(self.impl).target())
    else:
      if isinstance(pos, sva.PTransformd):
        deref(self.impl).target(deref((<sva.PTransformd>pos).impl))
      else:
        self.target(sva.PTransformd(pos))

cdef class BSplineTrajectoryTask(_TransformTrajectoryTask):
  def __cinit__(self, mc_rbdyn.Frame frame, double duration, double stiffness, double weight, sva.PTransformd target, posWaypoints = [], oriWaypoints = []):
    cdef VectorPairDoubleMatrix3d oriWp = VectorPairDoubleMatrix3d(oriWaypoints)
    cdef eigen.Vector3dVector posWp = eigen.Vector3dVector(posWaypoints)
    self.impl = make_shared[c_mc_tasks.BSplineTrajectoryTask](deref(frame.impl), duration, stiffness, weight, deref(target.impl), posWp.v, oriWp.impl)
    self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]](self.impl)
    self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
  def oriWaypoints(self, oriWp):
    assert(self.impl.get())
    deref(self.impl).oriWaypoints(VectorPairDoubleMatrix3d(oriWp).impl)
  def posWaypoints(self, posWp):
    assert(self.impl.get())
    deref(self.impl).posWaypoints(eigen.Vector3dVector(posWp).v)
  def target(self, pos = None):
    if pos is None:
      return sva.PTransformdFromC(deref(self.impl).target())
    else:
      if isinstance(pos, sva.PTransformd):
        deref(self.impl).target(deref((<sva.PTransformd>pos).impl))
      else:
        self.target(sva.PTransformd(pos))
  def timeElapsed(self):
    assert(self.impl.get())
    return deref(self.impl).timeElapsed()

cdef class ExactCubicTrajectoryTask(_TransformTrajectoryTask):
  def __cinit__(self, mc_rbdyn.Frame frame, double duration, double stiffness, double weight, sva.PTransformd target, posWaypoints = [], eigen.Vector3d initVel = eigen.Vector3d.Zero(), eigen.Vector3d initAcc = eigen.Vector3d.Zero(), eigen.Vector3d finalVel = eigen.Vector3d.Zero(), eigen.Vector3d finalAcc = eigen.Vector3d.Zero(), oriWaypoints = []):
    cdef VectorPairDoubleMatrix3d oriWp = VectorPairDoubleMatrix3d(oriWaypoints)
    cdef VectorPairDoubleVector3d posWp = VectorPairDoubleVector3d(posWaypoints)
    self.impl = make_shared[c_mc_tasks.ExactCubicTrajectoryTask](deref(frame.impl), duration, stiffness, weight, deref(target.impl), posWp.impl, initVel.impl, initAcc.impl, finalVel.impl, finalAcc.impl, oriWp.impl)
    self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]](self.impl)
    self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
  def posWaypoints(self, posWp):
    assert(self.impl.get())
    deref(self.impl).posWaypoints(VectorPairDoubleVector3d(posWp).impl)
  def oriWaypoints(self, oriWp):
    assert(self.impl.get())
    deref(self.impl).oriWaypoints(VectorPairDoubleMatrix3d(oriWp).impl)
  def constraints(self, eigen.Vector3d initVel, eigen.Vector3d initAcc, eigen.Vector3d endVel, eigen.Vector3d endAcc):
    deref(self.impl).constraints(initVel.impl, initAcc.impl, endVel.impl, endAcc.impl)
  def target(self, pos = None):
    if pos is None:
      return sva.PTransformdFromC(deref(self.impl).target())
    else:
      if isinstance(pos, sva.PTransformd):
        deref(self.impl).target(deref((<sva.PTransformd>pos).impl))
      else:
        self.target(sva.PTransformd(pos))

cdef class PostureTask(_PostureTrajectoryTask):
  def __cinit__(self, mc_rbdyn.Robot robot, double stiffness = 1.0, double weight = 10.0):
    self.impl = make_shared[c_mc_tasks.PostureTask](deref(robot.impl), stiffness, weight)
    self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.PostureFunction]](self.impl)
    self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
  def posture(self, target = None):
    cdef cppmap[string, vector[double]] tgt
    cdef pair[string, vector[double]] elem
    assert(self.impl.get())
    if target is None:
      return deref(self.impl).posture()
    else:
      deref(self.impl).posture(target)
  def setJointGains(self, name, stiffness, damping):
    assert(self.impl.get())
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    deref(self.impl).setJointGains(name, stiffness, damping)
  def jointStiffness(self, name, stiffness):
    assert(self.impl.get())
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    deref(self.impl).setJointStiffness(name, stiffness)
  def target(self, in_):
    assert(self.impl.get())
    c_mc_tasks.PostureTaskTarget(deref(self.impl), in_)
