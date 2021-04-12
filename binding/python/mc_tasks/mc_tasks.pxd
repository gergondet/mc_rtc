#
# Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_mc_tasks

cimport mc_tvm.c_mc_tvm as c_mc_tvm

from libcpp cimport bool as cppbool
from libcpp.memory cimport shared_ptr

cdef class MetaTask(object):
  cdef c_mc_tasks.MetaTaskPtr mt_base

cdef class _PostureTrajectoryTask(MetaTask):
  cdef shared_ptr[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.PostureFunction]] ttg_base

cdef class _CoMTrajectoryTask(MetaTask):
  cdef shared_ptr[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.CoMFunction]] ttg_base

cdef class _PositionTrajectoryTask(MetaTask):
  cdef shared_ptr[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.PositionFunction]] ttg_base

cdef class _OrientationTrajectoryTask(MetaTask):
  cdef shared_ptr[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.OrientationFunction]] ttg_base

cdef class _VectorOrientationTrajectoryTask(MetaTask):
  cdef shared_ptr[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.VectorOrientationFunction]] ttg_base

cdef class _SplineTrajectoryTask(MetaTask):
  cdef shared_ptr[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]] ttg_base

cdef class _TransformTask(MetaTask):
  cdef shared_ptr[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]] ttg_base

cdef class BSplineTrajectoryTask(_SplineTrajectoryTask):
  cdef shared_ptr[c_mc_tasks.BSplineTrajectoryTask] impl

cdef class ExactCubicTrajectoryTask(_SplineTrajectoryTask):
  cdef shared_ptr[c_mc_tasks.ExactCubicTrajectoryTask] impl

cdef class CoMTask(_CoMTrajectoryTask):
  cdef shared_ptr[c_mc_tasks.CoMTask] impl

cdef class PositionTask(_PositionTrajectoryTask):
  cdef shared_ptr[c_mc_tasks.PositionTask] impl

cdef class PostureTask(_PostureTrajectoryTask):
  cdef shared_ptr[c_mc_tasks.PostureTask] impl

cdef class OrientationTask(_OrientationTrajectoryTask):
  cdef shared_ptr[c_mc_tasks.OrientationTask] impl

cdef class VectorOrientationTask(_VectorOrientationTrajectoryTask):
  cdef shared_ptr[c_mc_tasks.VectorOrientationTask] impl

cdef class TransformTask(_TransformTask):
  cdef shared_ptr[c_mc_tasks.TransformTask] impl

ctypedef fused AnyTTG:
  PostureTask
  CoMTask
  PositionTask
  OrientationTask
  VectorOrientationTask
  TransformTask

#Note : In recent versions of Cython, fused types can be fused and thus
# AnyTask can simply be the fusion of AnyTTG and other tasks
ctypedef fused AnyTask:
  PostureTask
  CoMTask
  PositionTask
  OrientationTask
  VectorOrientationTask
  TransformTask
  BSplineTrajectoryTask
  ExactCubicTrajectoryTask
