#
# Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport eigen.c_eigen as c_eigen
cimport sva.c_sva as c_sva
cimport rbdyn.c_rbdyn as c_rbdyn
cimport sch.c_sch as c_sch

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_solver.c_mc_solver as c_mc_solver
cimport mc_tasks.c_mc_tasks as c_mc_tasks

from libcpp.memory cimport shared_ptr
from libcpp.pair cimport pair
from libcpp cimport bool as cppbool

cdef extern from *:
  ctypedef cppbool TRUE "true"
  ctypedef cppbool FALSE "false"

cdef extern from "<mc_tasks/ImpedanceGains.h>" namespace "mc_tasks::force::details":
  cdef cppclass ImpedanceVecd[StrictlyPositive]:
    c_eigen.Vector6d vector()
    void vec(c_sva.ImpedanceVecd &)
    void vec(c_eigen.Vector6d &)
    void vec(c_eigen.Vector3d &, c_eigen.Vector3d &)
    void vec(double, double)
    c_sva.ImpedanceVecd & vec()
    void angular(c_eigen.Vector3d &)
    void angular(double)
    c_eigen.Vector3d & angular()
    void linear(c_eigen.Vector3d &)
    void linear(double)
    c_eigen.Vector3d & linear()

  ctypedef ImpedanceVecd[TRUE] ImpedanceVecdStrictlyPositive
  ctypedef ImpedanceVecd[FALSE] ImpedanceVecdPositive

cdef extern from "<mc_tasks/ImpedanceGains.h>" namespace "mc_tasks::force":
  cdef cppclass ImpedanceGains:
    ImpedanceVecdStrictlyPositive & mass()
    ImpedanceVecdPositive & damper()
    ImpedanceVecdPositive & spring()
    ImpedanceVecdPositive & wrench()

cdef extern from "<mc_tasks/ImpedanceTask.h>" namespace "mc_tasks::force":
  cdef cppclass ImpedanceTask(c_mc_tasks.TransformTask):
    ImpedanceTask(c_mc_rbdyn.Frame &, double, double)
    ImpedanceGains & gains()
    c_sva.PTransformd & targetPose()
    void targetPose(c_sva.PTransformd &)
    c_sva.MotionVecd & targetVel()
    void targetVel(c_sva.MotionVecd &)
    c_sva.MotionVecd & targetAccel()
    void targetAccel(c_sva.MotionVecd &)
    c_sva.ForceVecd targetWrench()
    void targetWrenchW(c_sva.ForceVecd &)
    void targetWrench(c_sva.ForceVecd &)
    double cutoffPeriod()
    void cutoffPeriod(double)

cdef extern from "<mc_tasks/ComplianceTask.h>" namespace "mc_tasks::force":
  pair[double, double] defaultFGain
  pair[double, double] defaultTGain

  cdef cppclass ComplianceTask(c_mc_tasks.MetaTask):
    ComplianceTask(c_mc_rbdyn.Frame &, c_eigen.Vector6d &,
                   double stiffness, double weight,
                   double forceThresh, double torqueThresh,
                   pair[double, double] forceGain, pair[double, double] torqueGain)
    void setTargetWrench(const c_sva.ForceVecd&)
    c_sva.ForceVecd getTargetWrench()
    void stiffness(double)
    double stiffness()
    void weight(double)
    double weight()
    void forceThreshold(double)
    double forceThreshold()
    void torqueThreshold(double)
    double torqueThreshold()
    void forceGain(pair[double, double])
    pair[double, double] forceGain()
    void torqueGain(pair[double, double])
    pair[double, double] torqueGain()
    void dof(c_eigen.Vector6d &)
    c_eigen.Vector6d & dof()

cdef extern from "<mc_tasks/AdmittanceTask.h>" namespace "mc_tasks::force":
  cdef cppclass AdmittanceTask(c_mc_tasks.TransformTask):
    AdmittanceTask(c_mc_rbdyn.Frame &, double, double)
    void admittance(c_sva.ForceVecd &)
    c_sva.ForceVecd & admittance()
    void targetPose(const c_sva.PTransformd &)
    c_sva.PTransformd & targetPose()
    void targetWrench(const c_sva.ForceVecd &)
    c_sva.ForceVecd & targetWrench()

cdef extern from "<mc_tasks/DampingTask.h>" namespace "mc_tasks::force":
  cdef cppclass DampingTask(AdmittanceTask):
    DampingTask(c_mc_rbdyn.Frame &, double, double)

cdef extern from "<mc_tasks/CoPTask.h>" namespace "mc_tasks::force":
  cdef cppclass CoPTask(DampingTask):
    CoPTask(c_mc_rbdyn.Frame &, double, double)
    c_eigen.Vector2d measuredCoP()
    c_eigen.Vector3d measuredCoPW()
    void setZeroTargetWrench()
    c_eigen.Vector2d targetCoP()
    c_eigen.Vector3d targetCoPW()
    void targetCoP(const c_eigen.Vector2d &)
    c_eigen.Vector3d targetForce()
    void targetForce(const c_eigen.Vector3d &)
    void targetForceW(const c_eigen.Vector3d &)

cdef extern from "mc_tasks_wrapper.hpp" namespace "mc_tasks":
  shared_ptr[T] make_shared_aligned[T](...) except +
  # Shortcut for static_pointer_cast where Cython can infer types by itself and generate correct code
  shared_ptr[T] cast[T](...)
