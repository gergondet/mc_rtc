#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_tasks.force.c_force as c_force

from mc_tasks.mc_tasks cimport MetaTask, TransformTask

from libcpp cimport bool as cppbool
from libcpp.memory cimport shared_ptr

cdef class ImpedanceVecdStrictlyPositive(object):
  cdef c_force.ImpedanceVecdStrictlyPositive * impl

cdef ImpedanceVecdStrictlyPositive ImpedanceVecdStrictlyPositiveFromRef(c_force.ImpedanceVecdStrictlyPositive &)

cdef class ImpedanceVecdPositive(object):
  cdef c_force.ImpedanceVecdPositive * impl

cdef ImpedanceVecdPositive ImpedanceVecdPositiveFromRef(c_force.ImpedanceVecdPositive &)

cdef class ImpedanceGains(object):
  cdef c_force.ImpedanceGains * impl

cdef ImpedanceGains ImpedanceGainsFromRef(c_force.ImpedanceGains &)

cdef class ImpedanceTask(TransformTask):
  cdef shared_ptr[c_force.ImpedanceTask] impedance_impl

cdef class ComplianceTask(MetaTask):
  cdef shared_ptr[c_force.ComplianceTask] impl

cdef class AdmittanceTask(TransformTask):
  cdef shared_ptr[c_force.AdmittanceTask] adm_impl

cdef class DampingTask(AdmittanceTask):
  cdef shared_ptr[c_force.DampingTask] damping_impl

cdef class CoPTask(DampingTask):
  cdef shared_ptr[c_force.CoPTask] cop_impl
