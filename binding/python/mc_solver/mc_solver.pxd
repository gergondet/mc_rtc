#
# Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_mc_solver

from libcpp.memory cimport shared_ptr

cdef class Constraint(object):
  cdef shared_ptr[c_mc_solver.Constraint] base

cdef class KinematicsConstraint(Constraint):
  cdef shared_ptr[c_mc_solver.KinematicsConstraint] impl

cdef class DynamicsConstraint(KinematicsConstraint):
  cdef shared_ptr[c_mc_solver.DynamicsConstraint] d_impl

cdef class CollisionsConstraint(Constraint):
  cdef shared_ptr[c_mc_solver.CollisionsConstraint] impl

cdef class QPSolver(object):
  cdef shared_ptr[c_mc_solver.QPSolver] impl
