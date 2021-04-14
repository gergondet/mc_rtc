#
# Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_mc_solver

from libcpp.memory cimport shared_ptr

cdef class Constraint(object):
  cdef shared_ptr[c_mc_solver.Constraint] base

cdef class KinematicsConstraint(Constraint):
  cdef shared_ptr[c_mc_solver.KinematicsConstraint] impl
  cdef __ptrinit__(self, shared_ptr[c_mc_solver.KinematicsConstraint])

cdef KinematicsConstraint KinematicsConstraintFromPtr(shared_ptr[c_mc_solver.KinematicsConstraint])

cdef class DynamicsConstraint(KinematicsConstraint):
  cdef shared_ptr[c_mc_solver.DynamicsConstraint] d_impl
  cdef __dptrinit__(self, shared_ptr[c_mc_solver.DynamicsConstraint])

cdef DynamicsConstraint DynamicsConstraintFromPtr(shared_ptr[c_mc_solver.DynamicsConstraint])

cdef class CollisionsConstraint(Constraint):
  cdef shared_ptr[c_mc_solver.CollisionsConstraint] impl
  cdef __ptrinit__(self, shared_ptr[c_mc_solver.CollisionsConstraint])

cdef CollisionsConstraint CollisionsConstraintFromPtr(shared_ptr[c_mc_solver.CollisionsConstraint])

cdef class QPSolver(object):
  cdef shared_ptr[c_mc_solver.QPSolver] ptr
  cdef c_mc_solver.QPSolver * impl

cdef QPSolver QPSolverFromRef(c_mc_solver.QPSolver &)
