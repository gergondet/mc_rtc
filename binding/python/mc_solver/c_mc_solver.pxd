#
# Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen import *
from sva.c_sva cimport *
from rbdyn.c_rbdyn cimport *
cimport sch.c_sch as sch

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_tasks.c_mc_tasks as c_mc_tasks

from libcpp.map cimport map as cppmap
from libcpp.memory cimport shared_ptr
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<array>" namespace "std" nogil:
  cdef cppclass array[T, N]:
    array() except +
    T& operator[](size_t)

cdef extern from *:
  ctypedef int three "3"
  ctypedef array[double, three] array3d

cdef extern from "<mc_solver/Constraint.h>" namespace "mc_solver":
  cdef cppclass Constraint:
    Constraint()
    string name()

  ctypedef shared_ptr[Constraint] ConstraintPtr

cdef extern from "<mc_solver/KinematicsConstraint.h>" namespace "mc_solver":
  cdef cppclass KinematicsConstraint(Constraint):
    KinematicsConstraint(c_mc_rbdyn.Robot&, const array3d&, double)

cdef extern from "<mc_solver/DynamicsConstraint.h>" namespace "mc_solver":
  cdef cppclass DynamicsConstraint(KinematicsConstraint):
    DynamicsConstraint(c_mc_rbdyn.Robot&, const array3d&, double)

cdef extern from "<mc_solver/CollisionsConstraint.h>" namespace "mc_solver":
  cdef double CollisionsConstraintDefaultDampingOffset "mc_solver::CollisionsConstraint::defaultDampingOffset"

  cdef cppclass CollisionsConstraint(Constraint):
    CollisionsConstraint()

    void addCollision(QPSolver &, c_mc_rbdyn.Collision &)
    void addCollisions(QPSolver &, c_mc_rbdyn.CollisionVector &)
    void removeCollisions(QPSolver &, string, string)
    void removeCollisions(QPSolver &, c_mc_rbdyn.CollisionVector &)

    void reset(QPSolver &)

    vector[c_mc_rbdyn.Collision] collisions()

cdef extern from "<mc_solver/QPSolver.h>" namespace "mc_solver":
  cdef cppclass QPSolver:

    void addConstraint(ConstraintPtr)
    void removeConstraint(ConstraintPtr)

    void addContact(c_mc_rbdyn.Contact &)
    void removeContact(c_mc_rbdyn.Contact &)
    const vector[c_mc_rbdyn.Contact] & contacts()

    void addTask(c_mc_tasks.MetaTaskPtr)
    void removeTask(c_mc_tasks.MetaTaskPtr)

    cppbool run()

    c_mc_rbdyn.Robots & robots()

    c_mc_rbdyn.Robots & realRobots()

    double dt()

cdef extern from "mc_solver_wrapper.hpp" namespace "mc_solver":
    #FIXME Provide a way to create a QPSolver instance from Python
    pass
