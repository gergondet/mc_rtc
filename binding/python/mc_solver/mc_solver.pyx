# distutils: language = c++

#
# Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_solver.c_mc_solver as c_mc_solver

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

cimport mc_tasks.mc_tasks as mc_tasks

cimport eigen.eigen as eigen

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.memory cimport make_shared, static_pointer_cast
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool
from libcpp cimport nullptr

cdef class Constraint(object):
  def __cinit__(self):
    pass
  def name(self):
    assert(self.base.get())
    return deref(self.base).name()

cdef class KinematicsConstraint(Constraint):
  def __cinit__(self, mc_rbdyn.Robot robot, damper, velocityPercent = 0.5):
    cdef c_mc_solver.array3d damp = c_mc_solver.array3d()
    if type(self) is KinematicsConstraint:
      assert(len(damper) == 3)
      for i in xrange(3):
        damp[i] = damper[i]
      self.impl = make_shared[c_mc_solver.KinematicsConstraint](deref(robot.impl), damp, <double>(velocityPercent))
      self.base = static_pointer_cast[c_mc_solver.Constraint, c_mc_solver.KinematicsConstraint](self.impl)

cdef class DynamicsConstraint(KinematicsConstraint):
  def __cinit__(self, mc_rbdyn.Robot robot, damper, velocityPercent = 0.5):
    cdef c_mc_solver.array3d damp = c_mc_solver.array3d()
    assert(len(damper) == 3)
    for i in xrange(3):
      damp[i] = damper[i]
    self.d_impl = make_shared[c_mc_solver.DynamicsConstraint](deref(robot.impl), damp, <double>(velocityPercent))
    self.impl = static_pointer_cast[c_mc_solver.KinematicsConstraint, c_mc_solver.DynamicsConstraint](self.d_impl)
    self.base = static_pointer_cast[c_mc_solver.Constraint, c_mc_solver.KinematicsConstraint](self.impl)

cdef class CollisionsConstraint(Constraint):
  defaultDampingOffset = c_mc_solver.CollisionsConstraintDefaultDampingOffset
  def __cinit__(self):
    self.impl = make_shared[c_mc_solver.CollisionsConstraint]()
    self.base = static_pointer_cast[c_mc_solver.Constraint, c_mc_solver.CollisionsConstraint](self.impl)
  def removeCollisions(self, QPSolver solver, r1, r2 = None):
    if r2 is None:
      if isinstance(r1, mc_rbdyn.CollisionVector):
        deref(self.impl).removeCollisions(deref(solver.impl), (<mc_rbdyn.CollisionVector>r1).impl)
        return
      else:
        return self.removeCollisions(solver, mc_rbdyn.CollisionVector(r1))
    if isinstance(r1, unicode):
        r1 = r1.encode(u'ascii')
    if isinstance(r2, unicode):
        r2 = r2.encode(u'ascii')
    deref(self.impl).removeCollisions(deref(solver.impl), r1, r2)
  def addCollision(self, QPSolver solver, mc_rbdyn.Collision col):
    deref(self.impl).addCollision(deref(solver.impl), col.impl)
  def addCollisions(self, QPSolver solver, cols):
    for c in cols:
      self.addCollision(solver, c)
  def reset(self, QPSolver solver):
    deref(self.impl).reset(deref(solver.impl))

cdef class QPSolver(object):
  def __cinit__(self):
    pass
  def addConstraint(self, Constraint cs):
    deref(self.impl).addConstraint(cs.base)
  def removeConstraint(self, Constraint c):
    deref(self.impl).removeConstraint(c.base)
  def addContact(self, mc_rbdyn.Contact c):
    deref(self.impl).addContact(c.impl)
  def removeContact(self, mc_rbdyn.Contact c):
    deref(self.impl).removeContact(c.impl)
  def contacts(self):
    end = deref(self.impl).contacts().const_end()
    it = deref(self.impl).contacts().const_begin()
    ret = []
    while it != end:
        ret.append(mc_rbdyn.ContactFromC(<c_mc_rbdyn.Contact &>deref(it)))
        preinc(it)
    return ret
  def addTask(self, mc_tasks.MetaTask task):
    deref(self.impl).addTask(task.mt_base)
  def removeTask(self, mc_tasks.MetaTask task):
    deref(self.impl).removeTask(task.mt_base)
  def run(self):
    return deref(self.impl).run()
  def robots(self):
    return mc_rbdyn.RobotsFromRef(deref(self.impl).robots())
  def realRobots(self):
    return mc_rbdyn.RobotsFromRef(deref(self.impl).realRobots())
  def dt(self):
    return deref(self.impl).dt()
