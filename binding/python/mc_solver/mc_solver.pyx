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
    def __ctor__(self, mc_rbdyn.Robot robot, damper, velocityPercent = 0.5):
        cdef c_mc_solver.array3d damp = c_mc_solver.array3d()
        if type(self) is KinematicsConstraint:
            assert(len(damper) == 3)
            for i in xrange(3):
                damp[i] = damper[i]
            self.__ptrinit__(make_shared[c_mc_solver.KinematicsConstraint](deref(robot.impl), damp, <double>(velocityPercent)))
    cdef __ptrinit__(self, shared_ptr[c_mc_solver.KinematicsConstraint] ptr):
            self.impl = ptr
            self.base = static_pointer_cast[c_mc_solver.Constraint, c_mc_solver.KinematicsConstraint](self.impl)
    def __cinit__(self, *args, skip_alloc = False, **kwargs):
            if skip_alloc:
                assert(len(args) + len(kwargs) == 0)
                return
            else:
                self.__ctor__(*args, **kwargs)

cdef KinematicsConstraint KinematicsConstraintFromPtr(shared_ptr[c_mc_solver.KinematicsConstraint] ptr):
    cdef KinematicsConstraint ret = KinematicsConstraint(skip_alloc = True)
    ret.__ptrinit__(ptr)
    return ret

cdef class DynamicsConstraint(KinematicsConstraint):
    def __ctor__(self, mc_rbdyn.Robot robot, damper, velocityPercent = 0.5):
        cdef c_mc_solver.array3d damp = c_mc_solver.array3d()
        assert(len(damper) == 3)
        for i in xrange(3):
            damp[i] = damper[i]
        self.__dptrinit__(make_shared[c_mc_solver.DynamicsConstraint](deref(robot.impl), damp, <double>(velocityPercent)))
    cdef __dptrinit__(self, shared_ptr[c_mc_solver.DynamicsConstraint] ptr):
        self.d_impl = ptr
        self.impl = static_pointer_cast[c_mc_solver.KinematicsConstraint, c_mc_solver.DynamicsConstraint](self.d_impl)
        self.base = static_pointer_cast[c_mc_solver.Constraint, c_mc_solver.KinematicsConstraint](self.impl)
    def __cinit__(self, *args, skip_alloc = False, **kwargs):
            if skip_alloc:
                assert(len(args) + len(kwargs) == 0)
                return
            else:
                self.__ctor__(*args, **kwargs)

cdef DynamicsConstraint DynamicsConstraintFromPtr(shared_ptr[c_mc_solver.DynamicsConstraint] ptr):
    cdef DynamicsConstraint ret = DynamicsConstraint(skip_alloc = True)
    ret.__dptrinit__(ptr)
    return ret

cdef class CollisionsConstraint(Constraint):
    defaultDampingOffset = c_mc_solver.CollisionsConstraintDefaultDampingOffset
    cdef __ptrinit__(self, shared_ptr[c_mc_solver.CollisionsConstraint] ptr):
        self.impl = ptr
        self.base = static_pointer_cast[c_mc_solver.Constraint, c_mc_solver.CollisionsConstraint](self.impl)
    def __cinit__(self, skip_alloc = False):
        if skip_alloc:
            return
        self.__ptrinit__(make_shared[c_mc_solver.CollisionsConstraint]())
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

cdef CollisionsConstraint CollisionsConstraintFromPtr(shared_ptr[c_mc_solver.CollisionsConstraint] ptr):
    cdef CollisionsConstraint ret = CollisionsConstraint(skip_alloc = True)
    ret.__ptrinit__(ptr)
    return ret

cdef class QPSolver(object):
    #FIXME Need a full constructor
    def __cinit__(self):
        pass
    def addConstraint(self, Constraint cs):
        assert(self.impl)
        deref(self.impl).addConstraint(cs.base)
    def removeConstraint(self, Constraint c):
        assert(self.impl)
        deref(self.impl).removeConstraint(c.base)
    def addContact(self, mc_rbdyn.Contact c):
        assert(self.impl)
        deref(self.impl).addContact(c.impl)
    def removeContact(self, mc_rbdyn.Contact c):
        assert(self.impl)
        deref(self.impl).removeContact(c.impl)
    def contacts(self):
        assert(self.impl)
        end = deref(self.impl).contacts().const_end()
        it = deref(self.impl).contacts().const_begin()
        ret = []
        while it != end:
                ret.append(mc_rbdyn.ContactFromC(<c_mc_rbdyn.Contact &>deref(it)))
                preinc(it)
        return ret
    def addTask(self, mc_tasks.MetaTask task):
        assert(self.impl)
        deref(self.impl).addTask(task.mt_base)
    def removeTask(self, mc_tasks.MetaTask task):
        assert(self.impl)
        deref(self.impl).removeTask(task.mt_base)
    def run(self):
        assert(self.impl)
        return deref(self.impl).run()
    def robots(self):
        assert(self.impl)
        return mc_rbdyn.RobotsFromRef(deref(self.impl).robots())
    def realRobots(self):
        assert(self.impl)
        return mc_rbdyn.RobotsFromRef(deref(self.impl).realRobots())
    def dt(self):
        assert(self.impl)
        return deref(self.impl).dt()

cdef QPSolver QPSolverFromRef(c_mc_solver.QPSolver & s):
    cdef QPSolver ret = QPSolver()
    ret.impl = &s
    return ret
