# distutils: language = c++

#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn

cimport eigen.c_eigen as c_eigen
cimport eigen.eigen as eigen

cimport sva.c_sva as c_sva
cimport sva.sva as sva

cimport rbdyn.c_rbdyn as c_rbdyn
cimport rbdyn.rbdyn as rbdyn

cimport sch.sch as sch

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

import json
import warnings

def deprecated():
    warnings.simplefilter('always', category=DeprecationWarning)
    warnings.warn("This call is deprecated", DeprecationWarning)
    warnings.simplefilter('ignore', category=DeprecationWarning)

# Hold python object that have to be kept alive
global __MODULE_OBJECTS__
__MODULE_OBJECTS__ = []

cdef class CollisionDescription(object):
    def __copyctor__(self, CollisionDescription other):
        self.impl = other.impl
    def __cinit__(self, *args):
        if len(args) == 1 and isinstance(args[0], CollisionDescription):
            self.__copyctor__(args[0])
        elif len(args) == 0:
            self.impl = c_mc_rbdyn.CollisionDescription()
        elif len(args) == 5:
            object1 = args[0]
            if isinstance(object1, unicode):
                object1 = object1.encode(u'ascii')
            object2 = args[1]
            if isinstance(object2, unicode):
                object2 = object2.encode(u'ascii')
            self.impl = c_mc_rbdyn.makeCollisionDescription(object1, object2, args[2], args[3], args[4])
        else:
            raise TypeError("Invalid arguments passed to Collision ctor")

    property object1:
        def __get__(self):
            return self.impl.object1
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.object1 = value
    property object2:
        def __get__(self):
            return self.impl.object2
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.object2 = value
    property iDist:
        def __get__(self):
            return self.impl.iDist
        def __set__(self, value):
            self.impl.iDist = value
    property sDist:
        def __get__(self):
            return self.impl.sDist
        def __set__(self, value):
            self.impl.sDist = value
    property damping:
        def __get__(self):
            return self.impl.damping
        def __set__(self, value):
            self.impl.damping = value

cdef CollisionDescription CollisionDescriptionFromC(c_mc_rbdyn.CollisionDescription & c):
    cdef CollisionDescription ret = CollisionDescription()
    ret.impl = c
    return ret

cdef class Collision(object):
    def __copyctor__(self, Collision other):
        self.impl = other.impl
    def __cinit__(self, *args):
        if len(args) == 1 and isinstance(args[0], Collision):
            self.__copyctor__(args[0])
        elif len(args) == 0:
            self.impl = c_mc_rbdyn.Collision()
        elif len(args) == 7:
            robot1 = args[0]
            if isinstance(robot1, unicode):
                robot1 = robot1.encode(u'ascii')
            robot2 = args[1]
            if isinstance(robot2, unicode):
                robot2 = robot2.encode(u'ascii')
            object1 = args[2]
            if isinstance(object1, unicode):
                object1 = object1.encode(u'ascii')
            object2 = args[3]
            if isinstance(object2, unicode):
                object2 = object2.encode(u'ascii')
            self.impl = c_mc_rbdyn.Collision(robot1, robot2, object1, object2, args[4], args[5], args[6])
        else:
            raise TypeError("Invalid arguments passed to Collision ctor")

    property robot1:
        def __get__(self):
            return self.impl.robot1
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.robot1 = value
    property robot2:
        def __get__(self):
            return self.impl.robot2
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.robot2 = value
    property object1:
        def __get__(self):
            return self.impl.object1
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.object1 = value
    property object2:
        def __get__(self):
            return self.impl.object2
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.object2 = value
    property iDist:
        def __get__(self):
            return self.impl.iDist
        def __set__(self, value):
            self.impl.iDist = value
    property sDist:
        def __get__(self):
            return self.impl.sDist
        def __set__(self, value):
            self.impl.sDist = value
    property damping:
        def __get__(self):
            return self.impl.damping
        def __set__(self, value):
            self.impl.damping = value

    def __richcmp__(Collision self, Collision other, int op):
        if op == 2:
            return self.impl == other.impl
        elif op == 3:
            return self.impl != other.impl
        else:
            raise NotImplementedError("This comparison is not supported")

    def __str__(self):
        return "{}::{}/{}::{} (iDist: {}, sDist: {})".format(self.robot1, self.body1, self.robot2, self.body2, self.iDist, self.sDist)
    def __repr__(self):
        return self.__str__()

cdef Collision CollisionFromC(const c_mc_rbdyn.Collision & col):
    cdef Collision ret = Collision()
    ret.impl = c_mc_rbdyn.Collision(col)
    return ret

cdef class CollisionVector(object):
    def __cinit__(self, *args):
        cdef vector[c_mc_rbdyn.CollisionDescription] descArg
        cdef vector[c_mc_rbdyn.Collision] colsArg
        if len(args) == 0:
            return
        if len(args) == 2 or len(args) == 3:
            r1 = args[0]
            if isinstance(r1, unicode):
                r1 = r1.encode(u'ascii')
            if len(args) == 3:
                r2 = args[1]
                cols = args[2]
                if isinstance(r2, unicode):
                    r2 = r2.encode(u'ascii')
            else:
                r2 = r1
                cols = args[1]
            for col in cols:
                descArg.push_back((<CollisionDescription>col).impl)
            self.impl = c_mc_rbdyn.CollisionVector(r1, r2, descArg)
            return
        if len(args) == 1:
            if isinstance(args[0], CollisionVector):
                self.impl = (<CollisionVector>(args[0])).impl
            else:
                for col in args[0]:
                    colsArg.push_back((<Collision>col).impl)
                self.impl = c_mc_rbdyn.CollisionVector(colsArg)
            return
        raise TypeError("Invalid arguments passed to CollisionVector ctor")

cdef class Flexibility(object):
    def __cinit__(self, *args):
        if len(args) == 4:
            self.jointName = args[0]
            self.K = args[1]
            self.C = args[2]
            self.O = args[3]
        elif len(args) != 0:
            raise TypeError("Invalid arguments passed to Flexibility ctor")
    property jointName:
        def __get__(self):
            return self.impl.jointName
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.jointName = value
    property K:
        def __get__(self):
            return self.impl.K
        def __set__(self, value):
            self.impl.K = value
    property C:
        def __get__(self):
            return self.impl.C
        def __set__(self, value):
            self.impl.C = value
    property O:
        def __get__(self):
            return self.impl.O
        def __set__(self, value):
            self.impl.O = value

cdef Flexibility FlexibilityFromC(const c_mc_rbdyn.Flexibility & flex):
    cdef Flexibility ret = Flexibility()
    ret.impl = c_mc_rbdyn.Flexibility(flex)
    return ret

cdef class BodySensor(object):
    def __dealloc__(self):
        if self.__own_impl:
            del self.impl
    def __cinit__(self, skip_alloc = False):
        self.__own_impl = not skip_alloc
        if skip_alloc:
            self.impl = NULL
            return
        self.impl = new c_mc_rbdyn.BodySensor()
    def name(self):
        return self.impl.name()
    def parentBody(self):
        return self.impl.parentBody()
    def X_b_s(self):
        return sva.PTransformdFromC(self.impl.X_b_s())
    def position(self):
        return eigen.Vector3dFromC(self.impl.position())
    def orientation(self):
        return eigen.QuaterniondFromC(self.impl.orientation())
    def linearVelocity(self):
        return eigen.Vector3dFromC(self.impl.linearVelocity())
    def angularVelocity(self):
        return eigen.Vector3dFromC(self.impl.angularVelocity())
    def acceleration(self):
        deprecated()
        return eigen.Vector3dFromC(self.impl.linearAcceleration())
    def linearAcceleration(self):
        return eigen.Vector3dFromC(self.impl.linearAcceleration())

cdef BodySensor BodySensorFromRef(c_mc_rbdyn.BodySensor & bs):
        cdef BodySensor ret = BodySensor(skip_alloc = True)
        ret.impl = &(bs)
        return ret

cdef BodySensor BodySensorFromCRef(const c_mc_rbdyn.BodySensor & bs):
        cdef BodySensor ret = BodySensor(skip_alloc = True)
        ret.impl = &(c_mc_rbdyn.const_cast_body_sensor(bs))
        return ret

cdef class ForceSensor(object):
    def __dealloc__(self):
        if self.__own_impl:
            del self.impl
    def __ctor__(self, sn, bn, sva.PTransformd X_p_f):
        if isinstance(sn, unicode):
            sn = sn.encode(u'ascii')
        if isinstance(bn, unicode):
            bn = bn.encode(u'ascii')
        self.impl = new c_mc_rbdyn.ForceSensor(sn, bn, deref(X_p_f.impl))
    def __cinit__(self, *args, skip_alloc = False):
        self.__own_impl = not skip_alloc
        if skip_alloc:
            assert(len(args) ==0)
            self.impl = NULL
            return
        if len(args) == 0:
            self.impl = new c_mc_rbdyn.ForceSensor()
        elif len(args) == 3:
            self.__ctor__(*args)
        else:
            raise TypeError("Invalid arguments passed to ForceSensor ctor")
    def name(self):
        return self.impl.name()
    def parentBody(self):
        return self.impl.parentBody()
    def X_p_f(self):
        return sva.PTransformdFromC(self.impl.X_p_f(), copy = False)
    def wrench(self):
        return sva.ForceVecdFromC(self.impl.wrench(), copy = False)
    def mass(self):
        return self.impl.mass()
    def wrenchWithoutGravity(self, Robot robot):
        return sva.ForceVecdFromC(self.impl.wrenchWithoutGravity(deref(robot.impl)))
    def worldWrench(self, Robot robot):
        return sva.ForceVecdFromC(self.impl.worldWrench(deref(robot.impl)))
    def worldWrenchWithoutGravity(self, Robot robot):
        return sva.ForceVecdFromC(self.impl.worldWrenchWithoutGravity(deref(robot.impl)))

cdef ForceSensor ForceSensorFromRef(c_mc_rbdyn.ForceSensor & fs):
        cdef ForceSensor ret = ForceSensor(skip_alloc = True)
        ret.impl = &(fs)
        return ret

cdef ForceSensor ForceSensorFromCRef(const c_mc_rbdyn.ForceSensor & fs):
        cdef ForceSensor ret = ForceSensor(skip_alloc = True)
        ret.impl = &(c_mc_rbdyn.const_cast_force_sensor(fs))
        return ret

cdef class Springs(object):
    def __cinit__(self, *args):
        if len(args) != 0:
            raise TypeError("Invalid arguments passed to Springs ctor")

    property springsBodies:
        def __get__(self):
            return self.impl.springsBodies
        def __set__(self, value):
            self.impl.springsBodies = value
    property afterSpringsBodies:
        def __get__(self):
            return self.impl.afterSpringsBodies
        def __set__(self, value):
            self.impl.afterSpringsBodies = value
    property springsJoints:
        def __get__(self):
            return self.impl.springsJoints
        def __set__(self, value):
            self.impl.springsJoints = value

cdef Springs SpringsFromC(const c_mc_rbdyn.Springs & sp):
    cdef Springs ret = Springs()
    ret.impl = c_mc_rbdyn.Springs(sp)
    return ret

cdef class Base(object):
    def __copyctor__(self, Base other):
        self.impl = other.impl
    def __cinit__(self, *args):
        if len(args) == 0:
            pass
        elif len(args) == 1:
            self.__copyctor__(args[0])
        elif len(args) == 4:
            self.baseName = args[0]
            self.X_0_s = args[1]
            self.X_b0_s = args[2]
            self.baseType = args[3]
        else:
            raise TypeError("Cannot build base from this arguments")
    property baseName:
        def __get__(self):
            return self.impl.baseName
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.baseName = value
    property X_0_s:
        def __get__(self):
            return sva.PTransformdFromC(self.impl.X_0_s, copy = False)
        def __set__(self, sva.PTransformd value):
            self.impl.X_0_s = deref(value.impl)
    property X_b0_s:
        def __get__(self):
            return sva.PTransformdFromC(self.impl.X_b0_s, copy = False)
        def __set__(self, sva.PTransformd value):
            self.impl.X_b0_s = deref(value.impl)
    property baseType:
        def __get__(self):
            return self.impl.baseType
        def __set__(self, value):
            self.impl.baseType = value

cdef class RobotModule(object):
    def __cinit__(self, *args):
        if len(args) != 0:
            raise TypeError("Wrong argument passed to RobotModule ctor")

    def bounds(self):
        assert(self.impl.get())
        ret = []
        bounds_end = deref(self.impl).bounds().const_end()
        bounds_it = deref(self.impl).bounds().const_begin()
        while bounds_it != bounds_end:
            end = deref(bounds_it).const_end()
            it = deref(bounds_it).const_begin()
            out = {}
            while it != end:
                out[deref(it).first] = deref(it).second
                preinc(it)
            ret.append(out)
            preinc(bounds_it)
        return ret
    def stance(self):
        assert(self.impl.get())
        end = deref(self.impl).stance().const_end()
        it = deref(self.impl).stance().const_begin()
        out = {}
        while it != end:
            out[deref(it).first] = deref(it).second
            preinc(it)
        return out
    def convexHull(self):
        assert(self.impl.get())
        end = deref(self.impl).convexHull().const_end()
        it = deref(self.impl).convexHull().const_begin()
        out = {}
        while it != end:
            out[deref(it).first] = deref(it).second
            preinc(it)
        return out
    def collisionTransforms(self):
        assert(self.impl.get())
        end = deref(self.impl)._collisionTransforms.const_end()
        it = deref(self.impl)._collisionTransforms.const_begin()
        ret = {}
        while it != end:
            ret[deref(it).first] = sva.PTransformdFromC(deref(it).second)
            preinc(it)
        return ret
    def flexibility(self):
        assert(self.impl.get())
        end = deref(self.impl)._flexibility.end()
        it = deref(self.impl)._flexibility.begin()
        ret = []
        while it != end:
            ret.append(FlexibilityFromC(deref(it)))
            preinc(it)
        return ret
    def forceSensors(self):
        assert(self.impl.get())
        end = deref(self.impl)._forceSensors.end()
        it = deref(self.impl)._forceSensors.begin()
        ret = []
        while it != end:
            ret.append(ForceSensorFromCRef(deref(it)))
            preinc(it)
        return ret
    def bodySensors(self):
        assert(self.impl.get())
        size = c_mc_rbdyn.getBodySensorsSize(deref(self.impl))
        ret = []
        for i in range(size):
                ret.append(BodySensorFromCRef(c_mc_rbdyn.getBodySensor(deref(self.impl), i)))
        return ret
    def springs(self):
        assert(self.impl.get())
        return SpringsFromC(deref(self.impl).springs())
    def minimalSelfCollisions(self):
        assert(self.impl.get())
        return [CollisionDescriptionFromC(c) for c in deref(self.impl)._minimalSelfCollisions]
    def commonSelfCollisions(self):
        assert(self.impl.get())
        return [CollisionDescriptionFromC(c) for c in deref(self.impl)._commonSelfCollisions]
    def ref_joint_order(self):
        assert(self.impl.get())
        cdef vector[string] joints = deref(self.impl).ref_joint_order()
        return [ j.decode('ascii') for j in joints ]
    def default_attitude(self):
        assert(self.impl.get())
        return c_mc_rbdyn.robotModuleDefaultAttitude(self.impl)
    property path:
        def __get__(self):
            assert(self.impl.get())
            return deref(self.impl).path
    property name:
        def __get__(self):
            assert(self.impl.get())
            return deref(self.impl).name
    property urdf_path:
        def __get__(self):
            assert(self.impl.get())
            return deref(self.impl).urdf_path
    property rsdf_dir:
        def __get__(self):
            assert(self.impl.get())
            return deref(self.impl).rsdf_dir
    @property
    def calib_dir(self):
        assert(self.impl.get())
        return deref(self.impl).calib_dir
    property mb:
        def __get__(self):
            assert(self.impl.get())
            return rbdyn.MultiBodyFromC(deref(self.impl).mb, copy = False)
        def __set__(self, rbdyn.MultiBody mb):
            assert(self.impl.get())
            deref(self.impl).mb = deref(mb.impl)
    property mbc:
        def __get__(self):
            assert(self.impl.get())
            return rbdyn.MultiBodyConfigFromC(deref(self.impl).mbc, copy = False)
        def __set__(self, rbdyn.MultiBodyConfig mbc):
            assert(self.impl.get())
            deref(self.impl).mbc = deref(mbc.impl)
    property mbg:
        def __get__(self):
            assert(self.impl.get())
            return rbdyn.MultiBodyGraphFromC(deref(self.impl).mbg, copy = False)

cdef RobotModule RobotModuleFromC(const c_mc_rbdyn.RobotModulePtr v):
    cdef RobotModule ret = RobotModule()
    ret.impl = v
    return ret

cdef RobotModule RobotModuleFromCRef(const c_mc_rbdyn.RobotModule & rm):
    cdef RobotModule ret = RobotModule()
    ret.impl = c_mc_rbdyn.copyRobotModule(rm)
    return ret

cdef class RobotModuleVector(object):
    def __addRM(self, RobotModule rm):
        self.v.push_back(rm.impl)
    def __cinit__(self, *args):
        if len(args) == 1:
            if isinstance(args[0], RobotModule):
                self.__addRM(args[0])
            elif isinstance(args[0], list):
                for rm in args[0]:
                    self.__addRM(rm)
        else:
            for rm in args:
                self.__addRM(rm)

class RobotLoader(object):
    @staticmethod
    def get_robot_module(name, *args):
        cdef shared_ptr[c_mc_rbdyn.RobotModule] rm
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        if len(args) == 0:
            rm = c_mc_rbdyn.get_robot_module(name)
        elif len(args) == 1:
            arg0 = args[0]
            if isinstance(arg0, unicode):
                arg0 = arg0.encode(u'ascii')
            rm = c_mc_rbdyn.get_robot_module(name, arg0)
        elif len(args) == 2:
            arg0 = args[0]
            if isinstance(arg0, unicode):
                arg0 = arg0.encode(u'ascii')
            arg1 = args[1]
            if isinstance(arg1, unicode):
                arg1 = arg1.encode(u'ascii')
            rm = c_mc_rbdyn.get_robot_module(name, arg0, arg1)
        else:
            raise TypeError("Wrong arguments passed to get_robot_module")
        return RobotModuleFromC(rm)
    @staticmethod
    def available_robots():
        cdef vector[string] bots
        bots = c_mc_rbdyn.available_robots()
        return [ b.decode('ascii') for b in bots ]
    @staticmethod
    def clear():
        c_mc_rbdyn.clear_robot_module_path()
    @staticmethod
    def update_robot_module_path(paths):
        paths = [ p.encode(u'ascii') for p in paths ]
        c_mc_rbdyn.update_robot_module_path(paths)

def get_robot_module(name, *args):
    if isinstance(name, unicode):
        name = name.encode(u'ascii')
    return RobotLoader.get_robot_module(name, *args)

cdef class Robots(object):
    def __copyctor__(self, Robots other):
        self.impl = shared_ptr[c_mc_rbdyn.Robots](new c_mc_rbdyn.Robots(deref(other.impl.get())))
    def __cinit__(self, *args, skip_alloc = False):
        if len(args) == 0:
            if not skip_alloc:
                self.impl = shared_ptr[c_mc_rbdyn.Robots](new c_mc_rbdyn.Robots())
        elif len(args) == 1 and isinstance(args[0], Robots):
            self.__copyctor__(args[0])
        else:
            raise TypeError("Wrong arguments passed to Robots ctor")

    def robots(self):
        end = deref(self.impl).robots().end()
        it = deref(self.impl).robots().begin()
        ret = []
        while it != end:
            ret.append(RobotFromC(deref(deref(it).get())))
            preinc(it)
        return ret

    def load(self, RobotModule module, name):
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return RobotFromC(deref(self.impl).load(deref(module.impl), name))

    def robot(self, idx = None):
        if idx is None:
            return RobotFromC(deref(self.impl).robot())
        else:
            if isinstance(idx, unicode):
                idx = idx.encode(u'ascii')
            return RobotFromC(deref(self.impl).robot(idx))

cdef Robots RobotsFromPtr(shared_ptr[c_mc_rbdyn.Robots] p):
        cdef Robots ret = Robots(skip_alloc = True)
        ret.impl = p
        return ret

cdef Robots RobotsFromRawPtr(c_mc_rbdyn.Robots * p):
        cdef Robots ret = Robots(skip_alloc = True)
        ret.impl = c_mc_rbdyn.robots_fake_shared(p)
        return ret

cdef Robots RobotsFromRef(c_mc_rbdyn.Robots & p):
        return RobotsFromRawPtr(&p)

cdef class Limits(object):
    def __cinit__(self):
        self.impl = NULL
    property ql:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.ql)
    property qu:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.qu)
    property vl:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.vl)
    property vu:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.vu)
    property al:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.al)
    property au:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.au)
    property tl:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.tl)
    property tu:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.tu)
    property tdl:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.tdl)
    property tdu:
        def __get__(self):
            assert(self.impl)
            return eigen.VectorXdFromC(self.impl.tdu)

cdef Limits LimitsFromC(c_mc_rbdyn.Limits & p):
    cdef Limits out = Limits()
    out.impl = &p
    return out

cdef class Frame(object):
    def __cinit__(self):
        self.impl = NULL
    def name(self):
        assert(self.impl)
        return self.impl.name()
    def body(self):
        assert(self.impl)
        return self.impl.body()
    def X_b_f(self):
        assert(self.impl)
        return sva.PTransformdFromC(self.impl.X_b_f())
    def robot(self):
        assert(self.impl)
        return RobotFromC(self.impl.robot())
    def position(self):
        assert(self.impl)
        return sva.PTransformdFromC(self.impl.position())
    def velocity(self):
        assert(self.impl)
        return sva.MotionVecdFromC(self.impl.velocity())
    def normalAcceleration(self):
        assert(self.impl)
        return sva.MotionVecdFromC(self.impl.normalAcceleration())
    def hasForceSensor(self):
        assert(self.impl)
        return self.impl.hasForceSensor()
    def forceSensor(self):
        assert(self.impl)
        return ForceSensorFromCRef(self.impl.forceSensor())
    def wrench(self):
        assert(self.impl)
        return sva.ForceVecdFromC(self.impl.wrench())

cdef Frame FrameFromC(c_mc_rbdyn.Frame & f):
    cdef Frame out = Frame()
    out.impl = &f
    return out

cdef class CoM(object):
    def __cinit__(self):
        self.impl = NULL
    def com(self):
        assert(self.impl)
        return eigen.Vector3dFromC(self.impl.com())
    def velocity(self):
        assert(self.impl)
        return eigen.Vector3dFromC(self.impl.velocity())
    def normalAcceleration(self):
        assert(self.impl)
        return eigen.Vector3dFromC(self.impl.normalAcceleration())
    def acceleration(self):
        assert(self.impl)
        return eigen.Vector3dFromC(self.impl.acceleration())
    def robot(self):
        assert(self.impl)
        return RobotFromC(self.impl.robot())

cdef CoM CoMFromC(c_mc_rbdyn.CoM & c):
    cdef CoM out = CoM()
    out.impl = &c
    return out

cdef class Convex(object):
    def __cinit__(self):
        self.impl = NULL
    def frame(self):
        assert(self.impl)
        return FrameFromC(self.impl.frame())
    def X_f_c(self):
        assert(self.impl)
        return sva.PTransformdFromC(self.impl.X_f_c())
    def convex(self):
        assert(self.impl)
        return sch.S_ObjectFromPtr(self.impl.convex().get())

cdef Convex ConvexFromC(c_mc_rbdyn.Convex & c):
    cdef Convex out = Convex()
    out.impl = &c
    return out

cdef class Robot(object):
    def __is_valid(self):
        assert self.impl, "This Robot instance has not been initialized correctly"
    def __cinit__(self, *args):
        if len(args):
            raise TypeError("You cannot create a stand-alone Robot, please go through a Robots")
        self.impl = NULL
    def name(self):
        self.__is_valid()
        return self.impl.name()
    def hasJoint(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.hasJoint(name)
    def hasBody(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.hasBody(name)
    def jointIndexByName(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.jointIndexByName(name)
    def bodyIndexByName(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.bodyIndexByName(name)

    def forceSensor(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return ForceSensorFromRef(self.impl.forceSensor(name))
    def hasForceSensor(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.hasForceSensor(name)
    def frameForceSensor(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return ForceSensorFromRef(self.impl.frameForceSensor(name))
    def frameHasForceSensor(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.frameHasForceSensor(name)

    def bodySensor(self, name = None):
        self.__is_valid()
        if name is None:
            if isinstance(name, unicode):
                name = name.encode(u'ascii')
            return BodySensorFromRef(self.impl.bodySensor())
        else:
            return BodySensorFromRef(self.impl.bodySensor(name))
    def bodySensors(self):
        self.__is_valid()
        size = c_mc_rbdyn.getBodySensorsSize(deref(self.impl))
        ret = []
        for i in range(size):
                ret.append(BodySensorFromCRef(c_mc_rbdyn.getBodySensor(deref(self.impl), i)))
        return ret
    def hasBodySensor(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.hasBodySensor(name)
    def frameHasBodySensor(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.frameHasBodySensor(name)
    def frameBodySensor(self, name):
        self.__is_valid()
        return BodySensorFromRef(self.impl.frameBodySensor(name))

    def hasFrame(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.hasFrame(name)
    def frame(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return FrameFromC(self.impl.frame(name))

    def cop(self, surfaceName, min_pressure):
            self.__is_valid()
            return eigen.Vector2dFromC(self.impl.cop(surfaceName, min_pressure))

    def copW(self, Robot robot, surfaceName, min_pressure):
            self.__is_valid()
            return eigen.Vector3dFromC(self.impl.copW(surfaceName,min_pressure))

    def zmp(self, vector[string] sensorsName, eigen.Vector3d plane_p, eigen.Vector3d plane_n, forceThreshold):
            self.__is_valid()
            return eigen.Vector3dFromC(self.impl.zmp(sensorsName, plane_p.impl, plane_n.impl, forceThreshold))

    def limits(self):
        self.__is_valid()
        return LimitsFromC(self.impl.limits())

    def com(self):
        self.__is_valid()
        return CoMFromC(self.impl.com())

    property mb:
        def __get__(self):
            self.__is_valid()
            return rbdyn.MultiBodyFromC(self.impl.mb(), False)
    property mbc:
        def __get__(self):
            self.__is_valid()
            return rbdyn.MultiBodyConfigFromC(self.impl.mbc(), False)
    property mbg:
        def __get__(self):
            self.__is_valid()
            return rbdyn.MultiBodyGraphFromC(self.impl.mbg(), False)

    def flexibility(self):
        self.__is_valid()
        end = deref(self.impl).flexibility().end()
        it = deref(self.impl).flexibility().begin()
        ret = []
        while it != end:
            ret.append(FlexibilityFromC(deref(it)))
            preinc(it)
        return ret

    def hasSurface(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return self.impl.hasSurface(name)
    def surface(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return SurfaceFromC(self.impl.surface(name))
    def surfaces(self):
        self.__is_valid()
        availableSurfaces = self.impl.availableSurfaces()
        ret = {}
        for s in availableSurfaces:
            ret[s] = SurfaceFromC(self.impl.surface(s))
        return ret

    def convex(self, name):
        self.__is_valid()
        if isinstance(name, unicode):
            name = name.encode(u'ascii')
        return ConvexFromC(self.impl.convex(name))
    def convexes(self):
        self.__is_valid()
        end = deref(self.impl).convexes().const_end()
        it = deref(self.impl).convexes().const_begin()
        ret = {}
        while it != end:
            ret[deref(it).first] = self.convex(deref(it).first)
            preinc(it)
        return ret

    def stance(self):
        self.__is_valid()
        end = deref(self.impl).stance().const_end()
        it = deref(self.impl).stance().const_begin()
        out = {}
        while it != end:
            out[deref(it).first] = deref(it).second
            preinc(it)
        return out

    def forwardKinematics(self):
        self.__is_valid()
        self.impl.forwardKinematics()

    def forwardVelocity(self):
        self.__is_valid()
        self.impl.forwardVelocity()

    def forwardAcceleration(self):
        self.__is_valid()
        self.impl.forwardAcceleration()

    def posW(self, sva.PTransformd pt = None):
        self.__is_valid()
        if pt is None:
            return sva.PTransformdFromC(self.impl.posW())
        else:
            self.impl.posW(deref(pt.impl))

    def module(self):
            self.__is_valid()
            return RobotModuleFromCRef(self.impl.module())


cdef Robot RobotFromC(const c_mc_rbdyn.Robot & robot):
    cdef Robot ret = Robot()
    ret.impl = &(c_mc_rbdyn.const_cast_robot(robot))
    return ret

cdef class Surface(object):
    def __cinit__(self):
        pass

    property name:
        def __get__(self):
            assert(self.impl)
            return self.impl.name()

    def points(self):
        assert(self.impl)
        # Force the C++ compiler to use the const variant
        return sva.PTransformdVectorFromC((<const c_mc_rbdyn.Surface*>(self.impl)).points())

    def X_b_s(self):
        assert(self.impl)
        return sva.PTransformdFromC(self.impl.X_b_s())

    def frame(self):
        assert(self.impl)
        return FrameFromC(self.impl.frame())

    def robot(self):
        assert(self.impl)
        return RobotFromC(self.impl.robot())

    def type(self):
        assert(self.impl)
        return self.impl.type()

cdef SurfaceFromC(const c_mc_rbdyn.Surface & surface):
    cdef Surface ret = Surface()
    cdef c_mc_rbdyn.Surface * ptr = &(c_mc_rbdyn.const_cast_surface(surface))
    s_type = str(ptr.type())
    if s_type == "planar":
        ret = PlanarSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_planar_surface(ptr))
    elif s_type == "gripper":
        ret = GripperSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_gripper_surface(ptr))
    elif s_type == "cylindrical":
        ret = CylindricalSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_cylindrical_surface(ptr))
    else:
        print "Unknown Surface type:",ptr.type()
        ret.impl = ptr
    return ret

cdef class PlanarSurface(Surface):
    def __cinit__(self):
        pass
    def planarPoints(self):
        assert(self.surf)
        return self.surf.planarPoints()

cdef PlanarSurface PlanarSurfaceFromPtr(c_mc_rbdyn.PlanarSurface * surf):
        cdef PlanarSurface ret = PlanarSurface()
        ret.surf = ret.impl = surf
        return ret

cdef class GripperSurface(Surface):
    def __cinit__(self):
        pass
    def pointsFromOrigin(self):
        assert(self.surf)
        end = self.surf.pointsFromOrigin().const_end()
        it = self.surf.pointsFromOrigin().const_begin()
        out = []
        while it != end:
            out.append(sva.PTransformdFromC(deref(it)))
            preinc(it)
        return out
    def X_b_motor(self):
        assert(self.surf)
        return sva.PTransformdFromC(self.surf.X_b_motor())
    def motorMaxTorque(self):
        assert(self.surf)
        return self.surf.motorMaxTorque()

cdef GripperSurface GripperSurfaceFromPtr(c_mc_rbdyn.GripperSurface * surf):
        cdef GripperSurface ret = GripperSurface()
        ret.surf = ret.impl = surf
        return ret

cdef class CylindricalSurface(Surface):
    def __cinit__(self):
        pass
    def radius(self):
        assert(self.surf)
        return self.surf.radius()
    def width(self):
        assert(self.surf)
        return self.surf.width()

cdef CylindricalSurface CylindricalSurfaceFromPtr(c_mc_rbdyn.CylindricalSurface * surf):
        cdef CylindricalSurface ret = CylindricalSurface()
        ret.surf = ret.impl = surf
        return ret

cdef class Contact(object):
    defaultFriction = c_mc_rbdyn.ContactdefaultFriction
    def __cinit__(self, r1, r2, r1Surface, r2Surface, friction = defaultFriction, eigen.Vector6d dof = eigen.Vector6d(*[1]*6)):
        if isinstance(r1, unicode):
            r1 = r1.encode(u'ascii')
        if isinstance(r2, unicode):
            r2 = r2.encode(u'ascii')
        if isinstance(r1Surface, unicode):
            r1Surface = r1Surface.encode(u'ascii')
        if isinstance(r2Surface, unicode):
            r2Surface = r2Surface.encode(u'ascii')
        self.impl = c_mc_rbdyn.Contact(r1, r2, r1Surface, r2Surface, friction, dof.impl)
    property r1:
        def __get__(self):
            return self.impl.r1
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.r1 = value
    property r2:
        def __get__(self):
            return self.impl.r2
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.r2 = value
    property r1Surface:
        def __get__(self):
            return self.impl.r1Surface
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.r1Surface = value
    property r2Surface:
        def __get__(self):
            return self.impl.r2Surface
        def __set__(self, value):
            if isinstance(value, unicode):
                value = value.encode(u'ascii')
            self.impl.r2Surface = value
    def __richcmp__(Contact self, Contact other, int op):
        if op == 2:
            return self.impl == other.impl
        else:
            raise NotImplementedError("This comparison is not supported")

cdef Contact ContactFromC(c_mc_rbdyn.Contact& c, cppbool copy=True):
    cdef Contact ret = Contact()
    ret.impl = c
    return ret

cdef class GeosGeomGeometry(object):
    def __cinit__(self):
        pass

cdef GeosGeomGeometry GeosGeomGeometryFromSharedPtr(shared_ptr[c_mc_rbdyn.Geometry] p):
    cdef GeosGeomGeometry ret = GeosGeomGeometry()
    ret.impl = p
    return ret

cdef class PolygonInterpolator(object):
    def __dealloc__(self):
        if self.__own_impl:
            del self.impl
    def __cinit__(self, *args, skip_alloc = False):
        if skip_alloc:
            assert(len(args) == 0)
            self.impl = NULL
            self.__own_impl = False
        else:
            self.__own_impl = True
            assert(len(args) == 1)
            data_path = args[0]
            if isinstance(data_path, unicode):
                data_path = data_path.encode(u'ascii')
            data = json.load(data_path)
            tuple_pairs = []
            for p in data["tuple_pairs"]:
                p1 = map(float, p["p1"])
                p2 = map(float, p["p2"])
                tuple_pairs.append(((p1[0], p1[1]), (p2[0], p2[1])))
            self.impl = c_mc_rbdyn.polygonInterpolatorFromTuplePairs(tuple_pairs)
    def fast_interpolate(self, percent):
        return GeosGeomGeometryFromSharedPtr(self.impl.fast_interpolate(percent))

def points_from_polygon(GeosGeomGeometry geom):
    cdef vector[c_eigen.Vector3d] vp = c_mc_rbdyn.points_from_polygon(geom.impl)
    return [eigen.Vector3dFromC(v) for v in vp]

def loadRobot(RobotModule module):
    robots = Robots()
    robots.load(module, module.name)
    return robots

def loadRobots(robot_modules):
    robots = Robots()
    for rm in robot_modules:
        robots.load(rm, rm.name)
    return robots
