# distutils: language = c++

#
# Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_tasks.force.c_force as c_force

cimport mc_rbdyn.mc_rbdyn as mc_rbdyn
cimport mc_tasks.c_mc_tasks as c_mc_tasks
cimport mc_tasks.mc_tasks as mc_tasks
cimport mc_tvm.c_mc_tvm as c_mc_tvm

from mc_tasks.mc_tasks cimport MetaTask, TransformTask

cimport sva.sva as sva

cimport eigen.eigen as eigen

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.pair cimport pair

import numbers

cdef class ImpedanceVecdStrictlyPositive(object):
    def __cinit__(self):
        self.impl = NULL
    def vector(self):
        assert(self.impl)
        return eigen.Vector6dFromC(self.impl.vector())
    def vec(self, angular = None, linear = None):
        assert(self.impl)
        if linear is None and angular is None:
            return sva.ImpedanceVecdFromC(self.impl.vec())
        if linear is not None:
            assert(angular is not None)
            if isinstance(angular, numbers.Number):
                assert(isinstance(linear, numbers.Number))
                self.impl.vec(<double>angular, <double>linear)
            else:
                if not isinstance(angular, eigen.Vector3d):
                    angular = eigen.Vector3d(angular)
                if not isinstance(linear, eigen.Vector3d):
                    linear = eigen.Vector3d(linear)
                self.impl.vec((<eigen.Vector3d>angular).impl, (<eigen.Vector3d>linear).impl)
        else:
            if isinstance(angular, sva.ImpedanceVecd):
                self.impl.vec((<sva.ImpedanceVecd>angular).impl)
            else:
                if not isinstance(angular, eigen.Vector6d):
                    angular = eigen.Vector6d(angular)
                self.impl.vec((<eigen.Vector6d>angular).impl)
    def angular(self, value = None):
        assert(self.impl)
        if value is None:
            return eigen.Vector3dFromC(self.impl.angular())
        else:
            if isinstance(value, numbers.Number):
                self.impl.angular(<double>value)
            else:
                if not isinstance(value, eigen.Vector3d):
                    value = eigen.Vector3d(value)
                self.impl.angular((<eigen.Vector3d>value).impl)
    def linear(self, value = None):
        assert(self.impl)
        if value is None:
            return eigen.Vector3dFromC(self.impl.linear())
        else:
            if isinstance(value, numbers.Number):
                self.impl.linear(<double>value)
            else:
                if not isinstance(value, eigen.Vector3d):
                    value = eigen.Vector3d(value)
                self.impl.linear((<eigen.Vector3d>value).impl)

cdef ImpedanceVecdStrictlyPositive ImpedanceVecdStrictlyPositiveFromRef(c_force.ImpedanceVecdStrictlyPositive & iv):
    cdef ImpedanceVecdStrictlyPositive ret = ImpedanceVecdStrictlyPositive()
    ret.impl = &iv
    return ret

cdef class ImpedanceVecdPositive(object):
    def __cinit__(self):
        self.impl = NULL
    def vector(self):
        assert(self.impl)
        return eigen.Vector6dFromC(self.impl.vector())
    def vec(self, angular = None, linear = None):
        assert(self.impl)
        if linear is None and angular is None:
            return sva.ImpedanceVecdFromC(self.impl.vec())
        if linear is not None:
            assert(angular is not None)
            if isinstance(angular, numbers.Number):
                assert(isinstance(linear, numbers.Number))
                self.impl.vec(<double>angular, <double>linear)
            else:
                if not isinstance(angular, eigen.Vector3d):
                    angular = eigen.Vector3d(angular)
                if not isinstance(linear, eigen.Vector3d):
                    linear = eigen.Vector3d(linear)
                self.impl.vec((<eigen.Vector3d>angular).impl, (<eigen.Vector3d>linear).impl)
        else:
            if isinstance(angular, sva.ImpedanceVecd):
                self.impl.vec((<sva.ImpedanceVecd>angular).impl)
            else:
                if not isinstance(angular, eigen.Vector6d):
                    angular = eigen.Vector6d(angular)
                self.impl.vec((<eigen.Vector6d>angular).impl)
    def angular(self, value = None):
        assert(self.impl)
        if value is None:
            return eigen.Vector3dFromC(self.impl.angular())
        else:
            if isinstance(value, numbers.Number):
                self.impl.angular(<double>value)
            else:
                if not isinstance(value, eigen.Vector3d):
                    value = eigen.Vector3d(value)
                self.impl.angular((<eigen.Vector3d>value).impl)
    def linear(self, value = None):
        assert(self.impl)
        if value is None:
            return eigen.Vector3dFromC(self.impl.linear())
        else:
            if isinstance(value, numbers.Number):
                self.impl.linear(<double>value)
            else:
                if not isinstance(value, eigen.Vector3d):
                    value = eigen.Vector3d(value)
                self.impl.linear((<eigen.Vector3d>value).impl)

cdef ImpedanceVecdPositive ImpedanceVecdPositiveFromRef(c_force.ImpedanceVecdPositive & iv):
    cdef ImpedanceVecdPositive ret = ImpedanceVecdPositive()
    ret.impl = &iv
    return ret

cdef class ImpedanceGains(object):
    def __cinit__(self):
        self.impl = NULL
    def mass(self):
        assert(self.impl)
        return ImpedanceVecdStrictlyPositiveFromRef(self.impl.mass())
    def damper(self):
        assert(self.impl)
        return ImpedanceVecdPositiveFromRef(self.impl.damper())
    def spring(self):
        assert(self.impl)
        return ImpedanceVecdPositiveFromRef(self.impl.spring())
    def wrench(self):
        assert(self.impl)
        return ImpedanceVecdPositiveFromRef(self.impl.wrench())

cdef ImpedanceGains ImpedanceGainsFromRef(c_force.ImpedanceGains & ig):
    cdef ImpedanceGains ret = ImpedanceGains()
    ret.impl = &ig
    return ret

cdef class ImpedanceTask(TransformTask):
    def __cinit__(self, mc_rbdyn.RobotFrame frame, double stiffness = 5.0, double weight = 1000.0):
        self.impedance_impl = c_force.make_shared_aligned[c_force.ImpedanceTask](deref(frame.impl), stiffness, weight)
        self.impl = c_force.cast[c_mc_tasks.TransformTask](self.impedance_impl)
        self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]](self.impl)
        self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
    def gains(self):
        assert(self.impedance_impl.get())
        return ImpedanceGainsFromRef(deref(self.impedance_impl).gains())
    def targetPose(self, sva.PTransformd pose = None):
        assert(self.impendace_impl.get())
        if pose is None:
            return sva.PTransformdFromC(deref(self.impedance_impl).targetPose())
        else:
            deref(self.impedance_impl).targetPose(deref(pose.impl))
    def targetVel(self, sva.MotionVecd vel = None):
        assert(self.impendace_impl.get())
        if vel is None:
            return sva.MotionVecdFromC(deref(self.impedance_impl).targetVel())
        else:
            deref(self.impedance_impl).targetVel(deref(vel.impl))
    def targetAccel(self, sva.MotionVecd accel = None):
        assert(self.impendace_impl.get())
        if accel is None:
            return sva.MotionVecdFromC(deref(self.impedance_impl).targetAccel())
        else:
            deref(self.impedance_impl).targetAccel(deref(accel.impl))
    def targetWrench(self, sva.ForceVecd wrench = None):
        assert(self.impendace_impl.get())
        if wrench is None:
            return sva.ForceVecdFromC(deref(self.impedance_impl).targetWrench())
        else:
            deref(self.impedance_impl).targetWrench(deref(wrench.impl))
    def targetWrenchW(self, sva.ForceVecd wrench):
        assert(self.impendace_impl.get())
        deref(self.impedance_impl).targetWrenchW(deref(wrench.impl))
    def cutoffPeriod(self, p = None):
        assert(self.impedance_impl.get())
        if p is None:
            return deref(self.impedance_impl).cutoffPeriod()
        else:
            deref(self.impedance_impl).cutoffPeriod(p)

cdef class ComplianceTask(MetaTask):
    defaultFGain = c_force.defaultFGain
    defaultTGain = c_force.defaultTGain
    def __cinit__(self, mc_rbdyn.RobotFrame frame, eigen.Vector6d dof = eigen.Vector6d(*6*[1]), double stiffness = 5.0, double weight = 1000.0, double forceThreshold = 3.0, double torqueThreshold = 1.0, pair[double, double] forceGain = defaultFGain, pair[double, double] torqueGain = defaultTGain):
        self.impl = c_force.make_shared_aligned[c_force.ComplianceTask](deref(frame.impl), dof.impl, stiffness, weight, forceThreshold, torqueThreshold, forceGain, torqueGain)
        self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)

    def setTargetWrench(self, wrench):
        assert(self.impl.get())
        if isinstance(wrench, sva.ForceVecd):
            deref(self.impl).setTargetWrench(deref((<sva.ForceVecd>(wrench)).impl))
        else:
            self.setTargetWrench(sva.ForceVecd(wrench))
    def getTargetWrench(self):
        assert(self.impl.get())
        return sva.ForceVecdFromC(deref(self.impl).getTargetWrench())
    def stiffness(self, value = None):
        assert(self.impl.get())
        if value is None:
            return deref(self.impl).stiffness()
        else:
            deref(self.impl).stiffness(value)
    def weight(self, value = None):
        assert(self.impl.get())
        if value is None:
            return deref(self.impl).weight()
        else:
            deref(self.impl).weight(value)
    def forceThreshold(self, value = None):
        assert(self.impl.get())
        if value is None:
            return deref(self.impl).forceThreshold()
        else:
            deref(self.impl).forceThreshold(value)
    def torqueThreshold(self, value = None):
        assert(self.impl.get())
        if value is None:
            return deref(self.impl).torqueThreshold()
        else:
            deref(self.impl).torqueThreshold(value)
    def forceGain(self, value = None):
        assert(self.impl.get())
        if value is None:
            return deref(self.impl).forceGain()
        else:
            deref(self.impl).forceGain(value)
    def torqueGain(self, value = None):
        assert(self.impl.get())
        if value is None:
            return deref(self.impl).torqueGain()
        else:
            deref(self.impl).torqueGain(value)
    def dof(self, eigen.Vector6d dof = None):
        assert(self.impl.get())
        if dof is None:
            return eigen.Vector6dFromC(deref(self.impl).dof())
        else:
            deref(self.impl).dof(dof.impl)

cdef class AdmittanceTask(TransformTask):
    def __cinit__(self, mc_rbdyn.RobotFrame frame, double stiffness = 5.0, double weight = 1000.0):
        self.adm_impl = c_force.make_shared_aligned[c_force.AdmittanceTask](deref(frame.impl), stiffness, weight)
        self.impl = c_force.cast[c_mc_tasks.TransformTask](self.adm_impl)
        self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]](self.impl)
        self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
    def targetPose(self, pos = None):
        if pos is None:
            return sva.PTransformdFromC(deref(self.adm_impl).targetPose())
        else:
            if isinstance(pos, sva.PTransformd):
                deref(self.adm_impl).targetPose(deref((<sva.PTransformd>pos).impl))
            else:
                self.targetPose(sva.PTransformd(pos))
    def targetWrench(self, wrench = None):
        if wrench is None:
            return sva.ForceVecdFromC(deref(self.adm_impl).targetWrench())
        else:
            if isinstance(wrench, sva.ForceVecd):
                deref(self.adm_impl).targetWrench(deref((<sva.ForceVecd>wrench).impl))
            else:
                self.targetWrench(sva.ForceVecd(wrench))
    def admittance(self, wrench = None):
        if wrench is None:
            return sva.ForceVecdFromC(deref(self.adm_impl).admittance())
        else:
            if isinstance(wrench, sva.ForceVecd):
                deref(self.adm_impl).admittance(deref((<sva.ForceVecd>wrench).impl))
            else:
                self.admittance(sva.ForceVecd(wrench))

cdef class DampingTask(AdmittanceTask):
    def __cinit__(self, mc_rbdyn.RobotFrame frame, double stiffness = 5.0, double weight = 1000.0):
        self.damping_impl = c_force.make_shared_aligned[c_force.DampingTask](deref(frame.impl), stiffness, weight)
        self.adm_impl = c_force.cast[c_force.AdmittanceTask](self.damping_impl)
        self.impl = c_force.cast[c_mc_tasks.TransformTask](self.adm_impl)
        self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]](self.impl)
        self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)

cdef class CoPTask(DampingTask):
    def __cinit__(self, mc_rbdyn.RobotFrame frame, double stiffness = 5.0, double weight = 1000.0):
        self.cop_impl = c_force.make_shared_aligned[c_force.CoPTask](deref(frame.impl), stiffness, weight)
        self.damping_impl = c_force.cast[c_force.DampingTask](self.cop_impl)
        self.adm_impl = c_force.cast[c_force.AdmittanceTask](self.damping_impl)
        self.impl = c_force.cast[c_mc_tasks.TransformTask](self.adm_impl)
        self.ttg_base = c_mc_tasks.cast[c_mc_tasks.TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]](self.impl)
        self.mt_base = c_mc_tasks.cast[c_mc_tasks.MetaTask](self.impl)
    def measuredCoP(self):
        assert(self.cop_impl.get())
        return eigen.Vector2dFromC(deref(self.cop_impl).measuredCoP())
    def measuredCoPW(self):
        assert(self.cop_impl.get())
        return eigen.Vector3dFromC(deref(self.cop_impl).measuredCoPW())
    def setZeroTargetWrench(self):
        assert(self.cop_impl.get())
        deref(self.cop_impl).setZeroTargetWrench()
    def targetCoP(self, eigen.Vector2d target = None):
        assert(self.cop_impl.get())
        if target is None:
            return eigen.Vector2dFromC(deref(self.cop_impl).targetCoP())
        else:
            deref(self.cop_impl).targetCoP(target.impl)
    def targetCoPW(self):
        assert(self.cop_impl.get())
        return eigen.Vector3dFromC(deref(self.cop_impl).targetCoPW())
    def targetForce(self, eigen.Vector3d force = None):
        assert(self.cop_impl.get())
        if force is None:
            return eigen.Vector3dFromC(deref(self.cop_impl).targetForce())
        else:
            deref(self.cop_impl).targetForce(force.impl)
