#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport eigen.c_eigen as c_eigen
cimport sva.c_sva as c_sva
cimport rbdyn.c_rbdyn as c_rbdyn
cimport sch.c_sch as c_sch
cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_solver.c_mc_solver as c_mc_solver
cimport mc_tvm.c_mc_tvm as c_mc_tvm

from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    T* get()

cdef extern from "<mc_tasks/MetaTask.h>" namespace "mc_tasks":
  cdef cppclass MetaTask:
    string name()
    void name(string)
    void reset()
    void dimWeight(const c_eigen.VectorXd &)
    c_eigen.VectorXd dimWeight()
    void selectActiveJoints(c_mc_solver.QPSolver &,
                            const vector[string] &)
    void selectUnactiveJoints(c_mc_solver.QPSolver &,
                            const vector[string] &)
    void resetJointsSelector(c_mc_solver.QPSolver &)
    c_eigen.VectorXd eval()
    c_eigen.VectorXd speed()

  ctypedef shared_ptr[MetaTask] MetaTaskPtr

cdef extern from "<mc_tasks/PostureTask.h>" namespace "mc_tasks":
  cdef cppclass PostureTask(MetaTask):
    PostureTask(const c_mc_solver.QPSolver&,
                unsigned int, double, double)
    void posture(const vector[vector[double]] &)
    vector[vector[double]] posture()
    #void jointGains(const c_mc_solver.QPSolver &, const vector[c_qp.JointGains])
    #void jointStiffness(const c_mc_solver.QPSolver &, const vector[c_qp.JointStiffness])
    void target(const cppmap[string, vector[double]] &)
    void stiffness(double)
    double stiffness()
    void weight(double)
    double weight()

cdef extern from "<mc_tasks/TrajectoryTaskGeneric.h>" namespace "mc_tasks":
  cdef cppclass TrajectoryTaskGeneric[T](MetaTask):
    void refVel(const c_eigen.VectorXd &)
    void refAccel(const c_eigen.VectorXd &)
    void stiffness(double)
    double stiffness()
    void setGains(double, double)
    double damping()
    void weight(double)
    double weight()

cdef extern from "<mc_tasks/CoMTask.h>" namespace "mc_tasks":
  cdef cppclass CoMTask(TrajectoryTaskGeneric[c_mc_tvm.CoMFunction]):
    CoMTask(const c_mc_rbdyn.Robots &, unsigned int,
            double, double)
    c_eigen.Vector3d com()
    void com(const c_eigen.Vector3d &)
    void move_com(const c_eigen.Vector3d &)

cdef extern from "<mc_tasks/PositionTask.h>" namespace "mc_tasks":
  cdef cppclass PositionTask(TrajectoryTaskGeneric[c_mc_tvm.PositionFunction]):
    PositionTask(const string &, const c_mc_rbdyn.Robots &,
                 unsigned int, double, double)
    c_eigen.Vector3d position()
    void position(const c_eigen.Vector3d &)

cdef extern from "<mc_tasks/OrientationTask.h>" namespace "mc_tasks":
  cdef cppclass OrientationTask(TrajectoryTaskGeneric[c_mc_tvm.OrientationFunction]):
    OrientationTask(const string &, const c_mc_rbdyn.Robots &,
                    unsigned int, double, double)
    c_eigen.Matrix3d orientation()
    void orientation(const c_eigen.Matrix3d &)

cdef extern from "<mc_tasks/VectorOrientationTask.h>" namespace "mc_tasks":
  cdef cppclass VectorOrientationTask(TrajectoryTaskGeneric[c_mc_tvm.VectorOrientationFunction]):
    VectorOrientationTask(const string &, const c_eigen.Vector3d &,
                          const c_eigen.Vector3d &, const c_mc_rbdyn.Robots &,
                          unsigned int, double, double)
    void bodyVector(const c_eigen.Vector3d&)
    c_eigen.Vector3d bodyVector()

cdef extern from "<mc_tasks/TransformTask.h>" namespace "mc_tasks":
  cdef cppclass TransformTask(TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]):
    TransformTask() # This does not exist but silence a compilation error on Cython 0.2
    TransformTask(const string &, const c_mc_rbdyn.Robots &,
                         unsigned int, double, double)
    c_sva.PTransformd target()
    void target(const c_sva.PTransformd &)

cdef extern from "<mc_tasks/SplineTrajectoryTask.h>" namespace "mc_tasks":
  cdef cppclass SplineTrajectoryTask[T](TrajectoryTaskGeneric[c_mc_tvm.TransformFunction]):
    SplineTrajectoryTask() # This does not exist but silence a compilation error on Cython 0.2
    SplineTrajectoryTask(const c_mc_rbdyn.Robots&,
            unsigned int, const string &, double, double, double,
            const c_eigen.Matrix3d &,
            const vector[pair[double,c_eigen.Matrix3d]] &)
    void oriWaypoints(const vector[pair[double,c_eigen.Matrix3d]] &)
    cppbool timeElapsed()
    c_sva.PTransformd target()
    void target(c_sva.PTransformd &)
    void refPose(const c_sva.PTransformd&)
    const c_sva.PTransformd & refPose()
    void displaySamples(unsigned int)
    unsigned int displaySamples()

cdef extern from "<mc_tasks/BSplineTrajectoryTask.h>" namespace "mc_tasks":
  cdef cppclass BSplineTrajectoryTask(SplineTrajectoryTask[BSplineTrajectoryTask]):
    BSplineTrajectoryTask(const c_mc_rbdyn.Robots&,
            unsigned int, const string &, double, double, double,
            const c_sva.PTransformd &)
    BSplineTrajectoryTask(const c_mc_rbdyn.Robots&,
            unsigned int, const string &, double, double, double,
            const c_sva.PTransformd &,
            const vector[c_eigen.Vector3d] &,
            const vector[pair[double,c_eigen.Matrix3d]] &)
    void posWaypoints(const vector[c_eigen.Vector3d] &)
    c_eigen.VectorXd evalTracking()

cdef extern from "<mc_tasks/ExactCubicTrajectoryTask.h>" namespace "mc_tasks":
  cdef cppclass ExactCubicTrajectoryTask(SplineTrajectoryTask[ExactCubicTrajectoryTask]):
    ExactCubicTrajectoryTask(const c_mc_rbdyn.Robots&,
            unsigned int, const string &, double, double, double,
            const c_sva.PTransformd &)
    ExactCubicTrajectoryTask(const c_mc_rbdyn.Robots&,
            unsigned int, const string &, double, double, double,
            const c_sva.PTransformd &,
            const vector[pair[double,c_eigen.Vector3d]] &,
            const c_eigen.Vector3d &,
            const c_eigen.Vector3d &,
            const c_eigen.Vector3d &,
            const c_eigen.Vector3d &,
            const vector[pair[double,c_eigen.Matrix3d]] &)
    void posWaypoints(const vector[pair[double,c_eigen.Vector3d]] &)
    void constraints(const c_eigen.Vector3d &,
            const c_eigen.Vector3d &,
            const c_eigen.Vector3d &,
            const c_eigen.Vector3d &)
    c_eigen.VectorXd evalTracking()
