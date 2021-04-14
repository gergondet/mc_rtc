#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen cimport *
from sva.c_sva cimport *
from rbdyn.c_rbdyn cimport *
cimport sch.c_sch as sch
from mc_rbdyn.c_mc_rbdyn cimport *
from mc_solver.c_mc_solver cimport *
cimport mc_solver.c_mc_solver as c_mc_solver
cimport mc_observers.c_mc_observers as c_mc_observers
cimport mc_rtc.c_mc_rtc as c_mc_rtc
cimport mc_rtc.gui.c_gui as c_mc_rtc_gui
cimport mc_tasks.c_mc_tasks as c_mc_tasks

from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool


cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr(T*)
    T* get()

cdef extern from "<mc_control/mc_controller.h>" namespace "mc_control":
  cdef cppclass ControllerResetData:
    const vector[vector[double]] & q

  cdef cppclass MCController:
    cppbool run()
    void reset(const ControllerResetData&)

    void addContact(c_mc_rbdyn.Contact &)
    void removeContact(c_mc_rbdyn.Contact &)
    vector[c_mc_rbdyn.Contact] contacts()
    bool hasContact(c_mc_rbdyn.Contact &)

    void addCollisions(string, string, vector[c_mc_rbdyn.CollisionDescription] &)
    void removeCollisions(string, string)
    void removeCollisions(string, string, vector[c_mc_rbdyn.CollisionDescription] &)

    bool hasRobot(string)
    Robot & robot()
    Robot & robot(string)
    Robots & robots()

    Robot & realRobot()
    Robot & realRobot(string)
    Robots & realRobots()

    Robot & loadRobot(c_mc_rbdyn.RobotModulePtr, string, PTransformd)
    void removeRobot(string)

    void supported_robots(vector[string] &)

    c_mc_rtc.Configuration & config()

    c_mc_rtc.Logger & logger()

    c_mc_rtc_gui.StateBuilder & gui()

    QPSolver & solver()

    cppbool hasObserverPipeline(const string &)
    c_mc_observers.ObserverPipeline & observerPipeline()
    c_mc_observers.ObserverPipeline & observerPipeline(const string&)
    vector[c_mc_observers.ObserverPipeline] & observerPipelines()

cdef extern from "<mc_control/mc_python_controller.h>" namespace "mc_control":
  cdef cppclass MCPythonController(MCController):
    MCPythonController(const vector[RobotModulePtr]&, double)
    shared_ptr[c_mc_solver.KinematicsConstraint] kinematicsConstraint()
    shared_ptr[c_mc_solver.DynamicsConstraint] dynamicsConstraint()
    shared_ptr[c_mc_solver.CollisionsConstraint] collisionConstraint()
    shared_ptr[c_mc_tasks.PostureTask] postureTask()

cdef extern from "<array>" namespace "std" nogil:
  cdef cppclass array[T, N]:
    array() except +
    T& operator[](size_t)

cdef extern from *:
  ctypedef int seven "7"
  ctypedef array[double, seven] array7d


cdef extern from "<mc_control/mc_global_controller.h>" namespace "mc_control":
  cdef cppclass MCGlobalController:
    MCGlobalController()
    MCGlobalController(string)
    MCGlobalController(string, RobotModulePtr)

    void init(vector[double])
    void init(vector[double], array7d)

    void setSensorPosition(Vector3d)
    void setSensorOrientation(Quaterniond)
    void setSensorLinearVelocity(Vector3d)
    void setSensorAngularVelocity(Vector3d)
    void setSensorLinearAcceleration(Vector3d)
    void setEncoderValues(vector[double])
    void setEncoderVelocities(vector[double])
    void setFlexibilityValues(vector[double])
    void setJointTorques(vector[double])
    void setWrenches(cppmap[string, ForceVecd])

    cppbool run()

    double timestep()
    MCController& controller()
    vector[string] ref_joint_order()
    Robot& robot()

    cppbool running

cdef extern from "mc_control_wrapper.hpp" namespace "mc_control":
  ControllerResetData & const_cast_crd(const ControllerResetData&)

  ctypedef cppbool (*run_callback_t)(void*) except+
  ctypedef void (*reset_callback_t)(const ControllerResetData&, void*) except+

  void set_run_callback(MCPythonController&, run_callback_t fn, void*)
  void set_reset_callback(MCPythonController&, reset_callback_t fn, void *)

  void add_anchor_frame_callback[T, U](MCPythonController &, const string &, T, U)
  void remove_anchor_frame_callback(MCPythonController &, const string &)
