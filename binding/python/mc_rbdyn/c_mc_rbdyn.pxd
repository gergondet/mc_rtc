#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen cimport *
from sva.c_sva cimport *
from rbdyn.c_rbdyn cimport *
cimport sch.c_sch as sch

from mc_rtc.c_mc_rtc cimport map as cppmap

from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<memory>" namespace "std" nogil:
    cdef cppclass shared_ptr[T]:
        shared_ptr()
        shared_ptr(T*)
        T* get()
        T& operator*()

cdef extern from "<mc_rbdyn/Collision.h>" namespace "mc_rbdyn":
    cdef cppclass CollisionDescription:
        CollisionDescription()
        CollisionDescription(string, string, double, double, double)

        string object1
        string object2
        double iDist
        double sDist
        double damping

    cdef cppclass Collision(CollisionDescription):
        Collision()
        Collision(const Collision &)
        Collision(string, string, string, string, double, double, double)

        string robot1
        string robot2

        cppbool operator==(const Collision&)
        cppbool operator!=(const Collision&)

    cdef cppclass CollisionVector:
        CollisionVector()
        CollisionVector(string, const vector[CollisionDescription]&)
        CollisionVector(string, string, const vector[CollisionDescription]&)
        CollisionVector(const vector[Collision]&)
        CollisionVector(const CollisionVector&)

cdef extern from "<mc_rbdyn/BodySensor.h>" namespace "mc_rbdyn":
    cdef cppclass BodySensor:
        BodySensor()

        string name()
        string parentBody()
        const PTransformd & X_b_s()
        const Vector3d & position()
        const Quaterniond & orientation()
        const Vector3d & linearVelocity()
        const Vector3d & angularVelocity()
        const Vector3d & linearAcceleration()

cdef extern from "<mc_rbdyn/Flexibility.h>" namespace "mc_rbdyn":
    cdef cppclass Flexibility:
        Flexibility()
        Flexibility(const Flexibility&)
        string jointName
        double K
        double C
        double O

cdef extern from "<mc_rbdyn/ForceSensor.h>" namespace "mc_rbdyn":
    cdef cppclass ForceSensor:
        ForceSensor()
        ForceSensor(const string&, const string&, const PTransformd&)
        ForceSensor(const ForceSensor&)

        string name()
        string parentBody()
        const PTransformd & X_p_f()
        const ForceVecd & wrench()
        double mass()
        ForceVecd wrenchWithoutGravity(const Robot &)
        ForceVecd worldWrench(const Robot &)
        ForceVecd worldWrenchWithoutGravity(const Robot &)

cdef extern from "<mc_rbdyn/Springs.h>" namespace "mc_rbdyn":
    cdef cppclass Springs:
        Springs()
        Springs(const Springs&)
        vector[string] springsBodies
        vector[string] afterSpringsBodies
        vector[vector[string]] springsJoints

cdef extern from "<mc_rbdyn/Base.h>" namespace "mc_rbdyn":
    cdef cppclass Base:
        Base()
        Base(const Base&)
        string baseName
        PTransformd X_0_s
        PTransformd X_b0_s
        JointType baseType

cdef extern from "<mc_rbdyn/RobotModule.h>" namespace "mc_rbdyn":
    cdef cppclass RobotModule:
        RobotModule(const string &, const string &)
        RobotModule(const string &, const string &, const string&)

        const vector[cppmap[string, vector[double]]] & bounds()
        const cppmap[string, vector[double]] & stance()
        const cppmap[string, pair[string, string]] & convexHull()
        cppmap[string, PTransformd] _collisionTransforms
        vector[Flexibility] _flexibility
        vector[ForceSensor] _forceSensors
        const Springs & springs()
        vector[CollisionDescription] _minimalSelfCollisions
        vector[CollisionDescription] _commonSelfCollisions
        const vector[string]& ref_joint_order()

        string path
        string name
        string urdf_path
        string rsdf_dir
        string calib_dir
        MultiBody mb
        MultiBodyConfig mbc
        MultiBodyGraph mbg

    ctypedef shared_ptr[RobotModule] RobotModulePtr

cdef extern from "<mc_rbdyn/Robots.h>" namespace "mc_rbdyn":
    cdef cppclass Robots:
        Robots()
        Robots(const Robots &)

        const vector[shared_ptr[Robot]] & robots()

        const Robot & robot()

        Robot & load(const RobotModule&, string)

        const Robot & robot(string)

cdef extern from "<mc_rbdyn/Limits.h>" namespace "mc_rbdyn":
    cdef cppclass Limits:
        VectorXd ql
        VectorXd qu
        VectorXd vl
        VectorXd vu
        VectorXd al
        VectorXd au
        VectorXd tl
        VectorXd tu
        VectorXd tdl
        VectorXd tdu

cdef extern from "<mc_rbdyn/FreeFrame.h>" namespace "mc_rbdyn":
    cdef cppclass FreeFrame:
        string name()
        PTransformd X_p_f()
        PTransformd position()
        MotionVecd velocity()

cdef extern from "<mc_rbdyn/RobotFrame.h>" namespace "mc_rbdyn":
    cdef cppclass RobotFrame(FreeFrame):
        Robot & robot()
        MotionVecd normalAcceleration()
        string body()
        bool hasForceSensor()
        const ForceSensor & forceSensor()
        ForceVecd wrench()

cdef extern from "<mc_rbdyn/CoM.h>" namespace "mc_rbdyn":
    cdef cppclass CoM:
        Vector3d com()
        Vector3d velocity()
        Vector3d normalAcceleration()
        Vector3d acceleration()
        Robot & robot()

cdef extern from "<mc_rbdyn/Convex.h>" namespace "mc_rbdyn":
    cdef cppclass Convex:
        FreeFrame & frame()
        shared_ptr[sch.S_Object] convex()
        PTransformd X_f_c()

    ctypedef shared_ptr[Convex] ConvexPtr

cdef extern from "<mc_rbdyn/Robot.h>" namespace "mc_rbdyn":
    cdef cppclass Robot:
        string name()
        cppbool hasJoint(string)
        cppbool hasBody(string)
        unsigned int jointIndexByName(string)
        unsigned int bodyIndexByName(string)

        bool hasForceSensor(string)
        ForceSensor& forceSensor(string)
        bool frameHasForceSensor(string)
        ForceSensor & frameForceSensor(string)

        BodySensor& bodySensor()
        bool hasBodySensor(string)
        BodySensor& bodySensor(string)
        bool frameHasBodySensor(string)
        BodySensor& frameBodySensor(string)

        bool hasFrame(string)
        RobotFrame & frame(string)

        const MultiBody& mb()
        const MultiBodyConfig& mbc()
        const MultiBodyGraph& mbg()

        const Limits & limits()

        CoM & com()

        Vector2d cop(string,double)
        Vector3d copW(string,double)
        Vector3d zmp(const vector[string]&,const Vector3d&,const Vector3d&,double)

        bool hasSurface(string)
        const Surface& surface(string)
        const cppmap[string, shared_ptr[Surface]]& surfaces()
        vector[string] availableSurfaces()

        Convex& convex(string)
        const cppmap[string, ConvexPtr] & convexes()

        cppmap[string, vector[double]] stance()

        vector[Flexibility] flexibility()

        void forwardKinematics()
        void forwardVelocity()
        void forwardAcceleration()

        PTransformd posW()
        void posW(PTransformd pt)

        RobotModule & module()

cdef extern from "<mc_rbdyn/Surface.h>" namespace "mc_rbdyn":
    cdef cppclass Surface:
        string name()
        string type()
        RobotFrame & frame()
        Robot & robot()

        const vector[PTransformd]& points()

        const PTransformd& X_b_s()

    ctypedef shared_ptr[Surface] SurfacePtr

cdef extern from "<mc_rbdyn/PlanarSurface.h>" namespace "mc_rbdyn":
    cdef cppclass PlanarSurface(Surface):
        const vector[pair[double,double]]& planarPoints()

    PlanarSurface * dynamic_cast_planar_surface"dynamic_cast<mc_rbdyn::PlanarSurface*>"(Surface*)

cdef extern from "<mc_rbdyn/GripperSurface.h>" namespace "mc_rbdyn":
    cdef cppclass GripperSurface(Surface):
        const vector[PTransformd]& pointsFromOrigin()
        const PTransformd& X_b_motor()
        double motorMaxTorque()

    GripperSurface * dynamic_cast_gripper_surface"dynamic_cast<mc_rbdyn::GripperSurface*>"(Surface*)

cdef extern from "<mc_rbdyn/CylindricalSurface.h>" namespace "mc_rbdyn":
    cdef cppclass CylindricalSurface(Surface):
        double radius()
        double width()

    CylindricalSurface * dynamic_cast_cylindrical_surface"dynamic_cast<mc_rbdyn::CylindricalSurface*>"(Surface*)

cdef extern from "<mc_rbdyn/Contact.h>" namespace "mc_rbdyn":
    cdef cppclass Contact:
        Contact()
        Contact(string, string, string, string, double, Vector6d)

        string r1
        string r2
        string r1Surface
        string r2Surface
        double friction
        Vector6d dof

        cppbool operator==(const Contact&)

    cdef double ContactdefaultFriction "mc_rbdyn::Contact::defaultFriction"

cdef extern from "<geos/geom/Geometry.h>" namespace "geos::geom":
    cdef cppclass Geometry:
            pass

cdef extern from "<mc_rbdyn/PolygonInterpolator.h>" namespace "mc_rbdyn":
    cdef cppclass PolygonInterpolator:
     # Actual constructor
     # PolygonInterpolator(const Json::Value&)
     shared_ptr[Geometry] fast_interpolate(double)

cdef extern from "<mc_rbdyn/polygon_utils.h>" namespace "mc_rbdyn":
    cdef vector[Vector3d] points_from_polygon(shared_ptr[Geometry])

cdef extern from "mc_rbdyn_wrapper.hpp" namespace "mc_rbdyn":
    #FIXME Work-around the lack of variadic template support
    RobotModulePtr get_robot_module(const string&) except +
    RobotModulePtr get_robot_module(const string&, const string&) except +
    RobotModulePtr get_robot_module(const string&, const string&, const string&) except +
    #XXX
    void update_robot_module_path(const vector[string] &)
    void clear_robot_module_path()
    vector[string] available_robots()
    Robots& const_cast_robots(const Robots&)
    Robot& const_cast_robot(const Robot&)
    ForceSensor& const_cast_force_sensor(const ForceSensor &)
    BodySensor& const_cast_body_sensor(const BodySensor &)
    Surface& const_cast_surface(const Surface&)
    Contact& const_cast_contact(const Contact&)
    vector[Contact]& const_cast_contact_vector(const vector[Contact]&)
    void contact_vector_set_item(vector[Contact]&, unsigned int, const Contact&)
    shared_ptr[Robots] robots_fake_shared(Robots*)
    PolygonInterpolator * polygonInterpolatorFromTuplePairs(const vector[pair[pair[double, double], pair[double, double]]]&)
    #FIXME Work-around lack of array support
    vector[double] robotModuleDefaultAttitude(RobotModulePtr rm)
    #XXX
    #FIXME Lack of support for vector[T,A] in Cython 0.20
    unsigned int getBodySensorsSize[T](T &)
    (BodySensor&) getBodySensor[T](T &, unsigned int)
    #XXX
    RobotModulePtr copyRobotModule(const RobotModule &)
    #FIXME Lack of brace-initialization support
    CollisionDescription makeCollisionDescription(string, string, double, double, double)
