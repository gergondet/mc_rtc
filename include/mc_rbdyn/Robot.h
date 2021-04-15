/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Base.h>
#include <mc_rbdyn/CoM.h>
#include <mc_rbdyn/Convex.h>
#include <mc_rbdyn/Limits.h>
#include <mc_rbdyn/Momentum.h>
#include <mc_rbdyn/RobotModule.h>

#include <mc_control/generic_gripper.h>

#include <mc_rtc/map.h>
#include <mc_rtc/shared.h>

#include <RBDyn/FD.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

#include <sch/S_Object/S_Object.h>

#include <tvm/Clock.h>
#include <tvm/Variable.h>
#include <tvm/VariableVector.h>
#include <tvm/graph/CallGraph.h>
#include <tvm/graph/abstract/Node.h>

#include <memory>
#include <optional>
#include <string_view>
#include <unordered_map>

namespace mc_rbdyn
{

/** Represent a robot managed by the optimization problem
 *
 * A Robot is created through a Robots container.
 *
 * It provides signals that are relevant for computing quantities related to the robot.
 *
 * It also acts as \ref Convex, \ref Frame and \ref Surface factories.
 *
 * Variables:
 * - q (split between free-flyer and joints)
 * - tau (see Outputs)
 *
 * Individual outputs:
 *
 * - FK: forward kinematics (computed by RBDyn::FK)
 * - FV: forward velocity (computed by RBDyn::FV), depends on FK
 * - FA: forward acceleration (computed by RBDyn::FA), depends on FV
 * - NormalAcceleration: update bodies' normal acceleration, depends on FV
 * - tau: generalized torque vector
 * - H: inertia matrix signal, depends on FV
 * - C: non-linear effect vector signal (Coriolis, gravity, external forces), depends on FV
 *
 * Meta outputs:
 *   These outputs are provided for convenience sake
 * - Geometry: depends on FK
 * - Dynamics: depends on FA + normalAcceleration (i.e. everything)
 *
 */
struct MC_RBDYN_DLLAPI Robot : public tvm::graph::abstract::Node<Robot>, mc_rtc::shared<Robot>
{
  SET_OUTPUTS(Robot, FK, FV, FA, NormalAcceleration, tau, H, C)
  SET_UPDATES(Robot, Time, FK, FV, FA, NormalAcceleration, H, C)

  friend struct Robots;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using mimic_variables_t = std::pair<tvm::VariableVector, Eigen::VectorXd>;

private:
  struct make_shared_token
  {
  };

public:
  /** Invoked by Robots parent instance after mb/mbc/mbg/RobotModule are stored
   *
   * When loadFiles is set to false, the convex and surfaces files are not
   * loaded. This is used when copying one robot into another.
   *
   */
  Robot(make_shared_token,
        RobotModule module,
        std::string_view name,
        bool loadFiles,
        const std::optional<sva::PTransformd> & base = std::nullopt,
        const std::optional<std::string_view> & baseName = std::nullopt);

  Robot(Robot &&) = default;
  Robot & operator=(Robot &&) = default;

  /** Returns the name of the robot */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

  /** Retrieve the associated RobotModule */
  inline const RobotModule & module() const noexcept
  {
    return module_;
  }

  /** Retrieve the joint limits */
  inline const Limits & limits() const noexcept
  {
    return limits_;
  }

  /** Retrieve the joint limits */
  inline Limits & limits() noexcept
  {
    return limits_;
  }

  /** Returns true if the robot has the given frame */
  bool hasFrame(std::string_view frame) const;

  /** Access a frame
   *
   * \param frame Name of the frame to access
   *
   * \throws If the frame does not exist
   */
  const Frame & frame(std::string_view frame) const;

  /** Access a frame (non-const)
   *
   * \param frame Name of the frame to access
   *
   * \throws If the frame does not exist
   */
  Frame & frame(std::string_view frame);

  /** Create a new frame attached to this robot
   *
   * \param name Name of the frame
   *
   * \param body Parent body of the frame
   *
   * \param X_b_f1 Transformation from the parent body to the frame
   *
   * \returns The newly created frame
   *
   * \throws If a frame with this name already exists
   */
  Frame & makeFrame(std::string_view name, std::string_view body, sva::PTransformd X_b_f);

  /** Create a new frame attached to this robot
   *
   * \param name Name of the frame
   *
   * \param frame Parent frame of the frame
   *
   * \param X_p_f Transformation from the parent frame to the frame
   *
   * \returns The newly created frame
   *
   * \throws If a frame with this name already exists or the parent frame does not belong to this robot
   */
  Frame & makeFrame(std::string_view name, const Frame & parent, sva::PTransformd X_p_f);

  /** @name Body sensors
   *
   * These functions are related to force sensors
   *
   * @{
   */

  /** Return the first BodySensor in the robot
   *
   * If the robot does not have body sensors, it returns a default
   * (invalid) one
   *
   */
  BodySensor & bodySensor();

  /** Return the first BodySensor in the robot (const) */
  const BodySensor & bodySensor() const;

  /** Return true if the robot has a body sensor named name
   *
   * @param name Name of the body sensor
   *
   */
  bool hasBodySensor(std::string_view name) const;

  /** Return true if the specified body has a body sensor attached to it
   *
   * @param body Body to query
   *
   */
  bool frameHasBodySensor(std::string_view body) const;

  /** Return a specific BobySensor by name
   *
   * @param name Name of the sensor
   *
   * @throws If the sensor does not exist
   *
   */
  BodySensor & bodySensor(std::string_view name);

  /** Return a specific BodySensor by name (const) */
  const BodySensor & bodySensor(std::string_view name) const;

  /** Return a specific BodySensor by body name
   *
   * @param name Name of the body
   *
   * @throws If there is no sensor attached to the body
   *
   */
  BodySensor & frameBodySensor(std::string_view name);

  /** Return a specific BodySensor by body name (const) */
  const BodySensor & frameBodySensor(std::string_view name) const;

  /** Return all body sensors */
  BodySensorVector & bodySensors();

  /** Return all body sensors (const) */
  const BodySensorVector & bodySensors() const;

  /** @} */
  /* End of Body sensors group */

  /** Returns true if the robot has a joint named \p name */
  bool hasJoint(std::string_view name) const;

  /** Returns true if the robot has a body named \p name */
  bool hasBody(std::string_view name) const;

  /** Returns the joint index of joint named \name
   *
   * \throws If the joint does not exist within the robot.
   */
  unsigned int jointIndexByName(std::string_view name) const;

  /** Returns the joint index in the mbc of the joint with index jointIndex in
   * refJointOrder
   *
   * @note Joint indices can be -1 for joints present in refJointOrder but not
   * in the robot's mbc (such as filtered joints in some robot modules)
   *
   * @param jointIndex Joint index in refJointOrder
   *
   * @returns joint index in the mbc
   *
   * @throws If jointIndex >= refJointOrder.size()
   */
  int jointIndexInMBC(size_t jointIndex) const;

  /** Returns the body index of joint named \name
   *
   * \throws If the body does not exist within the robot.
   */
  unsigned int bodyIndexByName(std::string_view name) const;

  /** Access MultiBody representation of the robot */
  rbd::MultiBody & mb();
  /** Access MultiBody representation of the robot (const) */
  const rbd::MultiBody & mb() const;

  /** Access MultiBodyConfig of the robot's mb() */
  rbd::MultiBodyConfig & mbc();
  /** Access MultiBodyConfig of the robot's mb() (const) */
  const rbd::MultiBodyConfig & mbc() const;

  /** Access MultiBodyGraph that generated the robot's mb() */
  rbd::MultiBodyGraph & mbg();
  /** Access MultiBodyGraph that generated the robot's mb() (const) */
  const rbd::MultiBodyGraph & mbg() const;

  /** Vector of normal acceleration in body coordinates */
  const std::vector<sva::MotionVecd> & normalAccB() const;
  /** Vector of normal acceleration in body coordinates */
  std::vector<sva::MotionVecd> & normalAccB();

  /** Access q variable (const) */
  inline const tvm::VariablePtr & q() const noexcept
  {
    return q_;
  }
  /** Access q variable */
  inline tvm::VariablePtr & q() noexcept
  {
    return q_;
  }

  /** Access floating-base variable (const) */
  inline const tvm::VariablePtr & qFloatingBase() const noexcept
  {
    return q_fb_;
  }
  /** Access free-flyer variable */
  inline tvm::VariablePtr & qFloatingBase() noexcept
  {
    return q_fb_;
  }

  /** Access joints variable (const) */
  inline const tvm::VariablePtr & qJoints() const noexcept
  {
    return q_joints_;
  }
  /** Access joints variable */
  inline tvm::VariablePtr & qJoints() noexcept
  {
    return q_joints_;
  }

  /** Given a joint index creates a TVM variable corresponding to this variable
   *
   * \warning This returns a different object (but the same variable from TVM pov) each time this function is called
   *
   * \param jIdx Joint index
   *
   * \throws If the joint index is out of bounds or the joint is not actuated
   */
  tvm::VariablePtr qJoint(size_t jIdx);

  /** Given a joint name, returns the TVM variable it belongs to and the index of the joint in this variable
   *
   * \param jName Joint name
   *
   * \throws If the robot does not have such a joint or the joint is not actuated
   */
  inline tvm::VariablePtr qJoint(std::string_view jName)
  {
    return qJoint(jointIndexByName(jName));
  }

  /** Access mimics' variable map (const) */
  inline const mc_rtc::map<tvm::VariablePtr, mimic_variables_t> & mimics() const noexcept
  {
    return mimics_;
  }
  /** Access mimics' variable map */
  inline mc_rtc::map<tvm::VariablePtr, mimic_variables_t> & mimics() noexcept
  {
    return mimics_;
  }

  /** Access tau variable (const) */
  inline const tvm::VariablePtr & tau() const noexcept
  {
    return tau_;
  }
  /** Access tau variable */
  inline tvm::VariablePtr & tau()
  {
    return tau_;
  }

  /** Returns the CoM algorithm associated to this robot (const) */
  inline const CoM & com() const noexcept
  {
    return *com_;
  }

  /** Returns the CoM algorithm associated to this robot */
  inline CoM & com() noexcept
  {
    return *com_;
  }

  /** Returns the momentum algorithm associated with this robot (const) */
  inline const Momentum & momentumAlgo() const noexcept
  {
    return *momentum_;
  }

  /** Returns the momentum algorithm associated with this robot (const) */
  inline Momentum & momentumAlgo() noexcept
  {
    return *momentum_;
  }

  /** Returns the current robot's momentum */
  const sva::ForceVecd & momentum() const noexcept
  {
    return momentum_->momentum();
  }

  /** Returns the mass matrix */
  inline const Eigen::MatrixXd & H() const noexcept
  {
    return fd_.H();
  }

  /** Returns the non-linear dynamics component */
  inline const Eigen::VectorXd & C() const noexcept
  {
    return fd_.C();
  }

  /** Shortcut to access the position of a body */
  inline const sva::PTransformd & bodyPosW(std::string_view body) const
  {
    return mbc().bodyPosW[bodyIndexByName(body)];
  }

  /** Shortcut to query the position of b2 relative to b1 */
  inline sva::PTransformd X_b1_b2(std::string_view b1, std::string_view b2) const
  {
    return bodyPosW(b2) * bodyPosW(b1).inv();
  }

  /** Compute the gravity-free wrench in a given frame
   *
   * @note If the frame is indirectly attached to the sensor (i.e there are
   * joints in-between), then the kinematic transformation will be taken into
   * account but the effect of bodies in-between is not accounted for in the
   * returned wrench.
   *
   * @param frameName A frame of this robot
   *
   * @return Measured wrench in the frame
   *
   * @throws If no sensor is attached to this frame or the frame does not exist
   */
  sva::ForceVecd frameWrench(std::string_view frameName) const;

  /** Compute the cop in the given frame computed from gravity-free force measurements
   *
   * @param frameName A frame attached to a force sensor
   * @param min_pressure Minimum pressure in N (default 0.5N).
   *
   * @return Measured cop in surface frame
   *  - CoP if pressure >= min_pressure
   *  - Zero otherwise
   *
   * @throws If no sensor is attached to this frame or the frame does not exist
   */
  Eigen::Vector2d cop(std::string_view frameName, double min_pressure = 0.5) const;

  /** Compute the cop in inertial frame compute from gravity-free force measurements
   *
   * @param frameName A frame attached to a force sensor
   * @param min_pressure Minimum pressure in N (default 0.5N).
   *
   * @return Measured cop in inertial frame
   *  - CoP if pressure >= min_pressure
   *  - Zero otherwise
   *
   * @throws If no sensor is attached to this surface
   */
  Eigen::Vector3d copW(std::string_view frameName, double min_pressure = 0.5) const;

  /**
   * @brief Computes net total wrench from a list of sensors
   *
   * @param sensorNames Names of all sensors used to compute the net wrench
   *
   * @return Net total wrench (without gravity) in the inertial frame
   */
  sva::ForceVecd netWrench(const std::vector<std::string> & sensorNames) const;

  /**
   * @brief Actual ZMP computation from net total wrench and the ZMP plane
   *
   * @param netTotalWrench Total wrench for all links in contact
   * @param plane_p Arbitrary point on the ZMP plane
   * @param plane_n Normal to the ZMP plane (normalized)
   * @param minimalNetNormalForce[N] Mininal force above which the ZMP computation
   * is considered valid. Must be >0 (prevents a divide by zero).
   *
   * @return zmp expressed in the requested plane
   *
   * @throws To prevent dividing by zero, throws if the projected force is below minimalNetNormalForce newton.
   * This is highly unlikely to happen and would likely indicate indicate that you are computing a ZMP from
   * invalid forces (such as with the robot in the air).
   *
   * \anchor zmpDoc
   *
   * \see Eigen::Vector3d mc_rbdyn::zmp(const sva::ForceVecd & netTotalWrench, const Eigen::Vector3d & plane_p, const
   * Eigen::Vector3d & plane_n, double minimalNetNormalForce)
   */
  Eigen::Vector3d zmp(const sva::ForceVecd & netTotalWrench,
                      const Eigen::Vector3d & plane_p,
                      const Eigen::Vector3d & plane_n,
                      double minimalNetNormalForce = 1.) const;

  /**
   * @brief ZMP computation from net total wrench and a frame
   *
   * See \ref zmpDoc
   *
   * \see Eigen::Vector3d mc_rbdyn::zmp(const sva::ForceVecd & netTotalWrench, const sva::PTransformd & zmpFrame, double
   * minimalNetNormalForce) const;
   *
   * @param netTotalWrench
   * @param zmpFrame Frame used for ZMP computation. The convention here is
   * that the contact frame should have its z-axis pointing in the normal
   * direction of the contact towards the robot.
   *
   * @return ZMP expressed in the plane defined by the zmpFrame frame.
   */
  Eigen::Vector3d zmp(const sva::ForceVecd & netTotalWrench,
                      const sva::PTransformd & zmpFrame,
                      double minimalNetNormalForce = 1.) const;

  /** Computes the ZMP from sensor names and a plane
   *
   * See \ref zmpDoc
   *
   * @param sensorNames Names of all sensors attached to a link in contact with the environment
   */
  Eigen::Vector3d zmp(const std::vector<std::string> & sensorNames,
                      const Eigen::Vector3d & plane_p,
                      const Eigen::Vector3d & plane_n,
                      double minimalNetNormalForce = 1.) const;

  /**
   * @brief Computes the ZMP from sensor names and a frame
   *
   * See \ref zmpDoc
   *
   * @param sensorNames Names of all sensors attached to a link in contact with the environment
   */
  Eigen::Vector3d zmp(const std::vector<std::string> & sensorNames,
                      const sva::PTransformd & zmpFrame,
                      double minimalNetNormalForce = 1.) const;

  /** Return the flexibilities of the robot (const) */
  const std::vector<Flexibility> & flexibility() const;
  /** Return the flexibilities of the robot */
  std::vector<Flexibility> & flexibility();

  /** Returns the mass of the robot */
  inline double mass() const noexcept
  {
    return mass_;
  }

  /** @name Joint sensors
   *
   * These functions give information about joints' status
   *
   * @{
   */

  /** Return the encoder values */
  const std::vector<double> & encoderValues() const;

  /** Set the encoder values */
  void encoderValues(const std::vector<double> & encoderValues);

  /** Return the encoder velocities */
  const std::vector<double> & encoderVelocities() const;

  /** Set the encoder velocities */
  void encoderVelocities(const std::vector<double> & encoderVelocities);

  /** Return the flexibilities values */
  const std::vector<double> & flexibilityValues() const;

  /** Set the flexibilities values */
  void flexibilityValues(const std::vector<double> & flexibilityValues);

  /** Return the joint torques from sensors */
  const std::vector<double> & jointTorques() const;

  /** Set joint torques from sensors */
  void jointTorques(const std::vector<double> & jointTorques);

  /** @} */
  /* End Joints sensors section */

  /** @name Force sensors
   *
   * These functions are related to force sensors
   *
   * @{
   */

  /** Check if a force sensor exists
   *
   * @param name Name of the sensor
   *
   * @returns True if the sensor exists, false otherwise
   */
  bool hasForceSensor(std::string_view name) const;

  /** Check if the frame has a force sensor attached to it
   *
   * @param frame Name of the frame
   *
   * @returns True if the frame has a force sensor attached to it, false
   * otherwise
   */
  bool frameHasForceSensor(std::string_view frame) const;

  /** Check if the frame has a force sensor indirectly attached to it
   *
   * \param frame Name of the frame
   *
   * \returns True if the frame has a force sensor indirectly attached to it, false otherwise
   */
  bool frameHasIndirectForceSensor(std::string_view frame) const;

  /** Return a force sensor by name
   *
   * @param name Name of the sensor
   *
   * @return The sensor named name
   *
   * @throws If no sensor with this name exists
   *
   */
  ForceSensor & forceSensor(std::string_view name);

  /** Const variant */
  const ForceSensor & forceSensor(std::string_view name) const;

  /** Return a force sensor attached to the provided frame
   *
   * @param frame Name of the frame to which the sensor is attached
   *
   * @return The attached sensor
   *
   * @throws If no sensor is directly attached to this frame's parent body
   *
   * @note if a sensor in indirectly attached to a frame, use findFrameForceSensor() instead
   */
  ForceSensor & frameForceSensor(std::string_view frame);

  /** Const variant */
  const ForceSensor & frameForceSensor(std::string_view frame) const;

  /**
   * @brief Looks for a force sensor up the kinematic chain from the frame to the root.
   *
   * @param frame Name of the frame
   *
   * @return The sensor to which the frame is indirectly attached
   *
   * @throws If no sensor is found between the frame and the root
   */
  ForceSensor & findFrameForceSensor(std::string_view frame);

  /** Const variant */
  const ForceSensor & findFrameForceSensor(std::string_view frame) const;

  /** Returns all force sensors */
  std::vector<ForceSensor> & forceSensors();

  /** Returns all force sensors (const) */
  const std::vector<ForceSensor> & forceSensors() const;

  /** @} */
  /* End of Force sensors group */

  /** @name Devices
   *
   * These functions are related to generic devices handling
   *
   * @{
   */

  /** Returns true if a generic device of type T and named name exists in the robot
   *
   * \param name Name of the device
   *
   * \tparam T Type of device requested
   *
   */
  template<typename T>
  bool hasDevice(std::string_view name) const;

  /** Alias for \see hasDevice */
  template<typename T>
  inline bool hasSensor(std::string_view name) const
  {
    return hasDevice<T>(name);
  }

  /** Get a generic device of type T named name
   *
   * The reference returned by this function is remains valid
   *
   * \param name Name of the device
   *
   * \tparam T type of the device requested
   *
   * \throws If the device does not exist or does not have the right type
   *
   */
  template<typename T>
  const T & device(std::string_view name) const;

  /** Non-const variant */
  template<typename T>
  T & device(std::string_view name)
  {
    return const_cast<T &>(const_cast<const Robot *>(this)->device<T>(name));
  }

  /** Alias for \see device */
  template<typename T>
  inline const T & sensor(std::string_view name) const
  {
    return device<T>(name);
  }

  /** Alias for \see device */
  template<typename T>
  inline T & sensor(std::string_view name)
  {
    return device<T>(name);
  }

  /** Add a generic device to the robot */
  void addDevice(DevicePtr device);

  /** Alias for \see addDevice */
  inline void addSensor(SensorPtr sensor)
  {
    addDevice(std::move(sensor));
  }

  /** @} */
  /* End of Devices group */

  /** Check if a surface \p surface exists
   *
   * \returns True if the surface exists, false otherwise
   */
  bool hasSurface(std::string_view surface) const;

  /** Access a surface by its name \p sName */
  Surface & surface(std::string_view sName);
  /** Access a surface by its name \p sName (const) */
  const Surface & surface(std::string_view sName) const;

  /** Adds a surface */
  void addSurface(SurfacePtr surface, bool overwrite = false);

  /** Returns all available surfaces */
  const mc_rtc::map<std::string, SurfacePtr> & surfaces() const;

  /** Returns a list of available surfaces */
  std::vector<std::string> availableSurfaces() const;

  /** Check if a convex \p name exists
   *
   * \returns True if the convex exists, false otherwise
   */
  bool hasConvex(std::string_view name) const;

  /** Access a convex named \p cName
   *
   * \returns a pair giving the convex's parent body and the sch::Object
   * object
   */
  Convex & convex(std::string_view cName);
  /** Access a convex named \p cName (const) */
  const Convex & convex(std::string_view cName) const;

  /** Access all convexes available in this robot
   *
   * \returns a map where keys are the convex name and values are those returned by \ref convex
   */
  inline const mc_rtc::map<std::string, ConvexPtr> & convexes() const noexcept
  {
    return convexes_;
  }

  /** Add a convex online
   *
   * This has no effect if \p name is already a convex of the robot
   *
   * \param name Name of the convex
   *
   * \param object Object used for collision detection
   *
   * \param parent Parent frame of the convex
   *
   * \param X_f_c Transformation between the convex and its frame
   *
   * \throws If such a convex already exists within the robot
   *
   */
  Convex & addConvex(std::string_view name,
                     S_ObjectPtr object,
                     std::string_view parent,
                     sva::PTransformd X_f_c = sva::PTransformd::Identity());

  /** Remove a given convex
   *
   * Using this function while the given convex is involved in a collision is
   * *not* safe and will very likely result in a crash.
   *
   * This has no effect if \p name is not a convex of the robot.
   *
   * \param name Name of the convex
   *
   */
  void removeConvex(std::string_view name);

  /** Access transformation from body \p bName to original base.
   *
   * This can be used to correct transformations that were stored with the
   * original base. Usually the robot's base is the original base so these
   * transforms are identity.
   */
  const sva::PTransformd & bodyTransform(std::string_view bName) const;

  /** Access body transform by index */
  const sva::PTransformd & bodyTransform(int bodyIndex) const;

  /** Access body transform vector */
  const std::vector<sva::PTransformd> & bodyTransforms() const;

  /** Load surfaces from the directory \p surfaceDir */
  void loadRSDFFromDir(std::string_view surfaceDir);

  /** Return the robot's default stance (e.g. half-sitting for humanoid) */
  inline const mc_rtc::map<std::string, std::vector<double>> & stance() const noexcept
  {
    return module().stance();
  }

  /** Returns the robot's default joint order */
  inline const std::vector<std::string> & refJointOrder() const noexcept
  {
    return module().ref_joint_order();
  }

  /** Apply Euler integration to the robot using \p step timestep */
  void eulerIntegration(double step);

  /** Return the robot's global pose */
  const sva::PTransformd & posW() const;

  /** Set the robot's global pose.
   * This is mostly meant for initialization purposes.
   * In other scenarios there might be more things to do
   * to properly move a robot (e.g. update contacts, set speed to zero).
   *
   * @param pt The new global pose
   *
   * @throws If joint(0) is neither free flyer nor fixed
   *
   * @note This function takes care of calling rbd::forwardKinematics
   */
  void posW(const sva::PTransformd & pt);

  /** Update the robot's floating base velocity.
   *
   * \param vel New floating base velocity in the inertial frame.
   *
   * @note This function takes care of calling rbd::forwardVelocity
   */
  void velW(const sva::MotionVecd & vel);

  /** Return the robot's floating base velocity expressed in the inertial frame */
  const sva::MotionVecd & velW() const;

  /** Update the robot's floating base acceleration.
   *
   * \param vel New floating base acceleration in the inertial frame.
   *
   * @note This function takes care of calling rbd::forwardAcceleration
   */
  void accW(const sva::MotionVecd & acc);

  /** Return the robot's floating base acceleration expressed in the inertial
   * frame
   */
  const sva::MotionVecd accW() const;

  /** Access a gripper by name
   *
   * \param gripper Gripper name
   *
   * \throws If the gripper does not exist within this robot
   */
  mc_control::Gripper & gripper(std::string_view gripper);

  /** Checks whether a gripper is part of this robot */
  bool hasGripper(const std::string & gripper) const;

  inline const mc_rtc::map<std::string, mc_control::GripperPtr> & grippersByName() const
  {
    return grippers_;
  }

  /** Access all grippers */
  inline const std::vector<mc_control::GripperRef> & grippers() const
  {
    return grippersRef_;
  }

  /** Update the robot's kinematic state */
  void forwardKinematics();

  /** Update the robot's velocity state */
  void forwardVelocity();

  /** Shortcut for forward acceleration */
  void forwardAcceleration();

private:
  /** Name of the robot */
  std::string name_;
  /** RobotModule that was used to create this robot */
  RobotModule module_;
  /** Joint limits */
  Limits limits_;
  /** Floating-base variable */
  tvm::VariablePtr q_fb_;
  /** Joints variable */
  tvm::VariablePtr q_joints_;
  /** Generalized configuration variable */
  tvm::VariablePtr q_;
  /** Map mimic leader joint to their followers */
  mc_rtc::map<tvm::VariablePtr, mimic_variables_t> mimics_;
  /** Derivative of q */
  tvm::VariablePtr dq_;
  /** Double derivative of q */
  tvm::VariablePtr ddq_;
  /** Tau variable */
  tvm::VariablePtr tau_;
  /** Robot's mass */
  double mass_;
  /** Normal accelerations of the bodies */
  std::vector<sva::MotionVecd> normalAccB_;
  /** Forward dynamics algorithm associated to this robot */
  rbd::ForwardDynamics fd_;
  /** CoM algorithm of this robot */
  CoMPtr com_;
  /** Momentum algorithm of this robot */
  MomentumPtr momentum_;
  /** List of body transformations */
  std::vector<sva::PTransformd> bodyTransforms_;
  /** List of frames available in this robot */
  mc_rtc::map<std::string, FramePtr> frames_;
  /** List of convex available in this robot */
  mc_rtc::map<std::string, ConvexPtr> convexes_;
  /** List of surfaces available in this robot */
  mc_rtc::map<std::string, SurfacePtr> surfaces_;
  /** Correspondance between refJointOrder (actuated joints) index and
   * mbc index. **/
  std::vector<int> refJointIndexToMBCIndex_;
  /** Encoder values provided by the low-level controller */
  std::vector<double> encoderValues_;
  /** Encoder velocities provided by the low-level controller or estimated from
   * encoder values **/
  std::vector<double> encoderVelocities_;
  /** Joint torques provided by the low-level controller */
  std::vector<double> jointTorques_;
  /** Hold all body sensors */
  BodySensorVector bodySensors_;
  /** Correspondance between body sensor's name and body sensor index*/
  mc_rtc::map<std::string, size_t> bodySensorsIndex_;
  /** Correspondance between frames' names and attached body sensors */
  mc_rtc::map<std::string, size_t> frameBodySensors_;
  /** List of springs in a Robot */
  Springs springs_;
  /** List of flexibility in a Robot */
  std::vector<Flexibility> flexibility_;
  /** Flexibility estimation from an estimator */
  std::vector<double> flexibilityValues_;
  /** List of force sensors attached to the robot */
  std::vector<ForceSensor> forceSensors_;
  /** Correspondance between force sensor's name and force sensor index */
  mc_rtc::map<std::string, size_t> forceSensorsIndex_;
  /** Correspondance between frames' names and attached force sensors */
  mc_rtc::map<std::string, size_t> frameForceSensors_;
  /** Correspondance between frames' names and indirectly attached force sensors */
  mc_rtc::map<std::string, size_t> frameIndirectForceSensors_;
  /** Grippers attached to this robot */
  mc_rtc::map<std::string, mc_control::GripperPtr> grippers_;
  /** Grippers reference for this robot */
  std::vector<mc_control::GripperRef> grippersRef_;
  /** Hold all devices that are neither force sensors nor body sensors */
  DevicePtrVector devices_;
  /** Correspondance between a device's name and a device index */
  mc_rtc::map<std::string, size_t> devicesIndex_;

protected:
  /** Copy existing Robot with a new base */
  RobotPtr copy(std::string_view name, const std::optional<Base> & base = std::nullopt) const;

private:
  Robot(const Robot &) = delete;
  Robot & operator=(const Robot &) = delete;

  void updateFK();
  void updateFV();
  void updateFA();
  void updateNormalAcceleration();
  void updateH();
  void updateC();

  void updateAll();

  /** Used internally to access a frame safely and provide context information */
  Frame & frame(std::string_view, std::string_view context);

  /** Used internally to access a frame safely and provide context information */
  const Frame & frame(std::string_view, std::string_view context) const;

  /** Used internally to update frame to force sensor mapping */
  Frame & updateFrameForceSensors(Frame & frame);

  /** List of inputs for the kinematics graph */
  std::shared_ptr<tvm::graph::internal::Inputs> kinematicsInputs_;

  /** Used internally to maintain a kinematic view of the robot up-to-date */
  tvm::graph::CallGraph kinematicsGraph_;
};

} // namespace mc_rbdyn

#include <mc_rbdyn/Robot.hpp>
