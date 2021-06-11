/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/ZMP.h>
#include <mc_rbdyn/surface_utils.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/pragma.h>

#include <RBDyn/CoM.h>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Superellipsoid.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <fstream>
#include <numeric>
#include <tuple>

namespace
{

bool VisualToConvex(mc_rbdyn::Robot & robot,
                    std::string_view cName,
                    std::string_view bName,
                    const rbd::parsers::Visual & visual)
{
  // Ignore visual types that we cannot easily map to SCH
  if(visual.geometry.type == rbd::parsers::Geometry::Type::UNKNOWN
     || visual.geometry.type == rbd::parsers::Geometry::Type::MESH)
  {
    return false;
  }
  // If we already have a convex with the same name, discard loading
  if(robot.hasConvex(cName))
  {
    mc_rtc::log::warning("While loading {}, a convex was already provided for collision geometry specified in URDF",
                         robot.name());
    return false;
  }
  auto fromBox = [&]() {
    const auto & box = boost::get<rbd::parsers::Geometry::Box>(visual.geometry.data);
    robot.addConvex(cName, std::make_shared<sch::S_Box>(box.size.x(), box.size.y(), box.size.z()), bName,
                    visual.origin);
  };
  auto fromCylinder = [&]() {
    const auto & cyl = boost::get<rbd::parsers::Geometry::Cylinder>(visual.geometry.data);
    robot.addConvex(cName,
                    std::make_shared<sch::S_Cylinder>(sch::Point3(0, 0, -cyl.length / 2),
                                                      sch::Point3(0, 0, cyl.length / 2), cyl.radius),
                    bName, visual.origin);
  };
  auto fromSphere = [&]() {
    const auto & sph = boost::get<rbd::parsers::Geometry::Sphere>(visual.geometry.data);
    robot.addConvex(cName, std::make_shared<sch::S_Sphere>(sph.radius), bName, visual.origin);
  };
  auto fromSuperEllipsoid = [&]() {
    const auto & sel = boost::get<rbd::parsers::Geometry::Superellipsoid>(visual.geometry.data);
    robot.addConvex(
        cName,
        std::make_shared<sch::S_Superellipsoid>(sel.size.x(), sel.size.y(), sel.size.z(), sel.epsilon1, sel.epsilon2),
        bName, visual.origin);
  };
  switch(visual.geometry.type)
  {
    case rbd::parsers::Geometry::Type::BOX:
      fromBox();
      break;
    case rbd::parsers::Geometry::Type::CYLINDER:
      fromCylinder();
      break;
    case rbd::parsers::Geometry::Type::SPHERE:
      fromSphere();
      break;
    case rbd::parsers::Geometry::Type::SUPERELLIPSOID:
      fromSuperEllipsoid();
      break;
    default:
      return false;
  }
  return true;
}

} // namespace

namespace mc_rbdyn
{

// We can safely ignore those since they are due to different index types and
// our index never go near unsafe territories
MC_RTC_diagnostic_push
MC_RTC_diagnostic_ignored(GCC, "-Wsign-conversion", ClangOnly, "-Wshorten-64-to-32")

Robot::Robot(make_shared_token,
             RobotModule moduleIn,
             std::string_view name,
             bool loadFiles,
             const std::optional<sva::PTransformd> & base,
             const std::optional<std::string_view> & bName)
: name_(name), module_(std::move(moduleIn)), normalAccB_(module_.mbc.bodyAccB.size()), fd_(module_.mb)
{
  if(base)
  {
    std::string baseName = bName ? std::string(bName.value()) : mb().body(0).name();
    module_.mb = mbg().makeMultiBody(baseName, mb().joint(0).type() == rbd::Joint::Fixed, *base);
    module_.mbc = rbd::MultiBodyConfig(mb());
  }

  using jt_method = int (rbd::Joint::*)() const;
  auto initBound = [&](const std::string & name, Eigen::VectorXd & bound, int size, jt_method getSize,
                       const RobotModule::bound_t & map, double defaultJoint, double defaultFB) {
    bound.resize(size);
    Eigen::DenseIndex i = 0;
    for(const auto & j : mb().joints())
    {
      auto jSize = (j.*getSize)();
      const auto & jName = j.name();
      auto it = map.find(jName);
      if(it == map.end())
      {
        const auto & def = j.type() == rbd::Joint::Free ? defaultFB : defaultJoint;
        bound.segment(i, jSize).setConstant(def);
      }
      else
      {
        const auto & value = it->second;
        if(value.size() != static_cast<size_t>(jSize))
        {
          mc_rtc::log::error_and_throw<std::runtime_error>(
              "{} provided bound size ({}) different from expected size ({}) for joint {}", name, value.size(), jSize,
              jName);
        }
        for(size_t k = 0; k < static_cast<size_t>(jSize); ++k)
        {
          bound(i + k) = value[k];
        }
      }
      i += jSize;
    }
  };

  auto inf = std::numeric_limits<double>::infinity();
  if(module_.bounds().size() != 6)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>(
        "The bounds of robotmodule \"{}\" have a size of {} instead of 6 (ql, qu, vl, vu, tl, tu).", module_.name,
        module_.bounds().size());
  }

  initBound("lower position", limits_.ql, mb().nrParams(), &rbd::Joint::params, module_.bounds()[0], -inf, -inf);
  initBound("upper position", limits_.qu, mb().nrParams(), &rbd::Joint::params, module_.bounds()[1], inf, inf);
  initBound("lower velocity", limits_.vl, mb().nrDof(), &rbd::Joint::dof, module_.bounds()[2], -inf, -inf);
  initBound("upper velocity", limits_.vu, mb().nrDof(), &rbd::Joint::dof, module_.bounds()[3], inf, inf);
  initBound("lower torque", limits_.tl, mb().nrDof(), &rbd::Joint::dof, module_.bounds()[4], -inf, 0);
  initBound("upper torque", limits_.tu, mb().nrDof(), &rbd::Joint::dof, module_.bounds()[5], inf, 0);

  auto initOtherBound = [&](std::string_view name, const auto & bounds, Eigen::VectorXd & lower,
                            Eigen::VectorXd & upper) {
    if(bounds.size() == 0)
    {
      lower = Eigen::VectorXd::Constant(mb().nrDof(), 1, -inf);
      upper = Eigen::VectorXd::Constant(mb().nrDof(), 1, inf);
    }
    else if(bounds.size() == 2)
    {
      initBound(fmt::format("lower {}", name), lower, mb().nrDof(), &rbd::Joint::dof, bounds[0], -inf, -inf);
      initBound(fmt::format("upper {}", name), upper, mb().nrDof(), &rbd::Joint::dof, bounds[1], inf, inf);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("The {} bounds of RobotModule \"{}\" have a size of {} "
                                                          "instead of 2 ([lower, upper]).",
                                                          name, module_.name, module_.accelerationBounds().size());
    }
  };
  initOtherBound("acceleration", module_.accelerationBounds(), limits_.al, limits_.au);
  initOtherBound("torque derivative", module_.torqueDerivativeBounds(), limits_.tdl, limits_.tdu);

  module_.mbc.gravity = mc_rtc::constants::gravity;
  module_.mbc.zero(mb());
  {
    auto initQ = mbc().q;
    const auto & stance = module_.stance();
    for(size_t i = 0; i < mb().joints().size(); ++i)
    {
      const auto & j = mb().joint(static_cast<int>(i));
      if(stance.count(j.name()))
      {
        const auto & jQ = stance.at(j.name());
        if(initQ[i].size() != jQ.size())
        {
          mc_rtc::log::error_and_throw<std::runtime_error>(
              "Missmatch between RobotModule stance for joint {}\nStance provides {} values but should be {}", j.name(),
              jQ.size(), initQ[i].size());
        }
        initQ[i] = jQ;
      }
    }
    if(initQ[0].size())
    {
      const auto & attitude = module_.default_attitude();
      initQ[0] = {std::begin(attitude), std::end(attitude)};
    }
    module_.mbc.q = initQ;
    rbd::forwardKinematics(mb(), module_.mbc);
  }

  forceSensors_ = module_.forceSensors();
  for(auto & fs : forceSensors_)
  {
    bfs::path calib_file = bfs::path(module_.calib_dir) / std::string("calib_data." + fs.name());
    fs.loadCalibrator(calib_file.string(), mbc().gravity);
  }
  for(size_t i = 0; i < forceSensors_.size(); ++i)
  {
    const auto & fs = forceSensors_[i];
    forceSensorsIndex_[fs.name()] = i;
    frameForceSensors_[fs.parentBody()] = i;
  }

  bodyTransforms_.resize(mb().bodies().size());
  const auto & bbts = base ? module_.mbg.bodiesBaseTransform(mb().body(0).name(), *base)
                           : module_.mbg.bodiesBaseTransform(mb().body(0).name());
  for(size_t i = 0; i < mb().bodies().size(); ++i)
  {
    const auto & b = mb().body(static_cast<int>(i));
    bodyTransforms_[i] = bbts.at(b.name());
    makeFrame(b.name());
  }

  if(loadFiles)
  {
    const auto & cTransforms = module_.collisionTransforms();
    for(const auto & ch : module_.convexHull())
    {
      const auto & cName = ch.first;
      const auto & parent = ch.second.first;
      const auto & cURI = ch.second.second;
      if(!hasBody(parent))
      {
        mc_rtc::log::warning("Cannot load convex {} for {} since the parent body ({}) does not exist", cName,
                             this->name(), parent);
        continue;
      }
      if(!bfs::exists(cURI))
      {
        mc_rtc::log::warning("Cannot load convex {} for {} as the associated file ({}) does not exist", cName,
                             this->name(), cURI);
        continue;
      }
      auto transform_it = cTransforms.find(cName);
      const auto & cTransform = transform_it == cTransforms.end() ? sva::PTransformd::Identity() : transform_it->second;
      this->addConvex(cName, sch::mc_rbdyn::make_polyhedron(cURI), parent, cTransform);
    }
  }
  for(const auto & c : module_._collision)
  {
    const auto & body = c.first;
    const auto & collisions = c.second;
    if(collisions.size() == 1)
    {
      VisualToConvex(*this, body, body, collisions[0]);
      continue;
    }
    size_t added = 0;
    for(const auto & col : collisions)
    {
      if(VisualToConvex(*this, body + "_" + std::to_string(added), body, col))
      {
        added++;
      }
    }
  }

  if(loadFiles)
  {
    if(bfs::exists(module_.rsdf_dir))
    {
      loadRSDFFromDir(module_.rsdf_dir);
    }
    else if(module_.rsdf_dir.size())
    {
      mc_rtc::log::warning("RSDF directory ({}) specified by RobotModule for {} does not exist.", module_.rsdf_dir,
                           module_.name);
    }
  }

  // For each body in the robot find the closest force sensor attached to it (if any)
  for(const auto & body : mb().bodies())
  {
    if(frameForceSensors_.contains(body.name()))
    {
      continue;
    }
    int parentIndex = mb().parent(mb().bodyIndexByName().at(body.name()));
    while(parentIndex >= 0)
    {
      const auto & parent = mb().body(parentIndex);
      if(frameForceSensors_.contains(parent.name()))
      {
        frameIndirectForceSensors_[body.name()] = frameForceSensors_.at(parent.name());
        break;
      }
      parentIndex = mb().parent(parentIndex);
    }
  }

  bodySensors_ = module_.bodySensors();
  // Add a single default sensor if no sensor on the robot
  if(bodySensors_.size() == 0)
  {
    bodySensors_.emplace_back();
  }
  for(size_t i = 0; i < bodySensors_.size(); ++i)
  {
    const auto & bS = bodySensors_[i];
    bodySensorsIndex_[bS.name()] = i;
    frameBodySensors_[bS.parentBody()] = i;
  }

  devices_ = module_.devices();
  for(size_t i = 0; i < devices_.size(); ++i)
  {
    auto & d = devices_[i];
    if(d->parent() == "")
    {
      d->parent(mb().body(0).name());
    }
    devicesIndex_[d->name()] = i;
  }

  const auto & refJointOrder_ = module_.ref_joint_order();
  refJointIndexToQIndex_.resize(refJointOrder_.size());
  refJointIndexToQDotIndex_.resize(refJointOrder_.size());
  for(size_t i = 0; i < refJointOrder_.size(); ++i)
  {
    const auto & jN = refJointOrder_[i];
    if(hasJoint(jN))
    {
      auto jIndex = mb().jointIndexByName(jN);
      refJointIndexToQIndex_[i] = mb().joint(jIndex).dof() != 0 ? mb().jointPosInParam(jIndex) : -1;
      refJointIndexToQDotIndex_[i] = mb().joint(jIndex).dof() != 0 ? mb().jointPosInDof(jIndex) : -1;
    }
    else
    {
      refJointIndexToQIndex_[i] = -1;
      refJointIndexToQDotIndex_[i] = -1;
    }
  }

  springs_ = module_.springs();
  flexibility_ = module_.flexibility();

  std::string urdf;
  auto loadUrdf = [this, &urdf]() -> const std::string & {
    if(urdf.size())
    {
      return urdf;
    }
    const auto & urdfPath = module_.urdf_path;
    std::ifstream ifs(urdfPath);
    if(ifs.is_open())
    {
      std::stringstream urdfSS;
      urdfSS << ifs.rdbuf();
      urdf = urdfSS.str();
      return urdf;
    }
    mc_rtc::log::error("Could not open urdf file {} for robot {}, cannot initialize grippers", urdfPath, module_.name);
    mc_rtc::log::error_and_throw<std::runtime_error>("Failed to initialize grippers");
  };
  for(const auto & gripper : module_.grippers())
  {
    auto mimics = gripper.mimics();
    auto safety = gripper.safety();
    if(mimics)
    {
      grippers_[gripper.name].reset(new mc_control::Gripper(*this, gripper.joints, *mimics, gripper.reverse_limits,
                                                            safety ? *safety : module_.gripperSafety()));
    }
    else
    {
      grippers_[gripper.name].reset(new mc_control::Gripper(*this, gripper.joints, loadUrdf(), gripper.reverse_limits,
                                                            safety ? *safety : module_.gripperSafety()));
    }
  }
  for(auto & g : grippers_)
  {
    grippersRef_.push_back(std::ref(*g.second));
  }

  mass_ = std::accumulate(mb().bodies().begin(), mb().bodies().end(), 0.0,
                          [](double m, const auto & body) { return m + body.inertia().mass(); });
  com_ = std::make_shared<CoM>(CoM::ctor_token{}, *this);

  momentum_ = std::make_shared<Momentum>(Momentum::ctor_token{}, *com_);

  // Create TVM variables
  {
    q_ = tvm::Space(mb().nrDof(), mb().nrParams(), mb().nrDof()).createVariable(name_);
    if(mb().nrJoints() > 0 && mb().joint(0).type() == rbd::Joint::Free)
    {
      q_fb_ = q_->subvariable(tvm::Space(6, 7, 6), "qFloatingBase");
      q_joints_ = q_->subvariable(tvm::Space(mb().nrDof() - 6, mb().nrParams() - 7, mb().nrDof() - 6), "qJoints",
                                  tvm::Space(6, 7, 6));
    }
    else
    {
      q_fb_ = q_->subvariable(tvm::Space(0), "qFloatingBase");
      q_joints_ = q_;
    }
    Eigen::VectorXd mimicMultiplier = Eigen::VectorXd(0);
    mc_rtc::map<size_t, tvm::VariablePtr> mimicLeaders;
    mc_rtc::map<size_t, mimic_variables_t> mimicFollowers;
    const auto & joints = mb().joints();
    // Returns true if the provided joint is a leader in a mimic relationship
    auto isMimicLeader = [&](std::string_view jName) {
      return std::find_if(mb().joints().begin(), mb().joints().end(),
                          [&](const auto & j) { return j.isMimic() && j.mimicName() == jName; })
             != mb().joints().end();
    };
    // Returns a mimic block, i.e. following a given mimic joint all joints that follow that are also mimic of the same
    // joint
    auto getMimicBlock = [&](size_t startIdx) -> std::tuple<int, int, size_t> {
      const auto & mimicJ = joints[startIdx];
      size_t endIdx = startIdx;
      int params = mimicJ.params();
      int dof = mimicJ.dof();
      for(size_t i = startIdx + 1; i < joints.size(); ++i)
      {
        const auto & j = joints[i];
        if(j.isMimic() && j.mimicName() == mimicJ.mimicName())
        {
          endIdx = i;
          params += j.params();
          dof += j.dof();
        }
        else
        {
          break;
        }
      }
      return {params, dof, endIdx};
    };
    int nParams = 0;
    int nDof = 0;
    for(size_t i = 0; i < joints.size(); ++i)
    {
      const auto & j = joints[i];
      if(isMimicLeader(j.name()))
      {
        tvm::VariablePtr var = qJoint(i);
        mimicLeaders[i] = var;
      }
      else if(j.isMimic())
      {
        auto [params, dof, endIdx] = getMimicBlock(i);
        auto leaderIdx = jointIndexByName(j.mimicName());
        tvm::VariablePtr var =
            q_->subvariable(tvm::Space(dof, params), fmt::format("{}...{}", j.name(), joints[endIdx].name()),
                            tvm::Space(nDof, nParams));
        if(!mimicFollowers.count(leaderIdx))
        {
          mimicFollowers[leaderIdx].second.resize(0);
        }
        mimicFollowers[leaderIdx].first.add(var);
        auto & mult = mimicFollowers[leaderIdx].second;
        auto newM = Eigen::VectorXd(mult.size() + var->size());
        newM.head(mult.size()) = mult;
        for(size_t ii = i; ii <= endIdx; ++ii)
        {
          newM(mult.size() + ii - i) = joints[ii].mimicMultiplier();
        }
        mult = newM;
        nParams += params;
        nDof += dof;
        i = endIdx;
        continue;
      }
      nParams += j.params();
      nDof += j.dof();
    }
    for(auto & m : mimicLeaders)
    {
      mimics_[m.second] = mimicFollowers[m.first];
    }
  }
  tau_ = tvm::Space(mb().nrDof()).createVariable(name_ + "_tau");
  dq_ = tvm::dot(q_, 1);
  ddq_ = tvm::dot(q_, 2);
  Eigen::VectorXd q_init = q_->value();
  rbd::paramToVector(mbc().q, q_init);
  q_->set(q_init);
  dq_->setZero();
  ddq_->setZero();
  tau_->setZero();

  /** Signal setup */
  registerUpdates(Update::FK, &Robot::updateFK, Update::FV, &Robot::updateFV, Update::FA, &Robot::updateFA,
                  Update::NormalAcceleration, &Robot::updateNormalAcceleration, Update::H, &Robot::updateH, Update::C,
                  &Robot::updateC);
  /** Output dependencies setup */
  addOutputDependency(Output::FK, Update::FK);
  addOutputDependency(Output::FV, Update::FV);
  addOutputDependency(Output::FA, Update::FA);
  addOutputDependency(Output::NormalAcceleration, Update::NormalAcceleration);
  addOutputDependency(Output::H, Update::H);
  addOutputDependency(Output::C, Update::C);
  addOutputDependency(Output::FV, Update::FV);
  /** Internal dependencies setup */
  addInternalDependency(Update::FV, Update::FK);
  addInternalDependency(Update::H, Update::FV);
  addInternalDependency(Update::C, Update::FV);
  addInternalDependency(Update::FA, Update::FV);
  addInternalDependency(Update::NormalAcceleration, Update::FV);

  updateAll();
}

bool Robot::hasFrame(std::string_view frame) const
{
  return frames_.contains(frame);
}

const RobotFrame & Robot::frame(std::string_view frame) const
{
  return this->frame(frame, "Robot::frame");
}

RobotFrame & Robot::frame(std::string_view frame)
{
  return this->frame(frame, "Robot::frame");
}

RobotFrame & Robot::makeFrame(std::string_view body)
{
  if(hasFrame(body))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Attempt to duplicate body frame {} in {}", body, name());
  }
  auto out = frames_.emplace(body, std::make_shared<RobotFrame>(RobotFrame::ctor_token{}, body, *this, body));
  return updateFrameForceSensors(*out.first->second);
}

RobotFrame & Robot::makeFrame(std::string_view name, std::string_view body, sva::PTransformd X_b_f)
{
  return makeFrame(name, frame(body), X_b_f);
}

RobotFrame & Robot::makeFrame(std::string_view name, RobotFrame & parent, sva::PTransformd X_p_f, bool baked)
{
  if(hasFrame(name))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("A frame named {} already exists in {}", name, this->name());
  }
  if(&parent.robot_ != this)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Parent frame {} provided to build frame {} in {} belong to a different robot {}", parent.name(), name,
        this->name(), parent.robot().name());
  }
  if(baked)
  {
    return makeFrame(name, parent.body(), X_p_f * parent.X_b_f());
  }
  auto out =
      frames_.emplace(name, std::make_shared<RobotFrame>(RobotFrame::ctor_token{}, name, parent, std::move(X_p_f)));
  return updateFrameForceSensors(*out.first->second);
}

BodySensor & Robot::bodySensor()
{
  return bodySensors_[0];
}

const BodySensor & Robot::bodySensor() const
{
  return bodySensors_[0];
}

bool Robot::hasBodySensor(std::string_view name) const
{
  return bodySensorsIndex_.contains(name);
}

bool Robot::frameHasBodySensor(std::string_view frame) const
{
  return frameBodySensors_.contains(frame);
}

BodySensor & Robot::bodySensor(std::string_view name)
{
  return const_cast<BodySensor &>(static_cast<const Robot *>(this)->bodySensor(name));
}

const BodySensor & Robot::bodySensor(std::string_view name) const
{
  auto sensorIdx = bodySensorsIndex_.find(name);
  if(sensorIdx == bodySensorsIndex_.cend())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No body sensor named {} in {}", name, this->name());
  }
  return bodySensors_[sensorIdx->second];
}

BodySensor & Robot::frameBodySensor(std::string_view name)
{
  return const_cast<BodySensor &>(static_cast<const Robot *>(this)->frameBodySensor(name));
}

const BodySensor & Robot::frameBodySensor(std::string_view name) const
{
  auto sensorIdx = frameBodySensors_.find(name);
  if(sensorIdx == bodySensorsIndex_.cend())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No body sensor attached to {} in {}", name, this->name());
  }
  return bodySensors_[sensorIdx->second];
}

BodySensorVector & Robot::bodySensors()
{
  return bodySensors_;
}

const BodySensorVector & Robot::bodySensors() const
{
  return bodySensors_;
}

bool Robot::hasJoint(std::string_view name) const
{
  return mb().jointIndexByName().count(std::string(name)) != 0;
}

bool Robot::hasBody(std::string_view name) const
{
  return mb().bodyIndexByName().count(std::string(name)) != 0;
}

unsigned int Robot::jointIndexByName(std::string_view name) const
{
  return mb().jointIndexByName().at(std::string(name));
}

unsigned int Robot::bodyIndexByName(std::string_view name) const
{
  return mb().bodyIndexByName().at(std::string(name));
}

sva::ForceVecd Robot::frameWrench(std::string_view frameName) const
{
  if(frameHasForceSensor(frameName))
  {
    const auto & fs = frameForceSensor(frameName);
    const auto & frame = frames_.find(frameName)->second;
    sva::ForceVecd w_fsactual = fs.wrenchWithoutGravity(*this);
    sva::PTransformd X_fsactual_frame = frame->X_b_f() * fs.X_fsactual_parent();
    return X_fsactual_frame.dualMul(w_fsactual);
  }
  else
  { /* If a force sensor is not directly attached to the body,
       attempt to find it up the kinematic tree */
    const auto & fs = findFrameForceSensor(frameName);
    const auto & frame = frames_.find(frameName)->second;
    sva::ForceVecd w_fsactual = fs.wrenchWithoutGravity(*this);
    const auto & X_0_frame = frame->position();
    const auto & X_0_parent = mbc().bodyPosW[mb().bodyIndexByName(fs.parentBody())];
    const auto X_parent_frame = X_0_frame * X_0_parent.inv();
    sva::PTransformd X_fsactual_surf = X_parent_frame * fs.X_fsactual_parent();
    return X_fsactual_surf.dualMul(w_fsactual);
  }
}

Eigen::Vector2d Robot::cop(std::string_view frameName, double min_pressure) const
{
  const sva::ForceVecd w_surf = frameWrench(frameName);
  const double pressure = w_surf.force()(2);
  if(pressure < min_pressure)
  {
    return Eigen::Vector2d::Zero();
  }
  const Eigen::Vector3d & tau_surf = w_surf.couple();
  return Eigen::Vector2d(-tau_surf(1) / pressure, +tau_surf(0) / pressure);
}

Eigen::Vector3d Robot::copW(std::string_view frameName, double min_pressure) const
{
  Eigen::Vector3d cop_s;
  cop_s << cop(frameName, min_pressure), 0.;
  const sva::PTransformd X_0_s = frames_.find(frameName)->second->position();
  return X_0_s.translation() + X_0_s.rotation().transpose() * cop_s;
}

sva::ForceVecd Robot::netWrench(const std::vector<std::string> & sensorNames) const
{
  // Compute net total wrench from all sensors in contact
  sva::ForceVecd netTotalWrench{sva::ForceVecd::Zero()};
  for(const auto & sensorName : sensorNames)
  {
    const auto & sensor = forceSensor(sensorName);
    netTotalWrench += sensor.worldWrenchWithoutGravity(*this);
  }
  return netTotalWrench;
}

Eigen::Vector3d Robot::zmp(const sva::ForceVecd & netTotalWrench,
                           const Eigen::Vector3d & plane_p,
                           const Eigen::Vector3d & plane_n,
                           double minimalNetNormalForce) const
{
  return mc_rbdyn::zmp(netTotalWrench, plane_p, plane_n, minimalNetNormalForce);
}

Eigen::Vector3d Robot::zmp(const sva::ForceVecd & netWrench,
                           const sva::PTransformd & zmpFrame,
                           double minimalNetNormalForce) const
{
  return mc_rbdyn::zmp(netWrench, zmpFrame, minimalNetNormalForce);
}

Eigen::Vector3d Robot::zmp(const std::vector<std::string> & sensorNames,
                           const Eigen::Vector3d & plane_p,
                           const Eigen::Vector3d & plane_n,
                           double minimalNetNormalForce) const
{
  return zmp(netWrench(sensorNames), plane_p, plane_n, minimalNetNormalForce);
}

Eigen::Vector3d Robot::zmp(const std::vector<std::string> & sensorNames,
                           const sva::PTransformd & zmpFrame,
                           double minimalNetNormalForce) const
{
  Eigen::Vector3d n = zmpFrame.rotation().row(2);
  Eigen::Vector3d p = zmpFrame.translation();
  return zmp(sensorNames, p, n, minimalNetNormalForce);
}

const std::vector<Flexibility> & Robot::flexibility() const
{
  return flexibility_;
}

std::vector<Flexibility> & Robot::flexibility()
{
  return flexibility_;
}

const std::vector<double> & Robot::encoderValues() const
{
  return encoderValues_;
}

void Robot::encoderValues(const std::vector<double> & encoderValues)
{
  encoderValues_ = encoderValues;
}

const std::vector<double> & Robot::encoderVelocities() const
{
  return encoderVelocities_;
}

void Robot::encoderVelocities(const std::vector<double> & encoderVelocities)
{
  encoderVelocities_ = encoderVelocities;
}

const std::vector<double> & Robot::flexibilityValues() const
{
  return flexibilityValues_;
}

void Robot::flexibilityValues(const std::vector<double> & flexibilityValues)
{
  flexibilityValues_ = flexibilityValues;
}

const std::vector<double> & Robot::jointTorques() const
{
  return jointTorques_;
}

void Robot::jointTorques(const std::vector<double> & jointTorques)
{
  jointTorques_ = jointTorques;
}

bool Robot::hasForceSensor(std::string_view name) const
{
  return forceSensorsIndex_.contains(name);
}

bool Robot::frameHasForceSensor(std::string_view frame) const
{
  return frameForceSensors_.contains(frame);
}

bool Robot::frameHasIndirectForceSensor(std::string_view frame) const
{
  return frameHasForceSensor(frame) || frameIndirectForceSensors_.contains(frame);
}

ForceSensor & Robot::forceSensor(std::string_view name)
{
  return const_cast<ForceSensor &>(static_cast<const Robot *>(this)->forceSensor(name));
}

const ForceSensor & Robot::forceSensor(std::string_view name) const
{
  auto it = forceSensorsIndex_.find(name);
  if(it == forceSensorsIndex_.cend())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No force sensor named {} in {}", name, this->name());
  }
  return forceSensors_[it->second];
}

ForceSensor & Robot::frameForceSensor(std::string_view frame)
{
  return const_cast<ForceSensor &>(static_cast<const Robot *>(this)->frameForceSensor(frame));
}

const ForceSensor & Robot::frameForceSensor(std::string_view frame) const
{
  auto it = frameForceSensors_.find(frame);
  if(it == frameForceSensors_.cend())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No force sensor directly attached to frame {} in {}", frame,
                                                     name());
  }
  return forceSensors_.at(it->second);
}

const ForceSensor & Robot::findFrameForceSensor(std::string_view frame) const
{
  auto it = frameIndirectForceSensors_.find(frame);
  if(it == frameIndirectForceSensors_.cend())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No force sensor indirectly attached to frame {} in {}", frame,
                                                     name());
  }
  return forceSensors_.at(it->second);
}

ForceSensor & Robot::findFrameForceSensor(std::string_view frame)
{
  return const_cast<ForceSensor &>(static_cast<const Robot *>(this)->frameForceSensor(frame));
}

bool Robot::hasSurface(std::string_view surface) const
{
  return surfaces_.contains(surface);
}

std::vector<ForceSensor> & Robot::forceSensors()
{
  return forceSensors_;
}

const std::vector<ForceSensor> & Robot::forceSensors() const
{
  return forceSensors_;
}

Surface & Robot::surface(std::string_view sName)
{
  return const_cast<Surface &>(const_cast<const Robot *>(this)->surface(sName));
}

const Surface & Robot::surface(std::string_view sName) const
{
  auto it = surfaces_.find(sName);
  if(it == surfaces_.cend())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No surface named {} in {}", sName, name_);
  }
  return *it->second;
}

const mc_rtc::map<std::string, SurfacePtr> & Robot::surfaces() const
{
  return surfaces_;
}

std::vector<std::string> Robot::availableSurfaces() const
{
  std::vector<std::string> ret;
  ret.reserve(surfaces_.size());
  for(const auto & s : surfaces_)
  {
    ret.push_back(s.first);
  }
  return ret;
}

bool Robot::hasConvex(std::string_view name) const
{
  return convexes_.contains(name);
}

Convex & Robot::convex(std::string_view cName)
{
  return const_cast<Convex &>(const_cast<const Robot *>(this)->convex(cName));
}

const Convex & Robot::convex(std::string_view cName) const
{
  auto it = convexes_.find(cName);
  if(it == convexes_.cend())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No convex named {} found in robot {}", cName, this->name_);
  }
  return *it->second;
}

Convex & Robot::addConvex(std::string_view cName, S_ObjectPtr object, std::string_view parent, sva::PTransformd X_f_c)
{
  if(convexes_.count(cName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Attempted to add a convex named {} that already exists in {}",
                                                     cName, name());
  }
  auto it = convexes_.emplace(
      cName, std::make_shared<Convex>(Convex::ctor_token{}, object, frame(parent, "Robot::addConvex"), X_f_c));
  if(!module_.collisionTransforms().contains(cName))
  {
    module_._collisionTransforms.emplace(cName, X_f_c);
  }
  return *it.first->second;
}

void Robot::removeConvex(std::string_view cName)
{
  auto it = convexes_.find(cName);
  if(it != convexes_.end())
  {
    convexes_.erase(it);
  }
}

const sva::PTransformd & Robot::bodyTransform(std::string_view bName) const
{
  if(!hasBody(bName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No body transform with name {} found in this robot", bName);
  }
  return bodyTransforms_[bodyIndexByName(bName)];
}

const sva::PTransformd & Robot::bodyTransform(int bodyIndex) const
{
  return bodyTransforms_[bodyIndex];
}

const std::vector<sva::PTransformd> & Robot::bodyTransforms() const
{
  return bodyTransforms_;
}

void Robot::loadRSDFFromDir(std::string_view surfaceDir)
{
  std::vector<SurfacePtr> surfacesIn = readRSDFFromDir(*this, surfaceDir);
  for(const auto & sp : surfacesIn)
  {
    surfaces_[sp->name()] = sp;
  }
}

void mc_rbdyn::Robot::eulerIntegration(double step)
{
  rbd::eulerIntegration(mb(), module_.mbc, step);
}

const sva::PTransformd & Robot::posW() const
{
  return mbc().bodyPosW[0];
}

void Robot::posW(const sva::PTransformd & pt)
{
  if(mb().joint(0).type() == rbd::Joint::Type::Free)
  {
    Eigen::Quaterniond r{pt.rotation().transpose()};
    r.normalize();
    const auto & t = pt.translation();
    Eigen::Matrix<double, 7, 1> qFB;
    qFB << r.w(), r.x(), r.y(), r.z(), t.x(), t.y(), t.z();
    q_fb_->set(qFB);
    forwardKinematics();
  }
  else if(mb().joint(0).type() == rbd::Joint::Type::Fixed)
  {
    module_.mb.transform(0, pt);
    forwardKinematics();
  }
  else
  {
    mc_rtc::log::error_and_throw<std::logic_error>(
        "The root pose can only be changed for robots with a free flyer or a fixed joint as joint(0)");
  }
}

void Robot::velW(const sva::MotionVecd & vel)
{
  if(mb().joint(0).type() == rbd::Joint::Type::Free)
  {
    auto vB = sva::PTransformd(mbc().bodyPosW[0].rotation()) * vel;
    module_.mbc.alpha[0][0] = vB.angular().x();
    module_.mbc.alpha[0][1] = vB.angular().y();
    module_.mbc.alpha[0][2] = vB.angular().z();
    module_.mbc.alpha[0][3] = vB.linear().x();
    module_.mbc.alpha[0][4] = vB.linear().y();
    module_.mbc.alpha[0][5] = vB.linear().z();
    rbd::forwardVelocity(mb(), module_.mbc);
  }
  else
  {
    mc_rtc::log::warning("You cannot set the base velocity on a fixed-base robot");
  }
}

const sva::MotionVecd & Robot::velW() const
{
  return mbc().bodyVelW[0];
}

void Robot::accW(const sva::MotionVecd & acc)
{
  if(mb().joint(0).type() == rbd::Joint::Type::Free)
  {
    auto aB = sva::PTransformd(mbc().bodyPosW[0].rotation()) * acc;
    auto & alphaD = module_.mbc.alphaD;
    alphaD[0][0] = aB.angular().x();
    alphaD[0][1] = aB.angular().y();
    alphaD[0][2] = aB.angular().z();
    alphaD[0][3] = aB.linear().x();
    alphaD[0][4] = aB.linear().y();
    alphaD[0][5] = aB.linear().z();
    forwardAcceleration();
  }
  else
  {
    mc_rtc::log::warning("You cannot set the base acceleration on a fixed-base robot");
  }
}

const sva::MotionVecd Robot::accW() const
{
  Eigen::Matrix3d rot = posW().rotation().transpose();
  return sva::PTransformd{rot} * mbc().bodyAccB[0];
}

RobotPtr Robot::copy(std::string_view name, const std::optional<Base> & base) const
{
  std::shared_ptr<Robot> robot_ptr;
  if(base)
  {
    robot_ptr = std::allocate_shared<Robot>(Eigen::aligned_allocator<Robot>{}, make_shared_token{}, module_, name,
                                            false, base.value().X_0_s, base.value().baseName);
  }
  else
  {
    robot_ptr =
        std::allocate_shared<Robot>(Eigen::aligned_allocator<Robot>{}, make_shared_token{}, module_, name, false);
  }
  auto & robot = *robot_ptr;
  std::function<void(const mc_rbdyn::RobotFrame &)> copyFrame = [&](const mc_rbdyn::RobotFrame & other) {
    if(!other.parent())
    {
      assert(robot.hasFrame(other.name()));
      return;
    }
    std::string_view parentFrame = other.parent()->name();
    if(!robot.hasFrame(parentFrame))
    {
      copyFrame(this->frame(parentFrame));
    }
    if(!robot.hasFrame(other.name()))
    {
      robot.makeFrame(other.name(), robot.frame(parentFrame), other.X_p_f());
    }
  };
  for(const auto & f_it : frames_)
  {
    const auto & f = *f_it.second;
    copyFrame(f);
  }
  for(const auto & s : surfaces_)
  {
    robot.surfaces_[s.first] = s.second->copy(robot);
  }
  for(const auto & c : convexes_)
  {
    const auto & convex = c.second;
    sch::S_Polyhedron * poly = dynamic_cast<sch::S_Polyhedron *>(convex->convex().get());
    if(!poly)
    {
      mc_rtc::log::warning("Could not copy convex {} from {} to {} as it is not an sch::S_Polyhedron, you are allowed "
                           "to complain to mc_rtc maintainers",
                           c.first, this->name(), name);
      continue;
    }
    robot.addConvex(c.first, std::make_shared<sch::S_Polyhedron>(*poly), convex->frame().name(), convex->X_f_c());
  }
  return robot_ptr;
}

void mc_rbdyn::Robot::addSurface(SurfacePtr surface, bool overwrite)
{
  if(&surface->frame().robot_ != this)
  {
    mc_rtc::log::warning("Trying to add surface {} to {} but this surface is attached to {}", surface->name(),
                         this->name(), surface->frame().robot().name());
    return;
  }
  if(hasSurface(surface->name()) && !overwrite)
  {
    mc_rtc::log::warning("Surface {} already exists for the robot {}.", surface->name(), name());
    return;
  }
  surfaces_[surface->name()] = std::move(surface);
}

MC_RTC_diagnostic_pop

bool Robot::hasGripper(const std::string & gripper) const
{
  return grippers_.count(gripper);
}

mc_control::Gripper & Robot::gripper(std::string_view gripper)
{
  auto it = grippers_.find(gripper);
  if(it == grippers_.end())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No gripper named {} in robot {}", gripper, name());
  }
  return *it->second;
}

void Robot::addDevice(DevicePtr device)
{
  if(devicesIndex_.count(device->name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You cannot have multiple generic sensor with the same name in a robot");
  }
  devices_.push_back(std::move(device));
  auto & d = devices_.back();
  if(d->parent() == "")
  {
    d->parent(mb().body(0).name());
  }
  devicesIndex_[device->name()] = devices_.size() - 1;
}

void Robot::updateFK()
{
  rbd::vectorToParam(q_->value(), module_.mbc.q);
  rbd::forwardKinematics(mb(), module_.mbc);
}

void Robot::updateFV()
{
  rbd::vectorToParam(dq_->value(), module_.mbc.alpha);
  rbd::forwardVelocity(mb(), module_.mbc);
}

void Robot::updateFA()
{
  rbd::forwardAcceleration(mb(), module_.mbc);
}

void Robot::updateNormalAcceleration()
{
  if(mb().nrDof() == 0)
  {
    return;
  }
  const auto & pred = mb().predecessors();
  const auto & succ = mb().successors();
  for(int i = 0; i < mb().nrJoints(); ++i)
  {
    const auto & X_p_i = mbc().parentToSon[static_cast<size_t>(i)];
    const auto & vj_i = mbc().jointVelocity[static_cast<size_t>(i)];
    const auto & vb_i = mbc().bodyVelB[static_cast<size_t>(i)];
    auto succ_i = static_cast<size_t>(succ[static_cast<size_t>(i)]);
    auto pred_i = pred[static_cast<size_t>(i)];
    normalAccB_[succ_i] = vb_i.cross(vj_i);
    if(pred_i != -1)
    {
      normalAccB_[succ_i] += X_p_i * normalAccB_[static_cast<size_t>(pred_i)];
    }
  }
}

void Robot::updateH()
{
  fd_.computeH(mb(), mbc());
}

void Robot::updateC()
{
  fd_.computeC(mb(), mbc());
}

void Robot::updateAll()
{
  updateFK();
  updateFV();
  updateFA();
  updateNormalAcceleration();
  updateH();
  updateC();
}

RobotFrame & Robot::frame(std::string_view frame, std::string_view context)
{
  return const_cast<RobotFrame &>(const_cast<const Robot *>(this)->frame(frame, context));
}

const RobotFrame & Robot::frame(std::string_view frame, std::string_view context) const
{
  auto it = frames_.find(frame);
  if(it == frames_.cend())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No frame named {} in {} (from: {})", frame, name(), context);
  }
  return *it->second;
}

RobotFrame & Robot::updateFrameForceSensors(RobotFrame & frame)
{
  const auto & body = frame.body();
  const auto & name = frame.name();
  auto direct_fs_it = frameForceSensors_.find(body);
  if(direct_fs_it != frameForceSensors_.end())
  {
    frameForceSensors_.emplace(name, direct_fs_it->second);
  }
  else
  {
    auto indirect_fs_it = frameIndirectForceSensors_.find(body);
    if(indirect_fs_it != frameIndirectForceSensors_.end())
    {
      frameIndirectForceSensors_.emplace(name, indirect_fs_it->second);
    }
  }
  return frame;
}

void Robot::forwardKinematics()
{
  updateFK();
  com_->updateCoM();
  momentum_->updateMomentum();
  for(auto & f : frames_)
  {
    if(!f.second->parent_)
    {
      f.second->updatePosition();
    }
  }
  for(auto & f : frames_)
  {
    if(f.second->parent_)
    {
      f.second->Frame::updatePosition();
    }
  }
}

void Robot::forwardVelocity()
{
  updateFV();
  com_->updateVelocity();
  com_->updateAcceleration();
  for(auto & f : frames_)
  {
    if(!f.second->parent_)
    {
      f.second->updateVelocity();
    }
  }
  for(auto & f : frames_)
  {
    if(f.second->parent_)
    {
      f.second->Frame::updateVelocity();
    }
  }
}

void Robot::forwardAcceleration()
{
  updateFA();
}

tvm::VariablePtr Robot::qJoint(size_t jIdx)
{
  if(jIdx > mb().joints().size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} has no joint at index {}", name_, jIdx);
  }
  if(mb().joints()[jIdx].dof() == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} is not actuated in {}", mb().joints()[jIdx].name(), name_);
  }
  auto offsetParam = mb().jointPosInParam(static_cast<int>(jIdx));
  auto offsetDof = mb().jointPosInDof(static_cast<int>(jIdx));
  const auto & j = mb().joints()[jIdx];
  return q_->subvariable(tvm::Space(j.dof(), j.params(), j.dof()), j.name(),
                         tvm::Space(offsetDof, offsetParam, offsetDof));
}

} // namespace mc_rbdyn
