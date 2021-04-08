/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/configuration_io.h>

#include <RBDyn/parsers/urdf.h>

#include <mc_rtc/deprecated.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace
{
// Return relative path to go to "to" from "from"
bfs::path relative(bfs::path to, bfs::path from)
{
  bfs::path::const_iterator fromIter = from.begin();
  bfs::path::const_iterator toIter = to.begin();
  while(fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter))
  {
    ++toIter;
    ++fromIter;
  }
  bfs::path relPath;
  while(fromIter != from.end() && *fromIter != bfs::path("."))
  {
    relPath /= "..";
    ++fromIter;
  }
  while(toIter != to.end())
  {
    relPath /= *toIter;
    ++toIter;
  }
  return relPath;
}
} // namespace

namespace mc_rtc
{

rbd::Joint::Type ConfigurationLoader<rbd::Joint::Type>::load(const mc_rtc::Configuration & config)
{
  std::string type = "";
  config("type", type);
  if(type == "rev")
  {
    return rbd::Joint::Type::Rev;
  }
  if(type == "prism")
  {
    return rbd::Joint::Type::Prism;
  }
  if(type == "spherical")
  {
    return rbd::Joint::Type::Spherical;
  }
  if(type == "planar")
  {
    return rbd::Joint::Type::Planar;
  }
  if(type == "cylindrical")
  {
    return rbd::Joint::Type::Cylindrical;
  }
  if(type == "free")
  {
    return rbd::Joint::Type::Free;
  }
  if(type == "fixed")
  {
    return rbd::Joint::Type::Fixed;
  }
  mc_rtc::log::error_and_throw<std::runtime_error>("{} was stored as joint type, cannot comprehend that", type);
}

mc_rtc::Configuration ConfigurationLoader<rbd::Joint::Type>::save(const rbd::Joint::Type & type)
{
  mc_rtc::Configuration config;
  std::string typeStr = "";
  switch(type)
  {
    case rbd::Joint::Type::Rev:
      typeStr = "rev";
      break;
    case rbd::Joint::Type::Prism:
      typeStr = "prism";
      break;
    case rbd::Joint::Type::Spherical:
      typeStr = "spherical";
      break;
    case rbd::Joint::Type::Planar:
      typeStr = "planar";
      break;
    case rbd::Joint::Type::Cylindrical:
      typeStr = "cylindrical";
      break;
    case rbd::Joint::Type::Free:
      typeStr = "free";
      break;
    case rbd::Joint::Type::Fixed:
      typeStr = "fixed";
      break;
    default:
      mc_rtc::log::error_and_throw<std::runtime_error>("Cannot serialize joint type {}", type);
  }
  config.add("type", typeStr);
  return config;
}

mc_rbdyn::Base ConfigurationLoader<mc_rbdyn::Base>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("X_0_s"), config("X_b0_s"), config("type")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Base>::save(const mc_rbdyn::Base & b)
{
  mc_rtc::Configuration config;
  config.add("name", b.baseName);
  config.add("X_0_s", b.X_0_s);
  config.add("X_b0_s", b.X_b0_s);
  config.add("type", b.baseType);
  return config;
}

mc_rbdyn::BodySensor ConfigurationLoader<mc_rbdyn::BodySensor>::load(const mc_rtc::Configuration & config)
{
  return mc_rbdyn::BodySensor(config("name"), config("parentBody"), config("X_b_s"));
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::BodySensor>::save(const mc_rbdyn::BodySensor & bs)
{
  mc_rtc::Configuration config;
  config.add("name", bs.name());
  config.add("parentBody", bs.parentBody());
  config.add("X_b_s", bs.X_b_s());
  return config;
}

namespace
{

std::string object1OrBody1(const mc_rtc::Configuration & config)
{
  if(config.has("object1"))
  {
    return config("object1");
  }
  mc_rtc::log::deprecated("Collision", "object1", "body1");
  return config("body1");
}

std::string object2OrBody2(const mc_rtc::Configuration & config)
{
  if(config.has("object2"))
  {
    return config("object2");
  }
  mc_rtc::log::deprecated("Collision", "object2", "body2");
  return config("body2");
}

} // namespace

mc_rbdyn::Collision ConfigurationLoader<mc_rbdyn::Collision>::load(const mc_rtc::Configuration & config)
{
  if(config.has("robot"))
  {
    return {config("robot"),       object1OrBody1(config), object2OrBody2(config),
            config("iDist", 0.05), config("sDist", 0.01),  config("damping", 0.0)};
  }
  else
  {
    return {config("robot1"),      config("robot2"),      object1OrBody1(config), object2OrBody2(config),
            config("iDist", 0.05), config("sDist", 0.01), config("damping", 0.0)};
  }
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Collision>::save(const mc_rbdyn::Collision & c)
{
  mc_rtc::Configuration config;
  config.add("robot1", c.robot1);
  config.add("robot2", c.robot2);
  config.add("object1", c.object1);
  config.add("object2", c.object2);
  config.add("iDist", c.iDist);
  config.add("sDist", c.sDist);
  config.add("damping", c.damping);
  return config;
}

mc_rbdyn::CollisionDescription ConfigurationLoader<mc_rbdyn::CollisionDescription>::load(
    const mc_rtc::Configuration & config)
{
  return {object1OrBody1(config), object2OrBody2(config), config("iDist"), config("sDist"), config("damping")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::CollisionDescription>::save(const mc_rbdyn::CollisionDescription & c)
{
  mc_rtc::Configuration config;
  config.add("object1", c.object1);
  config.add("object2", c.object2);
  config.add("iDist", c.iDist);
  config.add("sDist", c.sDist);
  config.add("damping", c.damping);
  return config;
}

mc_rbdyn::CollisionVector ConfigurationLoader<mc_rbdyn::CollisionVector>::load(const mc_rtc::Configuration & config)
{
  if(config.has("robot"))
  {
    return {config("robot"), config("collisions")};
  }
  else
  {
    return {config("robot1"), config("robot2"), config("collisions")};
  }
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::CollisionVector>::save(const mc_rbdyn::CollisionVector & c)
{
  mc_rtc::Configuration config;
  auto out = config.array("out", c.size());
  for(const auto & col : c)
  {
    out.push(col);
  }
  return out;
}

mc_rbdyn::Contact ConfigurationLoader<mc_rbdyn::Contact>::load(const mc_rtc::Configuration & config)
{
  return {config("r1"),
          config("r2"),
          config("r1Surface"),
          config("r2Surface"),
          config("friction", mc_rbdyn::Contact::defaultFriction),
          config("dof", Eigen::Vector6d::Ones().eval())};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Contact>::save(const mc_rbdyn::Contact & c)
{
  mc_rtc::Configuration out;
  out.add("r1", c.r1);
  out.add("r1Surface", c.r1Surface);
  out.add("r2", c.r2);
  out.add("r2Surface", c.r2Surface);
  out.add("dof", c.dof);
  out.add("friction", c.friction);
  return out;
}

mc_rbdyn::Flexibility ConfigurationLoader<mc_rbdyn::Flexibility>::load(const mc_rtc::Configuration & config)
{
  return {config("jointName"), config("K"), config("C"), config("O")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Flexibility>::save(const mc_rbdyn::Flexibility & flex)
{
  mc_rtc::Configuration config;
  config.add("jointName", flex.jointName);
  config.add("K", flex.K);
  config.add("C", flex.C);
  config.add("O", flex.O);
  return config;
}

mc_rbdyn::ForceSensor ConfigurationLoader<mc_rbdyn::ForceSensor>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("parentBody"), config("X_p_f")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::ForceSensor>::save(const mc_rbdyn::ForceSensor & fs)
{
  mc_rtc::Configuration config;
  config.add("name", fs.name());
  config.add("parentBody", fs.parentBody());
  config.add("X_p_f", fs.X_p_f());
  return config;
}

mc_rbdyn::Plane ConfigurationLoader<mc_rbdyn::Plane>::load(const mc_rtc::Configuration & config)
{
  return mc_rbdyn::Plane{config("normal"), config("offset")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Plane>::save(const mc_rbdyn::Plane & pl)
{
  mc_rtc::Configuration config;
  config.add("normal", pl.normal);
  config.add("offset", pl.offset);
  return config;
}

mc_rbdyn::PolygonInterpolator ConfigurationLoader<mc_rbdyn::PolygonInterpolator>::load(
    const mc_rtc::Configuration & config)
{
  std::vector<mc_rbdyn::PolygonInterpolator::tuple_pair_t> vec = config("tuple_pairs");
  return mc_rbdyn::PolygonInterpolator(vec);
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::PolygonInterpolator>::save(const mc_rbdyn::PolygonInterpolator & pi)
{
  mc_rtc::Configuration config;
  config.add("tuple_pairs", pi.tuple_pairs());
  return config;
}

mc_rbdyn::Springs ConfigurationLoader<mc_rbdyn::Springs>::load(const mc_rtc::Configuration & config)
{
  mc_rbdyn::Springs spr;
  spr.springsBodies = config("springsBodies");
  spr.afterSpringsBodies = config("afterSpringsBodies");
  spr.springsJoints = config("springsJoints");
  return spr;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Springs>::save(const mc_rbdyn::Springs & spr)
{
  mc_rtc::Configuration config;
  config.add("springsBodies", spr.springsBodies);
  config.add("afterSpringsBodies", spr.afterSpringsBodies);
  config.add("springsJoints", spr.springsJoints);
  return config;
}

sva::RBInertiad ConfigurationLoader<sva::RBInertiad>::load(const mc_rtc::Configuration & config)
{
  Eigen::Matrix3d inertia = config("inertia");
  return {config("mass"), config("momentum"), inertia};
}

mc_rtc::Configuration ConfigurationLoader<sva::RBInertiad>::save(const sva::RBInertiad & rbi)
{
  mc_rtc::Configuration config;
  config.add("mass", rbi.mass());
  config.add("momentum", rbi.momentum());
  config.add("inertia", rbi.inertia());
  return config;
}

rbd::Body ConfigurationLoader<rbd::Body>::load(const mc_rtc::Configuration & config)
{
  return {config("inertia"), config("name")};
}

mc_rtc::Configuration ConfigurationLoader<rbd::Body>::save(const rbd::Body & bod)
{
  mc_rtc::Configuration config;
  config.add("name", bod.name());
  config.add("inertia", bod.inertia());
  return config;
}

rbd::Joint ConfigurationLoader<rbd::Joint>::load(const mc_rtc::Configuration & config)
{
  rbd::Joint j{config("type"), config("axis"), config("forward"), config("name")};
  bool isMimic = config("isMimic");
  if(isMimic)
  {
    j.makeMimic(config("mimicName"), config("mimicMultiplier"), config("mimicOffset"));
  }
  return j;
}

mc_rtc::Configuration ConfigurationLoader<rbd::Joint>::save(const rbd::Joint & j)
{
  mc_rtc::Configuration config;
  config.add("type", j.type());
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  switch(j.type())
  {
    case rbd::Joint::Rev:
    case rbd::Joint::Cylindrical:
      axis = j.direction() * j.motionSubspace().col(0).head<3>();
      break;
    case rbd::Joint::Prism:
      axis = j.direction() * j.motionSubspace().col(0).tail<3>();
      break;
    default:
      break;
  }
  config.add("axis", axis);
  config.add("forward", j.forward());
  config.add("isMimic", j.isMimic());
  if(j.isMimic())
  {
    config.add("mimicName", j.mimicName());
    config.add("mimicMultiplier", j.mimicMultiplier());
    config.add("mimicOffset", j.mimicOffset());
  }
  config.add("name", j.name());
  return config;
}

rbd::MultiBody ConfigurationLoader<rbd::MultiBody>::load(const mc_rtc::Configuration & config)
{
  return {config("bodies"), config("joints"),  config("preds"),
          config("succs"),  config("parents"), config("transforms")};
}

mc_rtc::Configuration ConfigurationLoader<rbd::MultiBody>::save(const rbd::MultiBody & mb)
{
  mc_rtc::Configuration config;
  config.add("bodies", mb.bodies());
  config.add("joints", mb.joints());
  config.add("preds", mb.predecessors());
  config.add("succs", mb.successors());
  config.add("parents", mb.parents());
  config.add("transforms", mb.transforms());
  return config;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> ConfigurationLoader<Eigen::Matrix<double, 6, Eigen::Dynamic>>::load(
    const mc_rtc::Configuration & config)
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> m(6, static_cast<int>(config("cols")));
  auto data = config("data");
  if(static_cast<Eigen::DenseIndex>(data.size()) != 6 * m.cols())
  {
    auto msg = fmt::format("Stored data size ({}) is different from the expected size ({})", data.size(), 6 * m.cols());
    mc_rtc::log::critical(msg);
    throw mc_rtc::Configuration::Exception(msg, data);
  }
  for(Eigen::DenseIndex i = 0; i < 6; ++i)
  {
    for(Eigen::DenseIndex j = 0; j < m.cols(); ++j)
    {
      m(i, j) = data[static_cast<size_t>(m.cols() * i + j)];
    }
  }
  return m;
}

mc_rtc::Configuration ConfigurationLoader<Eigen::Matrix<double, 6, Eigen::Dynamic>>::save(
    const Eigen::Matrix<double, 6, Eigen::Dynamic> & m)
{
  mc_rtc::Configuration config;
  config.add("cols", static_cast<int>(m.cols()));
  auto data = config.array("data", static_cast<size_t>(6 * m.cols()));
  for(Eigen::DenseIndex i = 0; i < 6; ++i)
  {
    for(Eigen::DenseIndex j = 0; j < m.cols(); ++j)
    {
      data.push(m(i, j));
    }
  }
  return config;
}

rbd::MultiBodyConfig ConfigurationLoader<rbd::MultiBodyConfig>::load(const mc_rtc::Configuration & config)
{
  rbd::MultiBodyConfig mbc;
  mbc.q = config("q");
  mbc.alpha = config("alpha");
  mbc.alphaD = config("alphaD");
  mbc.force = config("force");
  mbc.jointConfig = config("jointConfig");
  mbc.jointVelocity = config("jointVelocity");
  mbc.jointTorque = config("jointTorque");
  mbc.motionSubspace = config("motionSubspace");
  mbc.bodyPosW = config("bodyPosW");
  mbc.parentToSon = config("parentToSon");
  mbc.bodyVelW = config("bodyVelW");
  mbc.bodyVelB = config("bodyVelB");
  mbc.bodyAccB = config("bodyAccB");
  mbc.gravity = config("gravity");
  return mbc;
}

mc_rtc::Configuration ConfigurationLoader<rbd::MultiBodyConfig>::save(const rbd::MultiBodyConfig & mbc)
{
  mc_rtc::Configuration config;
  config.add("q", mbc.q);
  config.add("alpha", mbc.alpha);
  config.add("alphaD", mbc.alphaD);
  config.add("force", mbc.force);
  config.add("jointConfig", mbc.jointConfig);
  config.add("jointVelocity", mbc.jointVelocity);
  config.add("jointTorque", mbc.jointTorque);
  config.add("motionSubspace", mbc.motionSubspace);
  config.add("bodyPosW", mbc.bodyPosW);
  config.add("parentToSon", mbc.parentToSon);
  config.add("bodyVelW", mbc.bodyVelW);
  config.add("bodyVelB", mbc.bodyVelB);
  config.add("bodyAccB", mbc.bodyAccB);
  config.add("gravity", mbc.gravity);
  return config;
}

mc_rbdyn::Mimic ConfigurationLoader<mc_rbdyn::Mimic>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("joint"), config("multiplier", 1.0), config("offset", 0.0)};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Mimic>::save(const mc_rbdyn::Mimic & mimic)
{
  mc_rtc::Configuration config;
  config.add("name", mimic.name);
  config.add("joint", mimic.joint);
  config.add("multiplier", mimic.multiplier);
  config.add("offset", mimic.offset);
  return config;
}

namespace
{

mc_rbdyn::RobotModule::Gripper loadGripper(const mc_rtc::Configuration & config,
                                           const mc_rbdyn::RobotModule::Gripper::Safety & safety)
{
  const std::string & name = config("name");
  const std::vector<std::string> & joints = config("joints");
  bool reverse_limits = config("reverse_limits");
  if(config.has("safety"))
  {
    const mc_rbdyn::RobotModule::Gripper::Safety & safety = config("safety");
    if(config.has("mimics"))
    {
      return {name, joints, reverse_limits, safety, config("mimics")};
    }
    return {name, joints, reverse_limits, safety};
  }
  if(config.has("mimics"))
  {
    return {name, joints, reverse_limits, safety, config("mimics")};
  }
  return {name, joints, reverse_limits};
}

} // namespace

mc_rbdyn::RobotModule::Gripper ConfigurationLoader<mc_rbdyn::RobotModule::Gripper>::load(
    const mc_rtc::Configuration & config)
{
  return loadGripper(config, {});
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModule::Gripper>::save(
    const mc_rbdyn::RobotModule::Gripper & rmg)
{
  mc_rtc::Configuration config;
  config.add("name", rmg.name);
  config.add("joints", rmg.joints);
  config.add("reverse_limits", rmg.reverse_limits);
  auto safety = rmg.safety();
  if(safety)
  {
    config.add("safety", *safety);
  }
  auto mimics = rmg.mimics();
  if(mimics)
  {
    config.add("mimics", *mimics);
  }
  return config;
}

mc_rbdyn::RobotModule::Gripper::Safety ConfigurationLoader<mc_rbdyn::RobotModule::Gripper::Safety>::load(
    const mc_rtc::Configuration & config)
{
  mc_rbdyn::RobotModule::Gripper::Safety safety;
  safety.load(config);
  return safety;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModule::Gripper::Safety>::save(
    const mc_rbdyn::RobotModule::Gripper::Safety & safety)
{
  return safety.save();
}

rbd::parsers::Geometry::Box ConfigurationLoader<rbd::parsers::Geometry::Box>::load(const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Box b;
  b.size = config("size");
  return b;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Box>::save(const rbd::parsers::Geometry::Box & b)
{
  mc_rtc::Configuration config;
  config.add("size", b.size);
  return config;
}

rbd::parsers::Geometry::Cylinder ConfigurationLoader<rbd::parsers::Geometry::Cylinder>::load(
    const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Cylinder c;
  c.radius = config("radius");
  c.length = config("length");
  return c;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Cylinder>::save(
    const rbd::parsers::Geometry::Cylinder & c)
{
  mc_rtc::Configuration config;
  config.add("radius", c.radius);
  config.add("length", c.length);
  return config;
}

rbd::parsers::Geometry::Sphere ConfigurationLoader<rbd::parsers::Geometry::Sphere>::load(
    const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Sphere s;
  s.radius = config("radius");
  return s;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Sphere>::save(const rbd::parsers::Geometry::Sphere & s)
{
  mc_rtc::Configuration config;
  config.add("radius", s.radius);
  return config;
}

rbd::parsers::Geometry::Mesh ConfigurationLoader<rbd::parsers::Geometry::Mesh>::load(
    const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Mesh m;
  m.filename = static_cast<std::string>(config("filename"));
  m.scale = config("scale");
  return m;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Mesh>::save(const rbd::parsers::Geometry::Mesh & m)
{
  mc_rtc::Configuration config;
  config.add("filename", m.filename);
  config.add("scale", m.scale);
  return config;
}

rbd::parsers::Geometry::Superellipsoid ConfigurationLoader<rbd::parsers::Geometry::Superellipsoid>::load(
    const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Superellipsoid s;
  s.size = config("size");
  s.epsilon1 = config("epsilon1");
  s.epsilon2 = config("epsilon2");
  return s;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Superellipsoid>::save(
    const rbd::parsers::Geometry::Superellipsoid & s)
{
  mc_rtc::Configuration config;
  config.add("size", s.size);
  config.add("epsilon1", s.epsilon1);
  config.add("epsilon2", s.epsilon2);
  return config;
}

rbd::parsers::Geometry ConfigurationLoader<rbd::parsers::Geometry>::load(const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry geom;
  if(config.has("box"))
  {
    geom.type = rbd::parsers::Geometry::Type::BOX;
    rbd::parsers::Geometry::Box b = config("box");
    geom.data = b;
  }
  else if(config.has("cylinder"))
  {
    geom.type = rbd::parsers::Geometry::Type::CYLINDER;
    rbd::parsers::Geometry::Cylinder b = config("cylinder");
    geom.data = b;
  }
  else if(config.has("sphere"))
  {
    geom.type = rbd::parsers::Geometry::Type::SPHERE;
    rbd::parsers::Geometry::Sphere b = config("sphere");
    geom.data = b;
  }
  else if(config.has("mesh"))
  {
    geom.type = rbd::parsers::Geometry::Type::MESH;
    rbd::parsers::Geometry::Mesh b = config("mesh");
    geom.data = b;
  }
  else if(config.has("superellipsoid"))
  {
    geom.type = rbd::parsers::Geometry::Type::SUPERELLIPSOID;
    rbd::parsers::Geometry::Superellipsoid s = config("superellipsoid");
    geom.data = s;
  }
  return geom;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry>::save(const rbd::parsers::Geometry & geom)
{
  mc_rtc::Configuration config;
  switch(geom.type)
  {
    case rbd::parsers::Geometry::Type::BOX:
      config.add("box", boost::get<rbd::parsers::Geometry::Box>(geom.data));
      break;
    case rbd::parsers::Geometry::Type::CYLINDER:
      config.add("cylinder", boost::get<rbd::parsers::Geometry::Cylinder>(geom.data));
      break;
    case rbd::parsers::Geometry::Type::SPHERE:
      config.add("sphere", boost::get<rbd::parsers::Geometry::Sphere>(geom.data));
      break;
    case rbd::parsers::Geometry::Type::MESH:
      config.add("mesh", boost::get<rbd::parsers::Geometry::Mesh>(geom.data));
      break;
    case rbd::parsers::Geometry::Type::SUPERELLIPSOID:
      config.add("superellipsoid", boost::get<rbd::parsers::Geometry::Superellipsoid>(geom.data));
      break;
    default:
      break;
  }
  return config;
}

rbd::parsers::Visual ConfigurationLoader<rbd::parsers::Visual>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("origin"), config("geometry")};
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Visual>::save(const rbd::parsers::Visual & vis)
{
  mc_rtc::Configuration config;
  config.add("name", vis.name);
  config.add("origin", vis.origin);
  config.add("geometry", vis.geometry);
  return config;
}

mc_rbdyn::RobotModule ConfigurationLoader<mc_rbdyn::RobotModule>::load(const mc_rtc::Configuration & config)
{
  bfs::path path((std::string)config("path"));
  bfs::path urdf_path((std::string)config("urdf_path"));
  if(!urdf_path.is_absolute())
  {
    urdf_path = path / urdf_path;
  }
  mc_rbdyn::RobotModule rm(path.string(), std::string(config("name")), urdf_path.string());
  if(config.has("mb"))
  {
    rm.mb = config("mb");
    rm.mbc = config("mbc");
    rm._bounds = config("bounds");
    rm._visual = config("visuals");
    rm._collisionTransforms = config("collisionTransforms");
  }
  else
  {
    auto filteredLinks = config("filteredLinks", std::vector<std::string>{});
    auto fixed = config("fixed", false);
    if(!bfs::exists(rm.urdf_path))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Could not open model for {} at {}", rm.name, rm.urdf_path);
    }
    rm.init(rbd::parsers::from_urdf_file(rm.urdf_path, fixed));
  }
  if(config.has("accelerationBounds"))
  {
    mc_rbdyn::RobotModule::bounds_t aBounds = config("accelerationBounds");
    if(aBounds.size() != 2)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("accelerationBounds entry should be an array of size 2");
    }
    rm._accelerationBounds.resize(2);
    rm._accelerationBounds[0] = aBounds[0];
    rm._accelerationBounds[1] = aBounds[1];
  }
  if(config.has("torqueDerivativeBounds"))
  {
    mc_rbdyn::RobotModule::bounds_t tdBounds = config("torqueDerivativeBounds");
    if(tdBounds.size() != 2)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("torqueDerivativeBounds entry should be an array of size 2");
    }
    rm._torqueDerivativeBounds.resize(2);
    rm._torqueDerivativeBounds[0] = tdBounds[0];
    rm._torqueDerivativeBounds[1] = tdBounds[1];
  }
  /* Default values work fine for those */
  if(config.has("rsdf_dir"))
  {
    bfs::path rsdf_dir((std::string)config("rsdf_dir"));
    if(!rsdf_dir.is_absolute())
    {
      rsdf_dir = path / rsdf_dir;
    }
    rm.rsdf_dir = rsdf_dir.string();
  }
  config("convexHulls", rm._convexHull);
  for(auto & cH : rm._convexHull)
  {
    bfs::path chPath(cH.second.second);
    if(!chPath.is_absolute())
    {
      cH.second.second = (path / chPath).string();
    }
  }
  config("flexibilities", rm._flexibility);
  config("forceSensors", rm._forceSensors);
  config("bodySensors", rm._bodySensors);
  config("springs", rm._springs);
  config("minimalSelfCollisions", rm._minimalSelfCollisions);
  config("commonSelfCollisions", rm._commonSelfCollisions);
  config("default_attitude", rm._default_attitude);

  /* Those cannot be empty */
  config("stance", rm._stance);
  rm.expand_stance();
  if(config.has("ref_joint_order"))
  {
    rm._ref_joint_order = config("ref_joint_order");
  }
  else
  {
    rm.make_default_ref_joint_order();
  }

  if(config.has("gripperSafety"))
  {
    rm._gripperSafety = config("gripperSafety");
  }
  if(config.has("grippers"))
  {
    std::vector<mc_rtc::Configuration> grippers = config("grippers");
    for(auto & g : grippers)
    {
      rm._grippers.push_back(loadGripper(g, rm.gripperSafety()));
    }
  }
  if(config.has("lipmStabilizer"))
  {
    rm._lipmStabilizerConfig.load(config("lipmStabilizer"));
  }

  return rm;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModule>::save(const mc_rbdyn::RobotModule & rm,
                                                                       bool save_mbc,
                                                                       const std::vector<std::string> & filteredLinks,
                                                                       bool fixed)
{
  mc_rtc::Configuration config;
  config.add("path", rm.path);
  config.add("name", rm.name);
  config.add("urdf_path", relative(rm.urdf_path, rm.path).string());
  config.add("rsdf_dir", relative(rm.rsdf_dir, rm.path).string());
  if(save_mbc)
  {
    config.add("mb", rm.mb);
    config.add("mbc", rm.mbc);
    config.add("collisionTransforms", rm._collisionTransforms);
    config.add("bounds", rm._bounds);
    config.add("visuals", rm._visual);
  }
  else
  {
    config.add("filteredLinks", filteredLinks);
    config.add("fixed", fixed);
  }
  if(rm._bounds.size() != 6)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Wrong number ({}) of _bounds entries in RobotModule",
                                                     rm._bounds.size());
  }
  if(rm._accelerationBounds.size() != 0 && rm._accelerationBounds.size() != 2)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Wrong number ({}) of _accelerationBounds entries in RobotModule",
                                                     rm._accelerationBounds.size());
  }
  if(rm._torqueDerivativeBounds.size() != 0 && rm._torqueDerivativeBounds.size() != 2)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Wrong number ({}) of _torqueDerivativeBounds entries in RobotModule", rm._torqueDerivativeBounds.size());
  }
  if(rm._accelerationBounds.size() == 2)
  {
    config.add("accelerationBounds", rm._accelerationBounds);
  }
  if(rm._torqueDerivativeBounds.size() == 2)
  {
    config.add("torqueDerivativeBounds", rm._torqueDerivativeBounds);
  }
  config.add("stance", rm._stance);
  auto cHs = rm._convexHull;
  for(auto & cH : cHs)
  {
    cH.second.second = relative(cH.second.second, rm.path).string();
  }
  config.add("convexHulls", cHs);
  auto sHs = rm._convexHull;
  for(auto & sH : sHs)
  {
    sH.second.second = relative(sH.second.second, rm.path).string();
  }
  config.add("flexibilities", rm._flexibility);
  config.add("forceSensors", rm._forceSensors);
  config.add("bodySensors", rm._bodySensors);
  config.add("springs", rm._springs);
  config.add("minimalSelfCollisions", rm._minimalSelfCollisions);
  config.add("commonSelfCollisions", rm._commonSelfCollisions);
  config.add("grippers", rm._grippers);
  config.add("ref_joint_order", rm._ref_joint_order);
  config.add("default_attitude", rm._default_attitude);
  config.add("gripperSafety", rm._gripperSafety);
  config.add("lipmStabilizer", rm._lipmStabilizerConfig);
  return config;
}

mc_rbdyn::RobotModulePtr ConfigurationLoader<mc_rbdyn::RobotModulePtr>::load(const mc_rtc::Configuration & config)
{
  mc_rbdyn::RobotModule rm = config;
  return std::make_shared<mc_rbdyn::RobotModule>(std::move(rm));
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModulePtr>::save(const mc_rbdyn::RobotModulePtr & rm,
                                                                          bool save_mbc,
                                                                          const std::vector<std::string> & filteredLinks,
                                                                          bool fixed)
{
  return ConfigurationLoader<mc_rbdyn::RobotModule>::save(*rm, save_mbc, filteredLinks, fixed);
}

} // namespace mc_rtc
