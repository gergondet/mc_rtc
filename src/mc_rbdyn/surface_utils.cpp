/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/surface_utils.h>

#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/rpy_utils.h>

// For some dom manipulations
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>

#include <geos/version.h>

#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>

#include <fstream>
#include <tinyxml2.h>

namespace bfs = boost::filesystem;

namespace mc_rbdyn
{

namespace details
{

inline sva::PTransformd tfFromOriginDom(const tinyxml2::XMLElement & dom)
{
  Eigen::Vector3d xyz = rbd::parsers::attrToVector(dom, "xyz");
  Eigen::Vector3d rpy = rbd::parsers::attrToVector(dom, "rpy");
  return sva::PTransformd(rpyToMat(rpy), xyz);
}

template<typename SurfT, typename... Args>
void try_push_surface(Robot & robot,
                      std::vector<SurfacePtr> & surfaces,
                      std::string_view name,
                      std::string_view body,
                      const sva::PTransformd & X_b_s,
                      Args &&... args)
{
  if(robot.hasFrame(name))
  {
    if(X_b_s != sva::PTransformd::Identity())
    {
      mc_rtc::log::error(
          "Robot {} already has a frame named {} with a different definition, discarding this surface loading",
          robot.name(), name);
      return;
    }
    auto & frame = robot.frame(name);
    surfaces.push_back(std::make_shared<SurfT>(name, frame, std::forward<Args>(args)...));
    return;
  }
  if(robot.hasSurface(name))
  {
    mc_rtc::log::error("Robot {} already has a surface named {}, discarding this surface loading", robot.name(), name);
    return;
  }
  auto & frame = robot.makeFrame(name, body, X_b_s * robot.bodyTransform(body));
  surfaces.push_back(std::make_shared<SurfT>(name, frame, std::forward<Args>(args)...));
}

inline void readRSDF(Robot & robot, const std::string & rsdf_string, std::vector<SurfacePtr> & surfaces)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(rsdf_string.c_str());

  tinyxml2::XMLElement * root = doc.FirstChildElement("robot");
  std::vector<tinyxml2::XMLElement *> psurfaces;
  {
    tinyxml2::XMLElement * psurface = root->FirstChildElement("planar_surface");
    while(psurface)
    {
      psurfaces.push_back(psurface);
      psurface = psurface->NextSiblingElement("planar_surface");
    }
  }
  for(tinyxml2::XMLElement * pdom : psurfaces)
  {
    std::string name = pdom->Attribute("name");
    std::string bodyName = pdom->Attribute("link");
    sva::PTransformd X_b_s = tfFromOriginDom(*(pdom->FirstChildElement("origin")));
    std::vector<std::pair<double, double>> points;
    tinyxml2::XMLElement * pointdom = pdom->FirstChildElement("points")->FirstChildElement("point");
    while(pointdom)
    {
      std::vector<double> pdata = rbd::parsers::attrToList(*pointdom, "xy");
      points.push_back(std::pair<double, double>(pdata[0], pdata[1]));
      pointdom = pointdom->NextSiblingElement("point");
    }
    try_push_surface<PlanarSurface>(robot, surfaces, name, bodyName, X_b_s, points);
  }

  std::vector<tinyxml2::XMLElement *> csurfaces;
  {
    tinyxml2::XMLElement * csurface = root->FirstChildElement("cylindrical_surface");
    while(csurface)
    {
      csurfaces.push_back(csurface);
      csurface = csurface->NextSiblingElement("cylindrical_surface");
    }
  }
  for(tinyxml2::XMLElement * cdom : csurfaces)
  {
    std::string name = cdom->Attribute("name");
    std::string bodyName = cdom->Attribute("link");
    double width = cdom->DoubleAttribute("width");
    double radius = cdom->DoubleAttribute("radius");
    sva::PTransformd X_b_s = tfFromOriginDom(*(cdom->FirstChildElement("origin")));
    try_push_surface<CylindricalSurface>(robot, surfaces, name, bodyName, X_b_s, radius, width);
  }

  std::vector<tinyxml2::XMLElement *> gsurfaces;
  {
    tinyxml2::XMLElement * gsurface = root->FirstChildElement("gripper_surface");
    while(gsurface)
    {
      gsurfaces.push_back(gsurface);
      gsurface = gsurface->NextSiblingElement("gripper_surface");
    }
  }
  for(tinyxml2::XMLElement * gdom : gsurfaces)
  {
    std::string name = gdom->Attribute("name");
    std::string bodyName = gdom->Attribute("link");
    sva::PTransformd X_b_s = tfFromOriginDom(*(gdom->FirstChildElement("origin")));
    tinyxml2::XMLElement * motorDom = gdom->FirstChildElement("motor");
    sva::PTransformd X_b_motor = tfFromOriginDom(*motorDom);
    double motorMaxTorque = motorDom->DoubleAttribute("max_torque");
    std::vector<sva::PTransformd> points;
    tinyxml2::XMLElement * pointdom = gdom->FirstChildElement("points")->FirstChildElement("origin");
    while(pointdom)
    {
      points.push_back(tfFromOriginDom(*pointdom));
      pointdom = pointdom->NextSiblingElement("origin");
    }
    try_push_surface<GripperSurface>(robot, surfaces, name, bodyName, X_b_s, points, X_b_motor, motorMaxTorque);
  }
}

} // namespace details

std::vector<std::shared_ptr<Surface>> readRSDFFromDir(Robot & robot, std::string_view dirname)
{
  std::vector<std::shared_ptr<Surface>> res;

  bfs::path p{std::string{dirname}};

  if(bfs::exists(p) && bfs::is_directory(p))
  {
    std::vector<bfs::path> files;
    std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
    for(const bfs::path & file : files)
    {
      if(file.extension() == ".rsdf")
      {
        std::ifstream ifs(file.string());
        std::stringstream ss;
        ss << ifs.rdbuf();
        details::readRSDF(robot, ss.str(), res);
      }
    }
  }

  return res;
}

std::vector<sva::PTransformd> intersection(const Surface & s1, const Surface & s2)
{
  if(s1.type() == "gripper")
  {
    return s1.points();
  }
  if(s1.type() != "planar" || (s2.type() != "planar" && s2.type() != "cylindrical"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Incompatible surfaces for computing intersection: {} (type: {}) and {} (type: {})", s1, s1.type(), s2,
        s2.type());
  }
  const auto & s1Points2D = reinterpret_cast<const PlanarSurface &>(s1).planarPoints();

  // Project s2 points in s1 surface frame
  std::vector<std::pair<double, double>> s2Points2D;
  auto X_s1_s2 = s2.frame().position() * s1.frame().position().inv();
  auto X_s2_s1 = X_s1_s2.inv();
  const auto & s1T = s1.X_b_s().rotation().row(0).transpose();
  const auto & s1B = s1.X_b_s().rotation().row(1).transpose();
  for(const auto & p : s2.points())
  {
    auto X_s1_p = p * s2.X_b_s().inv() * X_s2_s1;
    s2Points2D.emplace_back(s1T.dot(X_s1_p.translation()), s1B.dot(X_s1_p.translation()));
  }

  const geos::geom::GeometryFactory * factory_ptr = geos::geom::GeometryFactory::getDefaultInstance();
  const geos::geom::GeometryFactory & factory = *factory_ptr;

  // This helper creates a geos::geom::Polygon from a set of 2D points
  auto createPolygon = [&](const std::vector<std::pair<double, double>> & points) {
    auto seq = factory.getCoordinateSequenceFactory()->create(static_cast<size_t>(0));
    std::vector<geos::geom::Coordinate> gpoints;
    gpoints.reserve(points.size() + 1);
    for(const auto & p : points)
    {
      gpoints.emplace_back(p.first, p.second);
    }
    gpoints.emplace_back(points.back().first, points.back().second);
    seq->setPoints(gpoints);
    auto shell = factory.createLinearRing(std::move(seq));
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
    auto poly = factory.createPolygon(std::move(shell));
#else
    auto poly = factory.createPolygon(std::move(shell), nullptr);
#endif
    return std::move(poly);
  };

  // Compute the intersection
  auto s1Poly = createPolygon(s1Points2D);
  auto s2Poly = createPolygon(s2Points2D);
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
  auto intersectionGeom = s1Poly->intersection(s2Poly.get());
  auto intersectionPoly = dynamic_cast<geos::geom::Polygon *>(intersectionGeom.get());
#else
  auto intersectionGeom = s1Poly->intersection(s2Poly);
  auto intersectionPoly = dynamic_cast<geos::geom::Polygon *>(intersectionGeom);
#endif

  if(intersectionPoly == nullptr)
  {
    mc_rtc::log::info("{} and {} surfaces don't intersect");
    return s1.points();
  }

  std::vector<sva::PTransformd> res;
  auto intersectionPoints = intersectionPoly->getExteriorRing()->getCoordinates();
  for(size_t i = 0; i < intersectionPoints->getSize() - 1; ++i)
  {
    const auto & p = intersectionPoints->getAt(i);
    res.push_back(sva::PTransformd(Eigen::Vector3d(p.x, p.y, 0)) * s1.X_b_s());
  }
  return res;
}

} // namespace mc_rbdyn
