/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

namespace mc_rtc
{

namespace gui
{

namespace details
{

/** Point3D should display a 3D point in the environment
 *
 * A PointConfig is provided to control how the point should be displayed
 *
 * With this variant, the point cannot be edited
 *
 * It will also trigger an ArrayLabel with {"x", "y", "z"} labels
 *
 * \tparam GetT Should return an Eigen::Vector3d
 */
template<typename GetT,
         typename GetConfig = void,
         typename std::enable_if<details::CheckReturnType<GetT, Eigen::Vector3d>::value, bool>::type = true>
struct Point3DROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DROImpl(const std::string & name, PointConfigCallback<GetConfig> config, GetT get_fn)
  : DataElement<GetT>(name, get_fn), config_(config)
  {
  }

  static constexpr size_t write_size()
  {
    return DataElement<GetT>::write_size() + 1 + PointConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    builder.write(true); // Read-only
    config_.write(builder);
  }

  /** Invalid element */
  Point3DROImpl() {}

private:
  PointConfigCallback<GetConfig> config_;
};

/** Point3D should display a 3D point in the environment
 *
 * A PointConfig is provided to control how the point is displayed
 *
 * With this variant, the point can be edited
 *
 * It will also trigger an ArrayInput with {"x", "y", "z"} labels
 *
 * \tparam GetT Should return an Eigen::Vector3d
 *
 * \tparam SetT Will be called when the point is moved or the ArrayInput is triggered
 */
template<typename GetT,
         typename SetT,
         typename GetConfig = void,
         typename std::enable_if<details::CheckReturnType<GetT, Eigen::Vector3d>::value, bool>::type = true>
struct Point3DImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DImpl(const std::string & name, PointConfigCallback<GetConfig> config, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), config_(config)
  {
  }

  /** Invalid element */
  Point3DImpl() {}

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1 + PointConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    builder.write(false); // Not read-only
    config_.write(builder);
  }

private:
  PointConfigCallback<GetConfig> config_;
};

} // namespace details

/** Helper function to create a details::Point3DROImpl */
template<typename GetT>
details::Point3DROImpl<GetT> Point3D(const std::string & name, GetT get_fn)
{
  return details::Point3DROImpl<GetT>(name, {}, get_fn);
}

/** Helper function to create a details::Point3DImpl */
template<typename GetT, typename SetT>
details::Point3DImpl<GetT, SetT> Point3D(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::Point3DImpl<GetT, SetT>(name, {}, get_fn, set_fn);
}

/** Helper function to create a details::Point3DROImpl with configuration */
template<typename GetT>
details::Point3DROImpl<GetT> Point3D(const std::string & name, const PointConfig & config, GetT get_fn)
{
  return details::Point3DROImpl<GetT>(name, config, get_fn);
}

/** Helper function to create a details::Point3DImpl with configuration */
template<typename GetT, typename SetT>
details::Point3DImpl<GetT, SetT> Point3D(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn)
{
  return details::Point3DImpl<GetT, SetT>(name, config, get_fn, set_fn);
}

/** Helper function to create a details::Point3DROImpl with configuration callback */
template<typename GetT, typename GetConfig>
details::Point3DROImpl<GetT, GetConfig> Point3D(const std::string & name, GetConfig config, GetT get_fn)
{
  return details::Point3DROImpl<GetT, GetConfig>(name, config, get_fn);
}

/** Helper function to create a details::Point3DImpl with configuration callback */
template<typename GetT, typename SetT, typename GetConfig>
details::Point3DImpl<GetT, SetT, GetConfig> Point3D(const std::string & name,
                                                    GetConfig config,
                                                    GetT get_fn,
                                                    SetT set_fn)
{
  return details::Point3DImpl<GetT, SetT, GetConfig>(name, config, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
