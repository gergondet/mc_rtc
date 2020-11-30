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

/** Stores:
 * - a callback that returns an ArrowConfig object if GetConfig is not void
 * - a default ArrowConfig value otherwise
 */
template<typename GetConfig>
struct ArrowConfigCallback
{
  ArrowConfigCallback(GetConfig get_config_fn) : get_config_fn_(get_config_fn)
  {
    static_assert(details::CheckReturnType<GetConfig, ArrowConfig>::value,
                  "ArrowConfig callback must return an ArrowConfig");
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    get_config_fn_().write(builder);
  }

private:
  GetConfig get_config_fn_;
};

template<>
struct ArrowConfigCallback<void>
{
  ArrowConfigCallback() = default;

  ArrowConfigCallback(const ArrowConfig & config) : config_(config) {}

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    config_.write(builder);
  }

private:
  ArrowConfig config_;
};

/** Arrow should display an arrow from the point at the start to the point at the end
 *
 * An ArrowConfig can be provided to specify how the arrow should be displayed
 *
 * \tparam GetStart Returns an Eigen::Vector3d representing the starting point
 *
 * \tparam GetEnd Returns an Eigen::Vector3d representing the end point
 *
 * \tparam GetConfig Returns an ArrowConfig if non-void
 *
 */
template<typename GetStart, typename GetEnd, typename GetConfig = void>
struct ArrowROImpl : public Element
{
  static constexpr auto type = Elements::Arrow;

  ArrowROImpl(const std::string & name, ArrowConfigCallback<GetConfig> config, GetStart get_start_fn, GetEnd get_end_fn)
  : Element(name), get_start_fn_(get_start_fn), get_end_fn_(get_end_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetStart, Eigen::Vector3d>::value,
                  "Arrow element start callback must return an Eigen::Vector3d");
    static_assert(details::CheckReturnType<GetEnd, Eigen::Vector3d>::value,
                  "Arrow element end callback must return an Eigen::Vector3d");
  }

  /** Invalid element */
  ArrowROImpl(){};

  constexpr static size_t write_size()
  {
    return Element::write_size() + 3 + ArrowConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder, bool ro = true)
  {
    Element::write(builder);
    builder.write(get_start_fn_());
    builder.write(get_end_fn_());
    builder.write(ro);
    config_.write(builder);
  }

private:
  GetStart get_start_fn_;
  GetEnd get_end_fn_;
  ArrowConfigCallback<GetConfig> config_;
};

template<typename GetStart, typename SetStart, typename GetEnd, typename SetEnd, typename GetConfig = void>
struct ArrowImpl : public ArrowROImpl<GetStart, GetEnd, GetConfig>
{
  ArrowImpl(const std::string & name,
            ArrowConfigCallback<GetConfig> config,
            GetStart get_start_fn,
            SetStart set_start_fn,
            GetEnd get_end_fn,
            SetEnd set_end_fn)
  : ArrowROImpl<GetStart, GetEnd, GetConfig>(name, config, get_start_fn, get_end_fn), set_start_fn_(set_start_fn),
    set_end_fn_(set_end_fn)
  {
  }

  /** Invalid element */
  ArrowImpl(){};

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    ArrowROImpl<GetStart, GetEnd, GetConfig>::write(builder, false);
  }

  bool handleRequest(const mc_rtc::Configuration & data)
  {
    const Eigen::Vector6d & arrow = data;
    set_start_fn_(arrow.head<3>());
    set_end_fn_(arrow.tail<3>());
    return true;
  }

private:
  SetStart set_start_fn_;
  SetEnd set_end_fn_;
};

} // namespace details

/** Helper function to create a details::ArrowROImpl */
template<typename GetStart, typename GetEnd>
details::ArrowROImpl<GetStart, GetEnd> Arrow(const std::string & name, GetStart get_start_fn, GetEnd get_end_fn)
{
  return details::ArrowROImpl<GetStart, GetEnd>(name, {}, get_start_fn, get_end_fn);
}

/** Helper function to create a details::ArrowROImpl */
template<typename GetStart, typename GetEnd>
details::ArrowROImpl<GetStart, GetEnd> Arrow(const std::string & name,
                                             const ArrowConfig & config,
                                             GetStart get_start_fn,
                                             GetEnd get_end_fn)
{
  return details::ArrowROImpl<GetStart, GetEnd>(name, config, get_start_fn, get_end_fn);
}

/** Helper function to create a details::ArrowROImpl */
template<typename GetStart, typename GetEnd, typename GetConfig>
details::ArrowROImpl<GetStart, GetEnd, GetConfig> Arrow(const std::string & name,
                                                        GetConfig get_config_fn,
                                                        GetStart get_start_fn,
                                                        GetEnd get_end_fn)
{
  return details::ArrowROImpl<GetStart, GetEnd, GetConfig>(name, get_config_fn, get_start_fn, get_end_fn);
}

/** Helper function to create a details::ArrowImpl */
template<typename GetStart, typename SetStart, typename GetEnd, typename SetEnd>
details::ArrowImpl<GetStart, SetStart, GetEnd, SetEnd> Arrow(const std::string & name,
                                                             GetStart get_start_fn,
                                                             SetStart set_start_fn,
                                                             GetEnd get_end_fn,
                                                             SetEnd set_end_fn)
{
  return details::ArrowImpl<GetStart, SetStart, GetEnd, SetEnd>(name, ArrowConfig{}, get_start_fn, set_start_fn,
                                                                get_end_fn, set_end_fn);
}

/** Helper function to create a details::ArrowImpl */
template<typename GetStart, typename SetStart, typename GetEnd, typename SetEnd>
details::ArrowImpl<GetStart, SetStart, GetEnd, SetEnd> Arrow(const std::string & name,
                                                             const ArrowConfig & config,
                                                             GetStart get_start_fn,
                                                             SetStart set_start_fn,
                                                             GetEnd get_end_fn,
                                                             SetEnd set_end_fn)
{
  return details::ArrowImpl<GetStart, SetStart, GetEnd, SetEnd>(name, config, get_start_fn, set_start_fn, get_end_fn,
                                                                set_end_fn);
}

/** Helper function to create a details::ArrowImpl */
template<typename GetStart, typename SetStart, typename GetEnd, typename SetEnd, typename GetConfig>
details::ArrowImpl<GetStart, SetStart, GetEnd, SetEnd, GetConfig> Arrow(const std::string & name,
                                                                        GetConfig get_config_fn,
                                                                        GetStart get_start_fn,
                                                                        SetStart set_start_fn,
                                                                        GetEnd get_end_fn,
                                                                        SetEnd set_end_fn)
{
  return details::ArrowImpl<GetStart, SetStart, GetEnd, SetEnd>(name, get_config_fn, get_start_fn, set_start_fn,
                                                                get_end_fn, set_end_fn);
}

} // namespace gui

} // namespace mc_rtc
