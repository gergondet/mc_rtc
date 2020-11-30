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

/** Force should display a force vector in 3D environment
 *
 * The display style of the element can be configured \see ForceConfig
 *
 * Additionally an ArrayLabel with labels {"cx", "cy", "cz", "fx", "fy", "fz"} is created
 *
 * \tparam GetForce Should return an sva::ForceVecd
 *
 * \tparam GetSurface Should return an sva::PTransformd where the force will be displayed
 */
template<typename GetForce, typename GetSurface, typename GetConfig = void>
struct ForceROImpl : public Element
{
  static constexpr auto type = Elements::Force;

  ForceROImpl(const std::string & name,
              ForceConfigCallback<GetConfig> config,
              GetForce get_force_fn,
              GetSurface get_surface_fn)
  : Element(name), get_force_fn_(get_force_fn), get_surface_fn_(get_surface_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetForce, sva::ForceVecd>::value,
                  "Force element force callback must return an sva::ForceVecd");
    static_assert(details::CheckReturnType<GetSurface, sva::PTransformd>::value,
                  "Force element surface callback must return an sva::PTransformd");
  }

  static constexpr size_t write_size()
  {
    return Element::write_size() + 3 + ForceConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder, bool ro = true)
  {
    Element::write(builder);
    builder.write(get_force_fn_());
    builder.write(get_surface_fn_());
    builder.write(ro);
    config_.write(builder);
  }

private:
  GetForce get_force_fn_;
  GetSurface get_surface_fn_;
  ForceConfigCallback<GetConfig> config_;
};

template<typename GetForce, typename GetSurface, typename SetForce, typename GetConfig = void>
struct ForceImpl : public ForceROImpl<GetForce, GetSurface>
{
  static constexpr auto type = Elements::Force;
  using ForceRO = ForceROImpl<GetForce, GetSurface, GetConfig>;

  ForceImpl(const std::string & name,
            ForceConfigCallback<GetConfig> config,
            GetForce get_force_fn,
            SetForce set_force_fn,
            GetSurface get_surface_fn)
  : ForceRO(name, config, get_force_fn, get_surface_fn), set_force_fn_(set_force_fn)
  {
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    ForceRO::write(builder, false);
  }

  bool handleRequest(const mc_rtc::Configuration & data)
  {
    set_force_fn_(data);
    return true;
  }

private:
  SetForce set_force_fn_;
};

} // namespace details

/** Helper function to get a details::ForceROImpl */
template<typename GetForce, typename GetSurface>
details::ForceROImpl<GetForce, GetSurface> Force(const std::string & name,
                                                 GetForce get_force_fn,
                                                 GetSurface get_surface_fn)
{
  return details::ForceROImpl<GetForce, GetSurface>(name, {}, get_force_fn, get_surface_fn);
}

/** Helper function to get a details::ForceROImpl */
template<typename GetForce, typename GetSurface>
details::ForceROImpl<GetForce, GetSurface> Force(const std::string & name,
                                                 const ForceConfig & config,
                                                 GetForce get_force_fn,
                                                 GetSurface get_surface_fn)
{
  return details::ForceROImpl<GetForce, GetSurface>(name, config, get_force_fn, get_surface_fn);
}

/** Helper function to get a details::ForceROImpl */
template<typename GetForce, typename GetSurface, typename GetConfig>
details::ForceROImpl<GetForce, GetSurface> Force(const std::string & name,
                                                 GetConfig get_config_fn,
                                                 GetForce get_force_fn,
                                                 GetSurface get_surface_fn)
{
  return details::ForceROImpl<GetForce, GetSurface, GetConfig>(name, get_config_fn, get_force_fn, get_surface_fn);
}

/** Helper function to get a details::ForceImpl */
template<typename GetForce, typename GetSurface, typename SetForce>
details::ForceImpl<GetForce, GetSurface, SetForce> Force(const std::string & name,
                                                         GetForce get_force_fn,
                                                         SetForce set_force_fn,
                                                         GetSurface get_surface_fn)
{
  return details::ForceImpl<GetForce, GetSurface, SetForce>(name, ForceConfig{}, get_force_fn, set_force_fn,
                                                            get_surface_fn);
}

/** Helper function to get a details::ForceImpl */
template<typename GetForce, typename GetSurface, typename SetForce>
details::ForceImpl<GetForce, GetSurface, SetForce> Force(const std::string & name,
                                                         const ForceConfig & config,
                                                         GetForce get_force_fn,
                                                         SetForce set_force_fn,
                                                         GetSurface get_surface_fn)
{
  return details::ForceImpl<GetForce, GetSurface, SetForce>(name, config, get_force_fn, set_force_fn, get_surface_fn);
}

/** Helper function to get a details::ForceImpl */
template<typename GetForce, typename GetSurface, typename SetForce, typename GetConfig>
details::ForceImpl<GetForce, GetSurface, SetForce> Force(const std::string & name,
                                                         GetConfig get_config_fn,
                                                         GetForce get_force_fn,
                                                         SetForce set_force_fn,
                                                         GetSurface get_surface_fn)
{
  return details::ForceImpl<GetForce, GetSurface, SetForce, GetConfig>(name, get_config_fn, get_force_fn, set_force_fn,
                                                                       get_surface_fn);
}

} // namespace gui

} // namespace mc_rtc
