/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>

#ifndef MC_RTC_BUILD_STATIC

#  include <mc_rtc/version.h>

/** Set of macros to assist with the writing of an MCController */

/** A simple compile-time versus run-time version checking macro
 *
 * If you are not relying on CONTROLLER_CONSTRUCTOR or
 * SIMPLE_CONTROLLER_CONSTRUCTOR you should use this in your MC_RTC_CONTROLLER
 * implementation
 *
 */
#  define CONTROLLER_CHECK_VERSION(NAME)                                                                               \
    if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())                                                                    \
    {                                                                                                                  \
      mc_rtc::log::error("{} was compiled with {} but mc_rtc is currently at version {}, you might experience subtle " \
                         "issues and should recompile your code",                                                      \
                         NAME, mc_rtc::MC_RTC_VERSION, mc_rtc::version());                                             \
    }

/** Provides a handle to construct the controller with Json config */
#  define CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                                        \
    extern "C"                                                                                                      \
    {                                                                                                               \
      CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)                                \
      {                                                                                                             \
        CONTROLLER_CHECK_VERSION(NAME)                                                                              \
        names = {NAME};                                                                                             \
      }                                                                                                             \
      CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)                                            \
      {                                                                                                             \
        delete ptr;                                                                                                 \
      }                                                                                                             \
      CONTROLLER_MODULE_API unsigned int create_args_required()                                                     \
      {                                                                                                             \
        return 4;                                                                                                   \
      }                                                                                                             \
      CONTROLLER_MODULE_API mc_control::MCController * create(const std::string &,                                  \
                                                              const std::shared_ptr<mc_rbdyn::RobotModule> & robot, \
                                                              const double & dt,                                    \
                                                              const mc_rtc::Configuration & conf)                   \
      {                                                                                                             \
        return new TYPE(robot, dt, conf);                                                                           \
      }                                                                                                             \
    }

/** Provides a handle to construct a generic controller */
#  define SIMPLE_CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                                 \
    extern "C"                                                                                                      \
    {                                                                                                               \
      CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)                                \
      {                                                                                                             \
        CONTROLLER_CHECK_VERSION(NAME)                                                                              \
        names = {NAME};                                                                                             \
      }                                                                                                             \
      CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)                                            \
      {                                                                                                             \
        delete ptr;                                                                                                 \
      }                                                                                                             \
      CONTROLLER_MODULE_API unsigned int create_args_required()                                                     \
      {                                                                                                             \
        return 4;                                                                                                   \
      }                                                                                                             \
      CONTROLLER_MODULE_API mc_control::MCController * create(const std::string &,                                  \
                                                              const std::shared_ptr<mc_rbdyn::RobotModule> & robot, \
                                                              const double & dt,                                    \
                                                              const mc_rtc::Configuration &)                        \
      {                                                                                                             \
        return new TYPE(robot, dt);                                                                                 \
      }                                                                                                             \
    }

#else

#  include <mc_control/ControllerLoader.h>

#  define CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                            \
    namespace                                                                                           \
    {                                                                                                   \
    static auto registered = []() {                                                                     \
      using fn_t = std::function<TYPE *(const std::shared_ptr<mc_rbdyn::RobotModule> &, const double &, \
                                        const mc_rtc::Configuration &)>;                                \
      mc_control::ControllerLoader::loader().register_object(                                           \
          NAME, fn_t([](const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt,        \
                        const mc_rtc::Configuration & conf) { return new TYPE(robot, dt, conf); }));    \
      return true;                                                                                      \
    }();                                                                                                \
    }

#  define SIMPLE_CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                     \
    namespace                                                                                           \
    {                                                                                                   \
    static auto registered = []() {                                                                     \
      using fn_t = std::function<TYPE *(const std::shared_ptr<mc_rbdyn::RobotModule> &, const double &, \
                                        const mc_rtc::Configuration &)>;                                \
      mc_control::ControllerLoader::loader().register_object(                                           \
          NAME, fn_t([](const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt,        \
                        const mc_rtc::Configuration &) { return new TYPE(robot, dt); }));               \
      return true;                                                                                      \
    }();                                                                                                \
    }

#endif
