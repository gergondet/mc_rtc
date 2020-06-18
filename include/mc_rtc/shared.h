/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>

namespace mc_rtc
{

/** This template helper allow to enable implicit conversion from instances of
 * T (resp. const T) to std::shared_ptr<T> (resp. std::shared_ptr<const T>).
 *
 * This allows mc_rtc to expose reference (or const reference) to internal
 * objects stored as pointer which are often manipulated without requiring
 * ownership or access a "nested" object without incurring extra pointer copies
 * (e.g. accessing a frame of a robot).
 */
template<typename T>
struct shared : public std::enable_shared_from_this<T>
{
  operator std::shared_ptr<T>()
  {
    return static_cast<T *>(this)->shared_from_this();
  }

  operator std::shared_ptr<const T>() const
  {
    return static_cast<const T *>(this)->shared_from_this();
  }
};

} // namespace mc_rtc
