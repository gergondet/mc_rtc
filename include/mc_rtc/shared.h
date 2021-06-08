/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>
#include <type_traits>

namespace mc_rtc
{

/** This template helper allow to enable implicit conversion from instances of
 * T (resp. const T) to std::shared_ptr<T> (resp. std::shared_ptr<const T>).
 *
 * This allows mc_rtc to expose reference (or const reference) to internal
 * objects stored as pointer which are often manipulated without requiring
 * ownership or access a "nested" object without incurring extra pointer copies
 * (e.g. accessing a frame of a robot).
 *
 * \tparam Conditionally enable inheritance from std::enable_shared_from_this, useful in rnheritance scenario
 */
template<typename T, bool EnableSharedFromThis = true>
struct shared;

template<typename T>
struct shared<T, false>
{
  operator std::shared_ptr<T>()
  {
    return std::static_pointer_cast<T>(static_cast<T *>(this)->shared_from_this());
  }

  operator std::shared_ptr<const T>() const
  {
    return std::static_pointer_cast<const T>(static_cast<const T *>(this)->shared_from_this());
  }
};

template<typename T>
struct shared<T, true> : public std::enable_shared_from_this<T>, shared<T, false>
{
};

} // namespace mc_rtc
