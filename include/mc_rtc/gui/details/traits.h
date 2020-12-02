/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/** This file contains some utility template metaprogramming functions for the GUI */

#include <type_traits>

namespace mc_rtc
{

namespace gui
{

namespace details
{

/** Detect a callable type
 *
 * Taken from https://stackoverflow.com/questions/15393938/find-out-whether-a-c-object-is-callable
 *
 * FIXME To be replaced with std::is_invocable in C++17
 */
template<typename T>
struct is_callable
{
private:
  typedef char (&yes)[1];
  typedef char (&no)[2];

  struct Fallback
  {
    void operator()();
  };
  struct Derived : T, Fallback
  {
  };

  template<typename U, U>
  struct Check;

  template<typename>
  static yes test(...);

  template<typename C>
  static no test(Check<void (Fallback::*)(), &C::operator()> *);

public:
  static const bool value = sizeof(test<Derived>(0)) == sizeof(yes);
};

/** Tag for non-callable objects */
struct NonCallable
{
};

template<typename GetT, typename Enable = void>
struct ReturnType
{
  using type = NonCallable;
};

/** Get the return type of a getter function */
template<typename GetT>
struct ReturnType<GetT, typename std::enable_if<is_callable<GetT>::value>::type>
{
  using type = typename std::decay<decltype(std::declval<GetT>()())>::type;
};

/** Helper */
template<typename GetT>
using ReturnTypeT = typename ReturnType<GetT>::type;

/** Check the return type of a getter function
 *
 * value is true if GetT() returns one of the provided argument types
 */
template<typename GetT, typename... Args>
struct CheckReturnType
{
  static constexpr bool value = false;
};

template<typename GetT, typename T>
struct CheckReturnType<GetT, T>
{
  static constexpr bool value = std::is_convertible<ReturnTypeT<GetT>, typename std::decay<T>::type>::value;
};

template<typename GetT, typename T, typename... Args>
struct CheckReturnType<GetT, T, Args...>
{
  static constexpr bool value = CheckReturnType<GetT, T>::value || CheckReturnType<GetT, Args...>::value;
};

} // namespace details

} // namespace gui

} // namespace mc_rtc
