/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/** This file contains some utility template metaprogramming functions for the GUI */

#include <functional>
#include <type_traits>

namespace mc_rtc
{

namespace gui
{

namespace details
{

// clang-format off

/** Taken from https://stackoverflow.com/questions/51187974/can-stdis-invocable-be-emulated-within-c11
 *
 * Can be replaced with std::is_invocable in C++17
 */
template <typename F, typename... Args>
struct is_invocable :
    std::is_constructible<
        std::function<void(Args ...)>,
        std::reference_wrapper<typename std::remove_reference<F>::type>
    >
{
};

template<>
struct is_invocable<void> : public std::false_type {};

// clang-format on

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
struct ReturnType<GetT, typename std::enable_if<is_invocable<GetT>::value>::type>
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
