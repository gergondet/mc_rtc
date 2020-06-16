/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/robin_hood/robin_hood.h>

namespace mc_rtc
{

namespace details
{

template<typename T>
struct hash : public robin_hood::hash<T>
{
};

template<>
struct hash<std::string>
{
  using is_transparent = void;

  inline size_t operator()(const std::string_view & str) const noexcept
  {
    return robin_hood::hash_bytes(str.data(), str.size());
  }
};

template<typename T>
struct equal_to : public std::equal_to<T>
{
};

template<>
struct equal_to<std::string>
{
  using is_transparent = void;

  inline bool operator()(const std::string_view & lhs, const std::string & rhs) const noexcept
  {
    return lhs == rhs;
  }
};

} // namespace details

template<typename Key,
         typename T,
         typename Hash = details::hash<Key>,
         typename KeyEqual = details::equal_to<Key>,
         size_t MaxLoadFactor100 = 80>
using map = robin_hood::unordered_flat_map<Key, T, Hash, KeyEqual, MaxLoadFactor100>;

template<typename Key,
         typename Hash = robin_hood::hash<Key>,
         typename KeyEqual = std::equal_to<Key>,
         size_t MaxLoadFactor100 = 80>
using set = robin_hood::unordered_flat_set<Key, Hash, KeyEqual, MaxLoadFactor100>;

} // namespace mc_rtc
