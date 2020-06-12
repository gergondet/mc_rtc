/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/robin_hood/robin_hood.h>

namespace mc_rtc
{

template<typename Key,
         typename T,
         typename Hash = robin_hood::hash<Key>,
         typename KeyEqual = std::equal_to<Key>,
         size_t MaxLoadFactor100 = 80>
using map = robin_hood::unordered_flat_map<Key, T, Hash, KeyEqual, MaxLoadFactor100>;

template<typename Key,
         typename Hash = robin_hood::hash<Key>,
         typename KeyEqual = std::equal_to<Key>,
         size_t MaxLoadFactor100 = 80>
using set = robin_hood::unordered_flat_set<Key, Hash, KeyEqual, MaxLoadFactor100>;

} // namespace mc_rtc
