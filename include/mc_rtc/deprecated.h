/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/logging.h>

#include <optional>

/** Helper functions to emit deprecation messages when loading JSON/YAML from old versions */

namespace mc_rtc
{

namespace log
{

namespace internal
{

struct quoted : public fmt::string_view
{
  using fmt::string_view::string_view;
};

} // namespace internal

/** Used when \p replace should be used instead of \p old */
MC_RTC_UTILS_DLLAPI void deprecated(std::string_view source,
                                    std::string_view old,
                                    std::string_view replace,
                                    std::optional<std::string_view> extra = std::nullopt);

/** Used when multiple \p replace should be used instead of \p old */
MC_RTC_UTILS_DLLAPI void deprecated(std::string_view source,
                                    std::string_view old,
                                    std::initializer_list<internal::quoted> replace,
                                    std::optional<std::string_view> extra = std::nullopt);

/** Used when a key (\p missing) is missing */
MC_RTC_UTILS_DLLAPI void missing(std::string_view source,
                                 std::string_view missing,
                                 std::optional<std::string_view> extra = std::nullopt);

} // namespace log

} // namespace mc_rtc
