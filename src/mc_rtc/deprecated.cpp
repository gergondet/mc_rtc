/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/deprecated.h>

namespace mc_rtc::log
{

void deprecated(std::string_view source,
                std::string_view old,
                std::string_view replace,
                std::optional<std::string_view> extra)
{
  mc_rtc::log::warning("[MC_RTC_DEPRECATED][{}] Use of \"{}\" is deprecated, use \"{}\" instead{}", source, old,
                       replace, extra);
}

void deprecated(std::string_view source,
                std::string_view old,
                std::initializer_list<internal::quoted> replace,
                std::optional<std::string_view> extra)
{
  mc_rtc::log::warning("[MC_RTC_DEPRECATED][{}] Use of \"{}\" is deprecated, use {} instead{}", source, old,
                       fmt::join(replace, " and "), extra);
}

} // namespace mc_rtc::log

template<>
struct fmt::formatter<mc_rtc::log::internal::quoted> : public fmt::formatter<fmt::string_view>
{
  template<typename FormatContext>
  auto format(mc_rtc::log::internal::quoted q, FormatContext & ctx)
  {
    return format_to(ctx.out(), "\"{}\"", static_cast<fmt::string_view &>(q));
  }
};

template<>
struct fmt::formatter<std::optional<std::string_view>> : public fmt::formatter<std::string_view>
{
  template<typename FormatContext>
  auto format(const std::optional<std::string_view> & sv, FormatContext & ctx)
  {
    if(sv.has_value())
    {
      return format_to(ctx.out(), ". {}", sv.value());
    }
    else
    {
      return ctx.out();
    }
  }
};
