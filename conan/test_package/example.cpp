#include <mc_rtc/version.h>

#include <iostream>

int main()
{
  std::cout << "mc_rtc compile time version: " << mc_rtc::MC_RTC_VERSION << "\n";
  std::cout << "mc_rtc run time version: " << mc_rtc::version() << "\n";
  return 0;
}
