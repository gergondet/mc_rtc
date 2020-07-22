find_program(GIT git)
if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/.git AND GIT)
  execute_process(COMMAND ${GIT} rev-parse --short HEAD
    OUTPUT_VARIABLE GIT_REVISION
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(MC_RTC_VERSION "${PROJECT_VERSION}-${GIT_REVISION}")
else()
  set(MC_RTC_VERSION "${PROJECT_VERSION}")
endif()

set(FILE_IN "${PROJECT_SOURCE_DIR}/include/mc_rtc/version.h.in")
set(FILE_OUT "${PROJECT_BINARY_DIR}/include/mc_rtc/version.h")

configure_file("${FILE_IN}" "${FILE_OUT}")
