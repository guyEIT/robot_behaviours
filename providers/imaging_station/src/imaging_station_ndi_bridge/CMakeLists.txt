cmake_minimum_required(VERSION 3.8)
project(imaging_station_ndi_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# QUIET — the package still installs (an empty binary slot) when NDI isn't
# present yet, so `pixi install -e lite-native` succeeds before `setup-ndi`
# has run. The Python node detects the missing binary at startup and points
# the operator at the install instructions.
find_package(NDISdk QUIET)

if(NDISdk_FOUND)
  message(STATUS "NDI SDK headers: ${NDISdk_INCLUDE_DIRS} — building ndi_av_bridge")
  add_executable(ndi_av_bridge src/ndi_av_bridge.cpp)
  target_include_directories(ndi_av_bridge PRIVATE ${NDISdk_INCLUDE_DIRS})
  # libdl is needed for dlopen/dlsym; libndi itself is loaded at runtime.
  target_link_libraries(ndi_av_bridge PRIVATE ${CMAKE_DL_LIBS})

  install(TARGETS
    ndi_av_bridge
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "NDI SDK (Processing.NDI.Lib.h) not found — ndi_av_bridge will NOT be "
    "built. The package still installs so pixi solves don't fail. Install "
    "the SDK with `bash scripts/setup-ndi-host.sh`, then run "
    "`pixi run lite-native-install` again to pick up the binary."
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
