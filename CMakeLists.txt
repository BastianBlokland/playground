# --------------------------------------------------------------------------------------------------
# Root CMake file for the Playground project.
# --------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

if(${CMAKE_VERSION} VERSION_GREATER "3.20")
  cmake_policy(SET CMP0116 NEW) # Support Ninja dep-files in subdirectories.
endif()

project(Playground
  VERSION 0.1.0
  DESCRIPTION "Playground"
  LANGUAGES C
  )

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Supported configuration types" FORCE)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Custom options.
option(VOLO_SIMD                  "SIMD support"                    On)
option(VOLO_TRACE                 "Runtime performance tracing"     On)
option(VOLO_LTO                   "Link time optimization"          Off)
option(VOLO_SANITIZE              "Sanitizer instrumentation"       Off)
option(VOLO_WERROR                "Warnings as errors"              Off)
set(   VOLO_LABEL "" CACHE STRING "Label to be associated with the build")

# Diagnostic information.
message(STATUS "Host system: ${CMAKE_HOST_SYSTEM}")
message(STATUS "Host processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMake version: ${CMAKE_VERSION}")
message(STATUS "Project version: ${PROJECT_VERSION}")
message(STATUS "Source path: ${PROJECT_SOURCE_DIR}")
message(STATUS "Build path: ${PROJECT_BINARY_DIR}")
message(STATUS "Compiler: ${CMAKE_C_COMPILER_ID}")
message(STATUS "Generator: ${CMAKE_GENERATOR}")
message(STATUS "Version: ${CMAKE_PROJECT_VERSION}")
message(STATUS "Label: '${VOLO_LABEL}'")
message(STATUS "Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Simd: ${VOLO_SIMD}")
message(STATUS "Trace: ${VOLO_TRACE}")
message(STATUS "Lto: ${VOLO_LTO}")
message(STATUS "Sanitize: ${VOLO_SANITIZE}")

# --------------------------------------------------------------------------------------------------
# Global setup.
# --------------------------------------------------------------------------------------------------

include(cmake/compiler.cmake)

# --------------------------------------------------------------------------------------------------
# Setup targets.
# --------------------------------------------------------------------------------------------------

add_subdirectory(apps)
add_subdirectory(libs/app)
add_subdirectory(libs/asset)
add_subdirectory(libs/check)
add_subdirectory(libs/cli)
add_subdirectory(libs/core)
add_subdirectory(libs/data)
add_subdirectory(libs/dev)
add_subdirectory(libs/ecs)
add_subdirectory(libs/gap)
add_subdirectory(libs/geo)
add_subdirectory(libs/input)
add_subdirectory(libs/jobs)
add_subdirectory(libs/json)
add_subdirectory(libs/loc)
add_subdirectory(libs/log)
add_subdirectory(libs/net)
add_subdirectory(libs/rend)
add_subdirectory(libs/script)
add_subdirectory(libs/snd)
add_subdirectory(libs/trace)
add_subdirectory(libs/ui)
add_subdirectory(libs/xml)

# --------------------------------------------------------------------------------------------------
# Testing.
# --------------------------------------------------------------------------------------------------

add_custom_target(test.asset  COMMAND asset_test VERBATIM USES_TERMINAL)
add_custom_target(test.check  COMMAND check_test VERBATIM USES_TERMINAL)
add_custom_target(test.cli    COMMAND cli_test VERBATIM USES_TERMINAL)
add_custom_target(test.core   COMMAND core_test VERBATIM USES_TERMINAL)
add_custom_target(test.data   COMMAND data_test VERBATIM USES_TERMINAL)
add_custom_target(test.ecs    COMMAND ecs_test VERBATIM USES_TERMINAL)
add_custom_target(test.geo    COMMAND geo_test VERBATIM USES_TERMINAL)
add_custom_target(test.jobs   COMMAND jobs_test VERBATIM USES_TERMINAL)
add_custom_target(test.json   COMMAND json_test VERBATIM USES_TERMINAL)
add_custom_target(test.log    COMMAND log_test VERBATIM USES_TERMINAL)
add_custom_target(test.net    COMMAND net_test VERBATIM USES_TERMINAL)
add_custom_target(test.script COMMAND script_test VERBATIM USES_TERMINAL)
add_custom_target(test.trace  COMMAND trace_test VERBATIM USES_TERMINAL)
add_custom_target(test.xml    COMMAND xml_test VERBATIM USES_TERMINAL)

add_custom_target(test DEPENDS
  test.asset
  test.check
  test.cli
  test.core
  test.data
  test.ecs
  test.geo
  test.jobs
  test.json
  test.log
  test.net
  test.script
  test.trace
  test.xml
  )

# --------------------------------------------------------------------------------------------------
# Install.
# --------------------------------------------------------------------------------------------------

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}") # Install to project directory itself.

install(TARGETS smoke DESTINATION bin)

if(${VOLO_PLATFORM} STREQUAL "win32" AND NOT ${VOLO_COMPILER} STREQUAL "gcc")
  install(FILES
    $<TARGET_PDB_FILE:smoke>
    DESTINATION bin)
endif()

# --------------------------------------------------------------------------------------------------
# Helpers.
# --------------------------------------------------------------------------------------------------

add_custom_target(run.smoke
  COMMAND smoke "--assets" "${CMAKE_SOURCE_DIR}/assets"  VERBATIM USES_TERMINAL)
