cmake_minimum_required(VERSION 3.14)
project(nuwax_pgo_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(LCM REQUIRED IMPORTED_TARGET lcm)
pkg_check_modules(PCL REQUIRED IMPORTED_TARGET pcl_common pcl_filters pcl_kdtree pcl_registration)
find_program(LCM_GEN_EXECUTABLE lcm-gen)
if(NOT LCM_GEN_EXECUTABLE)
  message(FATAL_ERROR "lcm-gen is required for the native PGO typed protocol")
endif()

set(NATIVE_PGO_IDL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lcmtypes/nuwax_msgs")
set(NATIVE_PGO_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(NATIVE_PGO_IDLS
  "${NATIVE_PGO_IDL_DIR}/NativePgoRequest.lcm"
  "${NATIVE_PGO_IDL_DIR}/NativePgoResult.lcm"
  "${NATIVE_PGO_IDL_DIR}/NativePgoStatus.lcm"
)
set(NATIVE_PGO_GENERATED_HEADERS
  "${NATIVE_PGO_GENERATED_DIR}/nuwax_msgs/NativePgoRequest.hpp"
  "${NATIVE_PGO_GENERATED_DIR}/nuwax_msgs/NativePgoResult.hpp"
  "${NATIVE_PGO_GENERATED_DIR}/nuwax_msgs/NativePgoStatus.hpp"
)
add_custom_command(
  OUTPUT ${NATIVE_PGO_GENERATED_HEADERS}
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${NATIVE_PGO_GENERATED_DIR}"
  COMMAND "${LCM_GEN_EXECUTABLE}" -x --cpp-std c++11
          --cpp-hpath "${NATIVE_PGO_GENERATED_DIR}" ${NATIVE_PGO_IDLS}
  DEPENDS ${NATIVE_PGO_IDLS}
  VERBATIM
)

add_executable(
  pgo
  main.cpp
  online_owner.cpp
  simple_pgo.cpp
  commons.cpp
  ${NATIVE_PGO_GENERATED_HEADERS}
)
target_include_directories(pgo PRIVATE "${NATIVE_PGO_GENERATED_DIR}")
target_link_libraries(
  pgo
  PRIVATE PkgConfig::LCM PkgConfig::PCL OpenSSL::Crypto Threads::Threads
)
target_compile_options(pgo PRIVATE -Wall -Wextra -Wpedantic -O2)

enable_testing()
add_test(NAME pgo_help COMMAND pgo --help)
add_test(NAME pgo_unknown_arg_rejected COMMAND pgo --unknown)
add_test(NAME pgo_missing_value_rejected COMMAND pgo --batch-input)
add_test(
  NAME pgo_mixed_mode_rejected
  COMMAND pgo --world_frame map --batch-input unused-input --batch-output unused-output
)
set_tests_properties(
  pgo_unknown_arg_rejected
  pgo_missing_value_rejected
  pgo_mixed_mode_rejected
  PROPERTIES WILL_FAIL TRUE TIMEOUT 5
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp")
  add_executable(pgo_tests tests.cpp simple_pgo.cpp commons.cpp)
  target_compile_options(pgo_tests PRIVATE -Wall -Wextra -Wpedantic -O2)
  add_test(NAME pgo_non_finite_safety COMMAND pgo_tests)
  set_tests_properties(pgo_non_finite_safety PROPERTIES TIMEOUT 5)
endif()
