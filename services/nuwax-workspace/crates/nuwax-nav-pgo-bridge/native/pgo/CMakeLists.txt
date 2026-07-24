cmake_minimum_required(VERSION 3.14)
project(nuwax_pgo_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(pgo main.cpp simple_pgo.cpp commons.cpp)
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
