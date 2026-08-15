cmake_minimum_required(VERSION 3.20)

project(hospital_path_lab_dwa_core LANGUAGES CXX)

add_library(dwa_core SHARED dwa_core.cpp)
target_compile_features(dwa_core PRIVATE cxx_std_20)
target_compile_options(
  dwa_core
  PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/O2;/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-O3;-Wall;-Wextra;-Wpedantic>
)
set_target_properties(dwa_core PROPERTIES OUTPUT_NAME "dwa_core")

add_library(dwb_safety_core SHARED dwb_safety_core.cpp)
target_compile_features(dwb_safety_core PRIVATE cxx_std_20)
target_compile_options(
  dwb_safety_core
  PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/O2;/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-O3;-Wall;-Wextra;-Wpedantic>
)
set_target_properties(dwb_safety_core PROPERTIES OUTPUT_NAME "dwb_safety_core")
