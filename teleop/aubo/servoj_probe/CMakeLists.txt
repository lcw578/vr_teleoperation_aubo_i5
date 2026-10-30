cmake_minimum_required(VERSION 3.18)
project(aubo_servoj_probe LANGUAGES CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

set(AUBO_ARCS_SDK_ROOT "" CACHE PATH "Standalone AUBO ARCS SDK 0.24 root")
if(NOT EXISTS "${AUBO_ARCS_SDK_ROOT}/include/aubo_sdk/rpc.h" OR
   NOT EXISTS "${AUBO_ARCS_SDK_ROOT}/lib/libaubo_sdk.so")
  message(FATAL_ERROR "Specify an extracted AUBO ARCS SDK root with -DAUBO_ARCS_SDK_ROOT=...")
endif()

pybind11_add_module(_aubo_servoj_probe probe.cpp)
target_compile_features(_aubo_servoj_probe PRIVATE cxx_std_17)
target_include_directories(_aubo_servoj_probe PRIVATE "${AUBO_ARCS_SDK_ROOT}/include")
target_link_libraries(_aubo_servoj_probe PRIVATE "${AUBO_ARCS_SDK_ROOT}/lib/libaubo_sdk.so")
set_target_properties(_aubo_servoj_probe PROPERTIES
  BUILD_RPATH "${AUBO_ARCS_SDK_ROOT}/lib"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python")
