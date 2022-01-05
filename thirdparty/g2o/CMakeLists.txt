cmake_minimum_required(VERSION 3.1)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(g2o)

set(g2o_C_FLAGS)
set(g2o_CXX_FLAGS)

set(LIB_PREFIX g2o_)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

set(G2O_LIB_TYPE SHARED)

set(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})

list(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

if(UNIX)
  add_definitions(-DUNIX)
  message(STATUS "Compiling on Unix")
endif(UNIX)

set(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
if(G2O_USE_OPENMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    set (G2O_OPENMP 1)
    set(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
    message(STATUS "Compiling with OpenMP support")
  endif(OPENMP_FOUND)
endif(G2O_USE_OPENMP)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")

set(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

find_package(Eigen3 3.3 REQUIRED)
if (TARGET Eigen3::Eigen)
  set(G2O_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

include_directories(${g2o_SOURCE_DIR})

set(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in "${g2o_SOURCE_DIR}/g2o/config.h")

add_subdirectory(g2o/core)
add_subdirectory(g2o/solvers/eigen)
add_subdirectory(g2o/stuff)
add_subdirectory(g2o/types/slam3d)
