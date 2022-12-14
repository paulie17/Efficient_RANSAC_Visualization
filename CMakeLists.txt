# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.23)

project( primitives_extraction_visualization )
set(CMAKE_BUILD_TYPE Release)

set(CGAL_DATA_DIR ".")

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_definitions(-DCGAL_USE_BASIC_VIEWER)

# include for local directory

# include for local package

# Creating entries for all C++ files with "main" routine
# ##########################################################


# create_single_source_cgal_program( "efficient_RANSAC.cpp" )
add_executable(efficient_RANSAC efficient_RANSAC.cpp)
target_link_libraries(efficient_RANSAC CGAL::CGAL ${PCL_LIBRARIES})
