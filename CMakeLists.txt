
cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

project( SISTEMA_MEDICION_EROSION )

# OPENCV (required)
find_package( OpenCV 2.4.6 REQUIRED )

# PCL (required)
find_package( PCL 1.7 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
set(PCL_LIBRARY_DIRS /usr/local/lib) # TODO: improve libraries search
link_directories( ${PCL_LIBRARY_DIRS} )

# PROJECT setup
add_subdirectory(features)
add_subdirectory(test)

set(CMAKE_CXX_FLAGS "--std=c++11 -Wno-deprecated")
