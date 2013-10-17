
cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

project( SISTEMA_MEDICION_EROSION )

# OPENCV (required)
find_package( OpenCV 2.4.6 REQUIRED )

# PCL (required)
find_package( PCL 1.7 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${EIGEN3_DEFINITIONS}) 

# BOOST (required)
find_package( Boost 1.49 COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

# workaround
# replace on /usr/include/ni/XnPlatform.h 
# linux by __linux__ and i386 by __i386__

# PROJECT setup
add_subdirectory(source)
# add_subdirectory(test)

set(CMAKE_CXX_FLAGS "--std=c++11 -Wno-deprecated -g")
