
cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

project( SISTEMA_MEDICION_EROSION )

# OPENCV setup
find_package( OpenCV 2.4.6 REQUIRED )

# PCL setup
find_package( PCL 1.6 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

set(PCL_LIBRARY_DIRS /usr/local/lib) # TODO: hardcoded path
link_directories( ${PCL_LIBRARY_DIRS} )

# PROJECT setup
set(PROJECT_INCLUDE_DIRS include)
set(PROJECT_SOURCE_DIR source)
include_directories(${PROJECT_INCLUDE_DIRS})

add_executable( FeaturesFinderDemo.out ${PROJECT_SOURCE_DIR}/FeaturesFinderDemo.cpp 
			  					   ${PROJECT_SOURCE_DIR}/OrganizedFeaturesFinder.cpp
			  )

target_link_libraries( FeaturesFinderDemo.out ${OpenCV_LIBS}
											  # ${PCL_COMMON_LIBRARIES} 
											  # ${PCL_IO_LIBRARIES} 
											  ${PCL_LIBRARIES} 
					 )

set(CMAKE_CXX_FLAGS "--std=c++11")
