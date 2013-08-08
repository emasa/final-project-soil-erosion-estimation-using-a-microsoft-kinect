
cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

project( SISTEMA_MEDICION_EROSION )

# OPENCV setup
find_package( OpenCV 2.4.6 REQUIRED )

# PCL setup
find_package( PCL 1.7 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

set(PCL_LIBRARY_DIRS /usr/local/lib) # TODO: hardcoded path
link_directories( ${PCL_LIBRARY_DIRS} )

# PROJECT setup
set(PROJECT_INCLUDE_DIRS include)
set(PROJECT_IMPL_DIRS ${PROJECT_INCLUDE_DIRS}/impl)
set(PROJECT_SOURCE_DIR source)
include_directories(${PROJECT_INCLUDE_DIRS})

add_executable( FeaturesFinderDemo.out  # ${PROJECT_SOURCE_DIR}/Utils.cpp
										${PROJECT_SOURCE_DIR}/PCLFeaturesFinder.cpp
			  					   		${PROJECT_SOURCE_DIR}/OpenCVFeaturesFinder.cpp
			  					   		${PROJECT_SOURCE_DIR}/FeaturesFinderDemo.cpp 			  					   		
			  )

target_link_libraries( FeaturesFinderDemo.out ${OpenCV_LIBS}
											  ${PCL_LIBRARIES} 
					 )

add_executable( MatchFeaturesDemo.out  # ${PROJECT_SOURCE_DIR}/Utils.cpp
										${PROJECT_SOURCE_DIR}/PCLFeaturesFinder.cpp
			  					   		${PROJECT_SOURCE_DIR}/OpenCVFeaturesFinder.cpp
			  					   		${PROJECT_SOURCE_DIR}/MatchFeaturesDemo.cpp 			   		
			  )

target_link_libraries( MatchFeaturesDemo.out ${OpenCV_LIBS}
											 ${PCL_LIBRARIES} 
					 )


set(CMAKE_CXX_FLAGS "--std=c++11 -Wno-deprecated")
