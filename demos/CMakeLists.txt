
include_directories(${PROJECT_SOURCE_DIR}/features/include)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(FeaturesFinderDemo.out ${SRC_DIR}/FeaturesFinderDemo.cpp)

target_link_libraries(FeaturesFinderDemo.out ${PCL_LIBRARIES} 
							   				 ${OpenCV_LIBRARIES} 
								 		     features
					 )

set(CMAKE_CXX_FLAGS "--std=c++11 -Wno-deprecated -g")
