
include_directories(${PROJECT_SOURCE_DIR}/features/include)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_descriptor_representation.out ${SRC_DIR}/test_descriptor_representation.cpp)

target_link_libraries(test_descriptor_representation.out gtest 
														 gtest_main
					 )


# add_executable(test_opencv_features_finder.out ${SRC_DIR}/test_opencv_features_finder.cpp)

# target_link_libraries(test_opencv_features_finder.out ${PCL_LIBRARIES} 
# 							   				 		  ${OpenCV_LIBRARIES}
# 							   				 		  gtest 
# 													  features
# 					 )

set(CMAKE_CXX_FLAGS "--std=c++11 -g")