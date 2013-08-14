
add_executable(test_descriptor_representation.out test_descriptor_representation.cpp)
target_link_libraries(test_descriptor_representation.out gtest gtest_main)

set(CMAKE_CXX_FLAGS "--std=c++11")