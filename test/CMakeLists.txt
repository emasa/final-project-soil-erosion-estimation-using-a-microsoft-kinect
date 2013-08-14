
include_directories(${PROJECT_SOURCE_DIR}/features/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/gtest-1.6.0/include)

add_subdirectory(features)
add_subdirectory(gtest-1.6.0)
