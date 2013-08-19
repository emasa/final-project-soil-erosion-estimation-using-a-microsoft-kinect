
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${INCLUDE_DIR})

set(incl ${INCLUDE_DIR}/features/descriptor_types.h
		 ${INCLUDE_DIR}/features/descriptor_representations.h
		 ${INCLUDE_DIR}/features/FeaturesFinder.h
		 ${INCLUDE_DIR}/features/OpenCVBaseFeaturesFinder.h
		 ${INCLUDE_DIR}/features/OpenCVBinaryFeaturesFinder.h
		 ${INCLUDE_DIR}/features/OpenCVRealFeaturesFinder.h
		 ${INCLUDE_DIR}/features/Utils.h
	)

set(impl ${INCLUDE_DIR}/features/impl/OpenCVBaseFeaturesFinder.hpp
		 ${INCLUDE_DIR}/features/impl/OpenCVBinaryFeaturesFinder.hpp
		 ${INCLUDE_DIR}/features/impl/OpenCVRealFeaturesFinder.hpp
	)

set(src ${SRC_DIR}/OpenCVBinaryFeaturesFinder.cpp
		${SRC_DIR}/OpenCVRealFeaturesFinder.cpp
		${SRC_DIR}/Utils.cpp
	)

add_library(features ${src} ${incl} ${impl})

target_link_libraries(features ${PCL_LIBRARIES} 
							   ${OpenCV_LIBRARIES} 
							   ${Boost_LIBRARIES}
					 )

set(CMAKE_CXX_FLAGS "--std=c++11 -Wno-deprecated"
	)
