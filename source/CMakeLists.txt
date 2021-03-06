
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INCLUDE_DIR})

# -----------------------------------------------------------------------------

add_library(features
					 # Descriptor
					 ${INCLUDE_DIR}/Descriptor/DescriptorType.h
					 ${INCLUDE_DIR}/Descriptor/DescriptorRepresentation.h

					 # FeaturesFinder
					 ${INCLUDE_DIR}/FeaturesFinder/FeaturesFinder.h
					 ${INCLUDE_DIR}/FeaturesFinder/OpenCVBaseFeaturesFinder.h
					 ${INCLUDE_DIR}/FeaturesFinder/OpenCVBaseFeaturesFinder.hpp
					 ${INCLUDE_DIR}/FeaturesFinder/OpenCVBinaryFeaturesFinder.h
					 ${INCLUDE_DIR}/FeaturesFinder/OpenCVBinaryFeaturesFinder.hpp
					 ${SRC_DIR}/FeaturesFinder/OpenCVBinaryFeaturesFinder.cpp
					 ${INCLUDE_DIR}/FeaturesFinder/OpenCVRealFeaturesFinder.h
					 ${INCLUDE_DIR}/FeaturesFinder/OpenCVRealFeaturesFinder.hpp
					 ${SRC_DIR}/FeaturesFinder/OpenCVRealFeaturesFinder.cpp
					 
					 # FeaturesMatcher
					 ${INCLUDE_DIR}/FeaturesMatcher/FeaturesMatcher.h
					 ${INCLUDE_DIR}/FeaturesMatcher/RealFeaturesMatcher.h
					 ${INCLUDE_DIR}/FeaturesMatcher/RealFeaturesMatcher.hpp
					 ${SRC_DIR}/FeaturesMatcher/RealFeaturesMatcher.cpp

					 # GlobalRegistration
					 ${INCLUDE_DIR}/GlobalRegistration/GlobalRegistration.h
					 ${INCLUDE_DIR}/GlobalRegistration/GlobalRegistration.hpp
					 ${SRC_DIR}/GlobalRegistration/GlobalRegistration.cpp
					 ${INCLUDE_DIR}/GlobalRegistration/GlobalRegistrationFactory.h
					 ${SRC_DIR}/GlobalRegistration/GlobalRegistrationFactory.cpp
		)

target_link_libraries(features ${PCL_LIBRARIES} 
							   ${OpenCV_LIBRARIES} 
							   ${Boost_LIBRARIES}
					 )

# -----------------------------------------------------------------------------

add_library(utils
					 # Utils
					 ${INCLUDE_DIR}/Utils/Utils.h
					 ${SRC_DIR}/Utils/Utils.cpp 
		)

target_link_libraries(utils    ${PCL_LIBRARIES} 
					 )

# -----------------------------------------------------------------------------

add_library(io
					 # CloudGenerator
					 ${INCLUDE_DIR}/IO/CloudGenerator.h
					 ${SRC_DIR}/IO/CloudGenerator.cpp
		    )

target_link_libraries(io ${PCL_LIBRARIES} 
					     ${OpenCV_LIBRARIES} 
						 ${Boost_LIBRARIES}
					 )


# -----------------------------------------------------------------------------

add_library(tools
					 # RegistrationTool
					 ${INCLUDE_DIR}/Tools/RegistrationTool.h
					 ${INCLUDE_DIR}/Tools/RegistrationTool.hpp
					 ${SRC_DIR}/Tools/RegistrationTool.cpp
		    )

target_link_libraries(tools ${PCL_LIBRARIES} 
					        ${OpenCV_LIBRARIES} 
						    ${Boost_LIBRARIES}
					 		io
					 		features
					 		utils
					 )

add_executable(SecuencialRegistrationDemoWithInteraction.out 
					${SRC_DIR}/Demos/SecuencialRegistrationDemoWithInteraction.cpp
			  )

target_link_libraries(SecuencialRegistrationDemoWithInteraction.out   ${PCL_LIBRARIES} 
							   				  					      ${OpenCV_LIBRARIES} 
								 		      		 				  ${Boost_LIBRARIES}
 		      		 				   								  features
 		      		 				   								  utils
								 		      		 				  tools
					 )

# -----------------------------------------------------------------------------

add_executable(ScalerDemo.out 
					${SRC_DIR}/Demos/ScalerDemo.cpp
			  )

target_link_libraries(ScalerDemo.out   ${PCL_LIBRARIES} 
 		      		 				   ${Boost_LIBRARIES}
 		      		 				   features
 		      		 				   utils
					 )

# -----------------------------------------------------------------------------

add_executable(FindExtremaDemo.out 
					${SRC_DIR}/Demos/FindExtremaDemo.cpp
			  )

target_link_libraries(FindExtremaDemo.out   ${PCL_LIBRARIES} 
	 		      		 				    ${Boost_LIBRARIES}
	 		      		 				    features
	 		      		 				    utils
					 )

# -----------------------------------------------------------------------------


set(CMAKE_CXX_FLAGS "--std=c++11 -Wno-deprecated -g")
