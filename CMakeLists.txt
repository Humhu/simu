cmake_minimum_required(VERSION 2.8.3)
project(simu)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

if( NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "" )
    message( STATUS "No build type selected, default to Release" )
    set( CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE )
endif()
message( STATUS "Build configuration: ${CMAKE_BUILD_TYPE}" )

find_package( catkin REQUIRED 
              COMPONENTS argus_utils
                         extrinsics_array
                         geometry_msgs
                         message_generation
                         nav_msgs
                         paraset
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

#catkin_python_setup()

add_service_files( FILES
                   InitializeSimulation.srv
                   SetBodyPose.srv
                   SetBodyVelocity.srv
                   SetNoiseProperties.srv
)

generate_messages( DEPENDENCIES geometry_msgs )

catkin_package( INCLUDE_DIRS     include

                LIBRARIES        simu

                CATKIN_DEPENDS   argus_utils
                                 extrinsics_array
                                 geometry_msgs
                                 message_runtime
                                 nav_msgs
                                 paraset

                DEPENDS          Boost 
                                 system_lib
)

include_directories( include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    )

add_library( simu
            src/NoiseGenerator.cpp
            src/SimulationCore.cpp
            src/SimuCommon.cpp
            src/SimulatedMass.cpp
            src/SimulatedVelocitySensor.cpp
)
target_link_libraries( simu
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      )
add_dependencies( simu ${catkin_EXPORTED_TARGETS} )

add_executable( simulator_node nodes/simulator_node.cpp )
add_dependencies( simulator_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries( simulator_node ${catkin_LIBRARIES} simu )

## Mark executables and/or libraries for installation
install(TARGETS simu simulator_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
