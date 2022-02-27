cmake_minimum_required(VERSION 3.0.2)
project(m3d_multisensor)

## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(LADYBUG_SDK "NOT_SET" CACHE PATH "SDK ROOT DIR" )
set (LADYBUG_LIBS "")
IF (LADYBUG_SDK STREQUAL "NOT_SET")
	message("Building without Ladybug support")
else()
	add_definitions("-DWITH_LADYBUG")
	include_directories (${LADYBUG_SDK}/include)

	add_library(ladybug_capture  STATIC ladybug/ladybug_capture.cpp)
	SET(LADYBUG_LIBS ${LADYBUG_SDK}/lib64/ladybug.lib ladybug_capture)
endif()


## System dependencies are found with CMake's conventions

find_package(PCL REQUIRED COMPONENTS common io )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Boost COMPONENTS system filesystem thread  REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system thread chrono)

include_directories(
  include velodyne livox clients
  ${CMAKE_SOURCE_DIR}
  ${Boost_INCLUDE_DIR}
  ${PCL_COMMON_INCLUDE_DIR}
  ${PCL_COMMON_IO_DIR}
  ${EIGEN_INCLUDE_DIR}
)

add_subdirectory(3rd/Livox-SDK)
include_directories(3rd/Livox-SDK/sdk_core/include)
#find_library(LIVOX_SDK_LIBRARY STATIC liblivox_sdk_static.a /usr/local/lib)
add_library(m3d_clients_unit STATIC clients/mavlink_client_udp.cpp)
add_library(m3d_clients_livox STATIC clients/livox_client.cpp livox/lds_lidar.cpp)
add_library(m3d_clients_VLP  STATIC clients/velodyne_client.cpp velodyne/PacketDecoder.cpp velodyne/PacketDriver.cpp)

include_directories(3rd/mongoose)
add_library(mongoose  STATIC 3rd/mongoose/mongoose.c)

#xsens
#include_directories(3rd/xsens/xspublic/ 3rd/xsens/xspublic/xscommon/ 3rd/xsens/xspublic/xscontroller/ xsens/xspublic/xstypes/)
#link_directories(3rd/xsens/xspublic/xscommon/ 3rd/xsens/xspublic/xscontroller/ 3rd/xsens/xspublic/xstypes/ )

#add_library(m3d_clients_XDK  )
#set(XDK_LIBRARIES xscontroller xstypes xscommon ${CMAKE_DL_LIBS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_executable (node_native health_server.cpp  node_native.cpp)
#add_dependencies(node_native ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_native
        m3d_clients_unit 
		m3d_clients_VLP 
		${PCL_LIBRARIES}
		${Boost_LIBRARIES}
		m3d_clients_livox		
		#${LIVOX_SDK_LIBRARY} 
		${LADYBUG_LIBS}
		livox_sdk_static
		mongoose)
#m3d_clients_XDK


