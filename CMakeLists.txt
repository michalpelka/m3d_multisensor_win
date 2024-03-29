cmake_minimum_required(VERSION 3.0.2)
project(m3d_multisensor)

## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
option(BUILD_GL_VIEVER "Build GL viever " ON )
option(BUILD_GL_CALIB "Build GL calib " OFF )


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

SET(SOPHUS_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/3rd/Sophus)
add_definitions(-DSOPHUS_USE_BASIC_LOGGING)
include_directories( ${SOPHUS_INCLUDE_DIRS})

IF (${BUILD_GL_VIEVER} STREQUAL "ON")
	find_package (Eigen3 REQUIRED)
	INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
	message("EIGEN3_INCLUDE_DIR" ${EIGEN3_INCLUDE_DIR})
	find_package(OpenGL REQUIRED)	
	message("Building BUILD_GL_VIEVER")
	IF (WIN32)
			message("Building BUILD_GL_VIEVER - WIN32")
	# use prebuilt binaries for Windows
		SET(GLEW_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/3rd_precompiled_win64/glew-2.1.0/include)
		SET(GLEW_LIBRARIES ${CMAKE_SOURCE_DIR}/3rd_precompiled_win64/glew-2.1.0/lib/Release/x64/glew32.lib) # static GLEW library
		SET(GLFW_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/3rd_precompiled_win64/glfw-3.3.6.bin.WIN64/include)
		SET(GLFW_LIBRARIES ${CMAKE_SOURCE_DIR}/3rd_precompiled_win64/glfw-3.3.6.bin.WIN64/lib-vc2019/glfw3.lib) # static GLFW library
		SET(OPENGL_LIBRARIES opengl32)
	else()

		find_package(OpenGL REQUIRED)
		find_package(glfw3 REQUIRED)
		find_package(glm REQUIRED)
		find_package(GLEW REQUIRED)
		SET(GLFW_LIBRARIES glfw)
		
		if (GLEW_FOUND)
			include_directories(${GLEW_INCLUDE_DIRS})
			link_libraries(${GLEW_LIBRARIES})
		endif()
	endif()

	# glm is header only
	SET(GLM_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/3rd/glm/)
	#Sophus is header only 
	SET(SOPHUS_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/3rd/Sophus)
	add_definitions(-DSOPHUS_USE_BASIC_LOGGING)
	include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${SOPHUS_INCLUDE_DIRS}
	)
	include_directories( ${CMAKE_SOURCE_DIR}/3rd/ ${CMAKE_SOURCE_DIR}/3rd/imgui  ${CMAKE_SOURCE_DIR}/3rd/imgui/backends)

	add_library(GL_Codes STATIC
			# wrapers
			GL/IndexBuffer.cpp
			GL/Renderer.cpp
			GL/Texture.cpp
			GL/VertexBuffer.cpp
			GL/Shader.cpp
			GL/shaders_code.cpp
			GL/VertexArray.cpp
			GL/VertexBufferLayout.cpp
			# imgui
			3rd/imgui/imgui.cpp 3rd/imgui/imgui_demo.cpp 3rd/imgui/imgui.cpp
			3rd/imgui/imgui_draw.cpp 3rd/imgui/imgui_widgets.cpp
			3rd/imgui/imgui_tables.cpp
			3rd/imgui/backends/imgui_impl_opengl3.cpp 3rd/imgui/backends/imgui_impl_glfw.cpp
			#ImViewGizmo
			3rd/ImGuizmo/ImGuizmo.cpp
			3rd/stb_image.h
			)

	IF (${BUILD_GL_CALIB} STREQUAL "ON")	
		find_package(Ceres REQUIRED)
		find_package(OpenCV COMPONENTS core highgui calib3d REQUIRED)
		add_definitions("-DWITH_OPENCV")
	endif()
endif()

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED COMPONENTS common io )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Boost COMPONENTS system filesystem thread program_options regex  REQUIRED)
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
add_library(m3d_clients_robot  STATIC clients/robot_client.cpp )

include_directories(3rd/Boost-Serial-Port/)
add_library(m3d_clients_novatel  STATIC clients/novatel_client.cpp 3rd/Boost-Serial-Port/BoostSerial.cpp)

include_directories(3rd/mongoose)
add_library(mongoose STATIC 3rd/mongoose/mongoose.c)

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
		${PCL_COMMON_LIBRARY}
		${PCL_IO_LIBRARY}
		${PCL_OCTREE_LIBRARY}
		${Boost_LIBRARIES}
		m3d_clients_livox		
		${LADYBUG_LIBS}
		livox_sdk_static
		mongoose
		m3d_clients_robot
		m3d_clients_novatel)

# target_compile_features(node_native PUBLIC cxx_std_14)

#m3d_clients_XDK

MESSAGE(${PCL_LIBRARIES})
MESSAGE(INFO ${PCL_COMMON_LIBRARY})

add_executable(collapse_data collapse_data/collapse_data.cpp  gl_calib_livox/transform.cpp)
target_link_libraries(collapse_data ${PCL_COMMON_LIBRARY}
		${PCL_IO_LIBRARY}
		${PCL_OCTREE_LIBRARY}
		${Boost_LIBRARIES}
		${PCL_IO_LIBRARY}
		${PCL_OCTREE_LIBRARY}
		${Boost_LIBRARIES}
)

IF (${BUILD_GL_VIEVER} STREQUAL "ON")	
	add_executable (gl_viever gl_viewer/gl_viewer.cpp gl_calib_livox/structs.cpp gl_calib_livox/transform.cpp)
	#add_dependencies(node_native ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(gl_viever
			${PCL_COMMON_LIBRARY}
			${PCL_IO_LIBRARY}
			${PCL_OCTREE_LIBRARY}
			${Boost_LIBRARIES}	
			${LADYBUG_LIBS}
			${GLEW_LIBRARIES}
			${GLFW_LIBRARIES}
			GL_Codes )
	IF (WIN32)
		target_link_libraries(gl_viever opengl32)
	else()
		target_link_libraries(gl_viever ${OPENGL_LIBRARIES} ${CMAKE_DL_LIBS})
	endif()
	IF (${BUILD_GL_CALIB} STREQUAL "ON")
		target_link_libraries(gl_viever ${OpenCV_LIBS})
	endif()

endif()

IF (${BUILD_GL_CALIB} STREQUAL "ON")
	add_executable (gl_calib_livox gl_calib_livox/gl_calib_livox.cpp gl_calib_livox/structs.cpp gl_calib_livox/transform.cpp)
	#add_dependencies(node_native ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(gl_calib_livox
			${PCL_COMMON_LIBRARY}
			${PCL_IO_LIBRARY}
			${PCL_OCTREE_LIBRARY}
			${Boost_LIBRARIES}	
			${LADYBUG_LIBS}
			${GLEW_LIBRARIES} 
			${GLFW_LIBRARIES}
			${OPENGL_LIBRARIES}
			GL_Codes
			${CMAKE_DL_LIBS}
			${CERES_LIBRARIES}
			${OpenCV_LIBS})
	
	add_executable(gl_calib_velodyne gl_calib_velodyne/gl_calib_velodyne.cpp gl_calib_velodyne/utils.cpp )
	target_link_libraries(gl_calib_velodyne
			${CERES_LIBRARIES} GL_Codes ${OPENGL_LIBRARIES} glfw ${GLEW_LIBRARIES} ${PCL_LIBRARIES} Boost::program_options ${OpenCV_LIBS})

	add_executable(gl_calib_spherical gl_calib_velodyne/shperical_line_calibration.cpp gl_calib_velodyne/utils.cpp gl_calib_velodyne/line_detector.cpp gl_calib_livox/structs.cpp gl_calib_livox/transform.cpp)
	target_link_libraries(gl_calib_spherical
			${CERES_LIBRARIES} GL_Codes ${OPENGL_LIBRARIES} glfw ${GLEW_LIBRARIES} ${PCL_LIBRARIES} Boost::program_options ${OpenCV_LIBS})

	add_executable(gl_calib_perspective gl_calib_velodyne/perspective_line_calibration.cpp gl_calib_velodyne/utils.cpp gl_calib_velodyne/line_detector.cpp gl_calib_livox/structs.cpp gl_calib_livox/transform.cpp)
	target_link_libraries(gl_calib_perspective
			${CERES_LIBRARIES} GL_Codes ${OPENGL_LIBRARIES} glfw ${GLEW_LIBRARIES} ${PCL_LIBRARIES} Boost::program_options ${OpenCV_LIBS})

	# target_compile_features(gl_viever PUBLIC cxx_std_14)

endif()
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/index.htm
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
	 file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/VLP-16.xml
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
