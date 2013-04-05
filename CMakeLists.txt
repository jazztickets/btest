cmake_minimum_required(VERSION 2.8)

# project name
project(btest)

# add extra find modules
#set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/;${CMAKE_MODULE_PATH}")

# windows only options
if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
	#include_directories("${PROJECT_SOURCE_DIR}/extlibs/include")
	#link_directories("${PROJECT_SOURCE_DIR}/extlibs/")

	# set output path
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
	add_definitions("-D_CRT_SECURE_NO_WARNINGS")
elseif(UNIX)
	
	# set default build type
	if(NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE Release)
	endif()
	
	# set output path
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()

include_directories("bullet")
set(BULLET_PHYSICS_SOURCE_DIR ${PROJECT_SOURCE_DIR}/bullet)
set(BULLET_VERSION 2.81)
add_definitions("-DBT_NO_PROFILE")
subdirs("bullet")

# external libraries
#find_package(Threads REQUIRED)

# set include directories
include_directories("src")

# add source files
file(GLOB SRC_MAIN src/*.cpp src/*.h)
file(GLOB SRC_ALL ${SRC_MAIN})

# add virtual folders
source_group(main FILES ${SRC_MAIN})

# add precompiled header
#add_pch(${CMAKE_PROJECT_NAME} src/stdafx.cpp ${PROJECT_SOURCE_DIR}/src/stdafx.h)

# build binary
add_executable(${CMAKE_PROJECT_NAME} ${SRC_ALL})

# link libraries
target_link_libraries(${CMAKE_PROJECT_NAME} BulletDynamics BulletCollision LinearMath)