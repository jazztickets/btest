cmake_minimum_required(VERSION 2.8)

add_definitions("-DBT_NO_PROFILE")
#add_definitions("-DBT_USE_DOUBLE_PRECISION")

# project name
project(btest)

# for codelite clang integration
if(DEFINED ENV{CXX})
    message("-- Using CXX $ENV{CXX}")
    set(CMAKE_CXX_COMPILER $ENV{CXX} )
endif()

if(DEFINED ENV{CC})
    message("-- Using CC $ENV{CC}")
    set(CMAKE_CC_COMPILER $ENV{CC} )
endif()

# add extra find modules
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")

# windows only options
if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

	# set output path
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
	add_definitions("-D_CRT_SECURE_NO_WARNINGS")
else()

	# set default build type
	if(NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE Release)
	endif()

	# set output path
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()

set(BULLET_PHYSICS_SOURCE_DIR ${PROJECT_SOURCE_DIR}/bullet)
set(BULLET_VERSION 2.88)
subdirs("ext/bullet")

# set include directories
include_directories("ext/bullet")
include_directories("src")

# add source files
file(GLOB SRC_MAIN src/*.cpp src/*.h)
file(GLOB SRC_ALL ${SRC_MAIN})

# add virtual folders
source_group(main FILES ${SRC_MAIN})

# build binary
add_executable(${CMAKE_PROJECT_NAME} ${SRC_ALL})

# link libraries
target_link_libraries(${CMAKE_PROJECT_NAME} BulletDynamics BulletCollision LinearMath)
