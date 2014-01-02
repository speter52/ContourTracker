project(arc-template)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set build directory
set(CMAKE_CURRENT_BINARY_DIR ${PROJECT_SOURCE_DIR}/build)

	MESSAGE(STATUS "--------------------------------------------------")
    MESSAGE(STATUS "Project: ARC Template ")
    MESSAGE(STATUS "by Martin Miller <miller7@illinois.edu>")
	MESSAGE(STATUS "--------------------------------------------------")


    SET( CMAKE_BUILD_TYPE Debug ) 							# set to [Release|Debug] to switch optimization flags
# -------------------------------------------------------------------- #
# Application setup
# -------------------------------------------------------------------- #

SET ( APPLICATION_NAME arc-template )
    FILE( GLOB_RECURSE PROJECT_SOURCES src/*.cpp )
add_definitions(-g -Wall -Werror)
# -------------------------------------------------------------------- #
# Change with caution!
# -------------------------------------------------------------------- #

	MESSAGE(STATUS "Building -- ${CMAKE_BUILD_TYPE} -- Version")
	
	SET(CMAKE_VERBOSE_MAKEFILE ON)
	SET(OPTIMIZATION_FLAG_DEBUG "-O0 -pg")
	SET(OPTIMIZATION_FLAG_RELEASE "-O3 -march=native -mtune=native -msse3 -mfpmath=sse -fomit-frame-pointer -finline-functions -floop-optimize -ffinite-math-only")
    SET(WARNING_FLAG_DEBUG "-g -Werror -Wall -Winit-self -Wsequence-point -Wunused")
    SET(WARNING_FLAG_RELEASE "-Winit-self -Wsequence-point -Wunused -Wno-deprecated")

	IF( CMAKE_BUILD_TYPE MATCHES Release )
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPTIMIZATION_FLAG_RELEASE} -fmessage-length=0 -pipe ${WARNING_FLAG_RELEASE}")
	ELSE( CMAKE_BUILD_TYPE MATCHES Debug )
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPTIMIZATION_FLAG_DEBUG} -fmessage-length=0 -pipe ${WARNING_FLAG_DEBUG}")
	ENDIF( CMAKE_BUILD_TYPE MATCHES Release )
# -------------------------------------------------------------------- #
# Setting up Library linkages
# -------------------------------------------------------------------- #

	ADD_EXECUTABLE (
		${APPLICATION_NAME}
		${PROJECT_SOURCES}
	)
