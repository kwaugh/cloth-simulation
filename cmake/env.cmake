# Directories
LINK_DIRECTORIES("/usr/local/lib" "/opt/local/lib")
INCLUDE_DIRECTORIES("/usr/local/include" "/opt/local/include")
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Flags
#set(CMAKE_CXX_FLAGS "--std=c++11 -g -fmax-errors=1")
# it should use openmp by default
set(CMAKE_CXX_FLAGS "--std=c++11 -g ")
#set(CMAKE_CXX_FLAGS "--std=c++11 -g -pg")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

# Packages
#OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIRS})
LINK_DIRECTORIES(${OPENGL_LIBRARY_DIRS})
ADD_DEFINITIONS(${OPENGL_DEFINITIONS})

MESSAGE(STATUS "OpenGL: ${OPENGL_LIBRARIES}")
LIST(APPEND stdgl_libraries ${OPENGL_gl_LIBRARY})

if (APPLE)
	FIND_LIBRARY(COCOA_LIBRARY Cocoa REQUIRED)
	FIND_LIBRARY(IOKIT_LIBRARY IOKit REQUIRED)
	FIND_LIBRARY(CoreVideo_LIBRARY CoreVideo REQUIRED)
	LIST(APPEND stdgl_libraries iconv ${COCOA_LIBRARY} ${IOKIT_LIBRARY} ${CoreVideo_LIBRARY})
endif(APPLE)

#EIGEN
set(EIGEN3_DIR ${CMAKE_SOURCE_DIR}/lib/eigen/cmake)
FIND_PACKAGE(Eigen3 REQUIRED NO_MODULE)
if (NOT EIGEN3_FOUND)
    message(FATAL_ERROR "Eigen not found.")
endif()
message(STATUS "Eigen found.")

#LIBIGL
FIND_PACKAGE(LIBIGL REQUIRED)

if (NOT LIBIGL_FOUND)
    message(FATAL_ERROR "LIBIGL not found.")
endif()

option(LIBIGL_WITH_NANOGUI     "Use Nanogui menu"   ON)
option(LIBIGL_WITH_VIEWER      "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_OPENGL      "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW"           ON)

add_subdirectory("${PROJECT_SOURCE_DIR}/lib/libigl/shared/cmake" "libigl")

# Prepare the build environment
include_directories(${LIBIGL_INCLUDE_DIRS})
add_definitions(${LIBIGL_DEFINITIONS})
