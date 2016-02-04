CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(g2o)

# The library prefix
SET(LIB_PREFIX g2o_)

SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

SET(G2O_BUILD_EXAMPLES FALSE)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# Allow the developer to select if Dynamic or Static libraries are built
OPTION (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" ON)
SET (G2O_LIB_TYPE STATIC)
IF (BUILD_SHARED_LIBS)
  SET (G2O_LIB_TYPE SHARED)
ENDIF()

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)


# Set the output directory for the build executables and libraries
SET(g2o_RUNTIME_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
IF(WIN32)
  SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
ELSE(WIN32)
  SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
ENDIF(WIN32)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_RUNTIME_OUTPUT_DIRECTORY})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(UNIX)
IF(ANDROID)
  ADD_DEFINITIONS(-DANDROID)
  MESSAGE(STATUS "Compiling on ANDROID")
ENDIF(ANDROID)

#BLAS_LIBRARIES_DIR OR BLAS_LIBRARIES
# CHOLMOD FIX
set(ENV{CHOLMODDIR} "$ENV{SUITESPARSEDIR}/build/lib")
set(ENV{CHOLMODINCLUDEDIR} "$ENV{SUITESPARSEDIR}/SuiteSparse/CHOLMOD/Include")
set(CHOLMOD_INCLUDE_DIR $ENV{CHOLMODINCLUDEDIR})
set(CHOLMOD_LIBS $ENV{CHOLMODDIR})

message(STATUS "Cholmod dir: " $ENV{CHOLMODDIR})
message(STATUS "Cholmod include dir: " $ENV{CHOLMODINCLUDEDIR})

set(CHOLMOD_LIBRARY ${CHOLMOD_LIBS}/libcholmod.a)
set(AMD_LIBRARY ${CHOLMOD_LIBS}/libamd.a)
set(COLAMD_LIBRARY ${CHOLMOD_LIBS}/libcolamd.a)
set(CAMD_LIBRARY ${CHOLMOD_LIBS}/libcamd.a)
set(CCOLAMD_LIBRARY ${CHOLMOD_LIBS}/libccolamd.a)
set(SUITESPARSECONFIG_LIBRARY ${CHOLMOD_LIBS}/libsuitesparseconfig.a)

set(CHOLMOD_LIBRARIES
${CHOLMOD_LIBRARY}
${AMD_LIBRARY}
${COLAMD_LIBRARY}
${CAMD_LIBRARY}
${CCOLAMD_LIBRARY}
${SUITESPARSECONFIG_LIBRARY})

SET(CHOLMOD_FOUND TRUE)
# END FIX

#SET(BLAS_LIB_DIR "$ENV{CLAPACKDIR}/BLAS/SRC")

#

set(BLAS_LIBRARIES_DIR $ENV{CLAPACKDIR}/BLAS/SRC)
set(BLAS_LIBRARIES $ENV{CLAPACKDIR}/build/BLAS/SRC/libblas.a)
set(F2C_LIBRARIES $ENV{CLAPACKDIR}/build/F2CLIBS/libf2c/libf2c.a)
find_package(BLAS)
set(LAPACK_LIBRARIES_DIR $ENV{CLAPACKDIR}/build/SRC)
set(LAPACK_LIBRARIES $ENV{CLAPACKDIR}/build/SRC/liblapack.a)
find_package(LAPACK)
IF(CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
  MESSAGE(STATUS "Found CHOLMOD and its dependencies")
  SET(CHOLMOD_FOUND TRUE)

	message(STATUS "BLAS: " ${BLAS_LIBRARIES})
ELSE()
  SET(CHOLMOD_FOUND FALSE)
ENDIF()

SET(CSPARSE_FOUND FALSE)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
#FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# Compiler specific options for gcc
if(ANDROID)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=armv7-a -mfpu=neon -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=armv7-a -mfpu=neon -std=c++0x -fPIC")
else()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -std=c++0x -fPIC") 
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native -fPIC") 
endif()

# activate warnings !!!
SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
SET(G2O_EIGEN3_INCLUDE "${g2o_SOURCE_DIR}/EXTERNAL/eigen3")
SET(EIGEN3_INCLUDE_DIR ${G2O_EIGEN3_INCLUDE})
FIND_PACKAGE(Eigen3 REQUIRED)
IF(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)

MESSAGE(FATAL_ERROR "Eigen not found")
ENDIF(EIGEN3_FOUND)

# Generate config.h
SET(G2O_HAVE_CHOLMOD ${CHOLMOD_FOUND})
SET(G2O_SHARED_LIBS ${BUILD_SHARED_LIBS})
SET(G2O_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)

# Set up the top-level include directories
INCLUDE_DIRECTORIES(${g2o_SOURCE_DIR} ${G2O_EIGEN3_INCLUDE} $ENV{SUITESPARSEDIR})

# Include the subdirectories
ADD_SUBDIRECTORY(g2o)
