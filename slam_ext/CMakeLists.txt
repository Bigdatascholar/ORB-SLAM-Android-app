cmake_minimum_required(VERSION 2.4.6)
project(ORB_SLAM)

SET(CMAKE_BUILD_TYPE Release)
cmake_policy(SET CMP0012 NEW)
cmake_policy(SET CMP0003 NEW)

if(DEVELOPMENT_MODE)
set(BUILDLINUXDIR ${PROJECT_SOURCE_DIR}/build.linux)
set(ENV{CLAPACKDIR} ${BUILDLINUXDIR}/clapack)
set(ENV{SUITESPARSEDIR} ${BUILDLINUXDIR}/suitesparse)
set(ENV{G2ODIR} ${BUILDLINUXDIR}/g2o)
set(ENV{G2O_ROOT} ${BUILDLINUXDIR}/g2o)
endif()

set(Boost_NO_SYSTEM_PATHS ON)
set(BOOST_ROOT $ENV{BOOST_DIR})
message(STATUS "Boost dir: ${BOOST_ROOT}")
set(BOOST_INCLUDEDIR ${BOOST_ROOT}/include)

if(BUILD_ANDROID)
message(STATUS "Android build!")
set(OpenCV_DIR $ENV{ANDROIDOPENCV_ANDROID})
set(BOOST_LIBRARYDIR $ENV{BOOST_DIR_ANDROID}/lib)

else()
set(OpenCV_DIR $ENV{OPENCV_DIR})
set(BOOST_ROOT $ENV{BOOST_DIR})
set(BOOST_LIBRARYDIR ${BOOST_ROOT}/lib)

endif()

message(STATUS "Boost library dir: ${BOOST_LIBRARYDIR}")
set(Boost_LIBRARIES ${BOOST_LIBRARYDIR}/libboost_system.a ${BOOST_LIBRARYDIR}/libboost_thread.a)

set(G2O_ROOT $ENV{G2ODIR})
set(SUITE_SPARSE_ROOT $ENV{SUITESPARSEDIR}/build)

#add_definitions("-std=c++11")

if(BUILD_ANDROID)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=armv7-a -mfpu=neon -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -Wall -O3 -march=armv7-a -mfpu=neon -std=c++0x -fPIC")
else()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -std=c++0x -fPIC")
#add_definitions(-DUSE_CRON)
set(LINUX ON)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native -fPIC") 
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#set(Boost_USE_STATIC_LIBS OFF)
#set(Boost_USE_STATIC_RUNTIME OFF)
#set(Boost_COMPILER -gcc)

message(STATUS "OpenCV DIR: ${OpenCV_DIR}")
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)

include(${PROJECT_SOURCE_DIR}/cmake_modules/FindG2O.cmake)
if(G2O_FOUND)

else()
message(FATAL_ERROR "G2O not found!")
endif()

if(BUILD_ANDROID)
#find_host_package(Boost 1.54.0 COMPONENTS thread system REQUIRED)
find_host_package(Eigen3 REQUIRED)
else()
#find_package(Boost 1.54.0 COMPONENTS thread system REQUIRED)
find_package(Eigen3 REQUIRED)
endif()

#if(USE_FAST_FEATURES)
    #add_definitions(-DUSE_FAST_FEATURES -pg)

add_definitions(-fopenmp)

#endif()

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
${SUITESPARSECONFIG_LIBRARY}

)
# END FIX

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty
${PROJECT_SOURCE_DIR}/Thirdparty/g2o
${PROJECT_SOURCE_DIR}/Thirdparty/eigen3
${CHOLMOD_INCLUDE_DIR}
 ${BOOST_INCLUDEDIR}
$ENV{SUITESPARSEDIR}/SuiteSparse/SuiteSparse_config
)

set(HDRS_DBOW2
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/BowVector.h
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/FORB.h 
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/FClass.h       
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/FeatureVector.h
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/ScoringObject.h   
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/BowVector.cpp
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/FORB.cpp      
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DUtils/Random.h
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DUtils/Timestamp.h)
set(SRCS_DUTILS
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DUtils/Random.cpp
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DUtils/Timestamp.cpp)

add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 gomp ${OpenCV_LIBS})
if(DEFINED ENV{BUILD_ANDROID_PROFILER_LIB})
message(STATUS "Profiler set: " $ENV{BUILD_ANDROID_PROFILER_LIB})
target_link_libraries(DBoW2 $ENV{BUILD_ANDROID_PROFILER_LIB})
add_definitions(-pg -fno-omit-frame-pointer)
endif()

add_library(${PROJECT_NAME} SHARED
src/main.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
#src/FramePublisher.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
#src/MapPublisher.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
)

target_link_libraries(${PROJECT_NAME}
gomp
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Boost_LIBRARIES}
${CHOLMOD_LIBRARIES}
DBoW2
${G2O_LIBRARIES}
#pthread
)

if(DEFINED ENV{BUILD_ANDROID_PROFILER_LIB})
message(STATUS "Profiler set: " $ENV{BUILD_ANDROID_PROFILER_LIB})
target_link_libraries(${PROJECT_NAME} $ENV{BUILD_ANDROID_PROFILER_LIB})
add_definitions(-pg)
endif()

export(TARGETS ${PROJECT_NAME} DBoW2 FILE "${PROJECT_BINARY_DIR}/orbslam.cmake")
#export(PACKAGE ORB_SLAM)

# When using an IDE, add -DDEVELOPMENT_MODE=ON
if(LINUX)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/slam_linux/out")

# Build example app
file(GLOB SLAM_LINUX_SOURCES
    "slam_linux/*/*.h"
    "slam_linux/*/*.cpp"
    "slam_linux/*/*/*.h"
    "slam_linux/*/*/*.cpp"
    "slam_linux/*.h"
    "slam_linux/*.cpp"
    "src/*.cc"
    "include/*.h"
)
set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} -pg)
add_executable(slam_linux ${SLAM_LINUX_SOURCES})
target_link_libraries(slam_linux
gomp
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Boost_LIBRARIES}
${CHOLMOD_LIBRARIES}
DBoW2
${G2O_LIBRARIES})

endif()
