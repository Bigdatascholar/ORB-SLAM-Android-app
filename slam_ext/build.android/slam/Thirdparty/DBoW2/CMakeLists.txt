cmake_minimum_required(VERSION 2.8)
project(DBoW2)

if(USE_FAST_FEATURES)
  add_definitions(-DUSE_FAST_FEATURES)
endif()


set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(OpenCV REQUIRED)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS} ${PROJECT_NAME})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

if(DEFINED ENV{BUILD_ANDROID_PROFILER_LIB})
message(STATUS "Profiler set: " $ENV{BUILD_ANDROID_PROFILER_LIB})
target_link_libraries(DBoW2 $ENV{BUILD_ANDROID_PROFILER_LIB})
add_definitions(-pg)
endif()

add_executable(vocBuilder vocBuilder/vocBuilder.cpp)
target_link_libraries(vocBuilder ${PROJECT_NAME} ${OpenCV_LIBS})
#file(COPY demo/images DESTINATION ${CMAKE_BINARY_DIR}/)
