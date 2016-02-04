# Find the header files

set(G2O_INCLUDE_DIR ${G2O_ROOT})
message(STATUS "G2O Find, using dir ${G2O_INCLUDE_DIR}")

#message(STATUS "CMAKE_FIND_ROOT_PATH='${CMAKE_FIND_ROOT_PATH}'")

#FIND_PATH(G2O_INCLUDE_DIR core/base_vertex.h
#  PATHS
#  ${G2O_ROOT}
#  ${G2O_ROOT}/include
#  $ENV{G2O_ROOT}/include
#  $ENV{G2O_ROOT}
#  /usr/local/include
#  /usr/include
#  /opt/local/include
#  /sw/local/include
#  /sw/include
#  PATH_SUFFIXES g2o
#  NO_DEFAULT_PATH
#  )

#message(STATUS "G2O_INCLUDE_DIR='${G2O_INCLUDE_DIR}'")

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

MACRO(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)
  
set(${MYLIBRARY} "${G2O_ROOT}/lib/libg2o_${MYLIBRARYNAME}.so")

  #FIND_LIBRARY(${MYLIBRARY}
  #  g2o_${MYLIBRARYNAME}
   # PATHS
  #  ${G2O_ROOT}/lib
  #  NO_DEFAULT_PATH
  #  )
  
  #  IF(MYLIBRARY)
  #    SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})

  #  ENDIF(MYLIBRARY)

	if(${MYLIBRARY})
message(STATUS "Found ${MYLIBRARY}")
else()
message(STATUS "NOT Found ${MYLIBRARY}")
endif()
  
ENDMACRO(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)

# Find the CLI library
#FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
#FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
#FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
#FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)

 SET(G2O_LIBRARIES  ${G2O_CORE_LIBRARY} 
                    ${G2O_STUFF_LIBRARY}
                    ${G2O_CLI_LIBRARY}
                    ${G2O_SOLVER_CHOLMOD}
                    #${G2O_SOLVER_CSPARSE}
                    #${G2O_SOLVER_CSPARSE_EXTENSION}
                    ${G2O_SOLVER_DENSE}
                    ${G2O_SOLVER_PCG}
                    ${G2O_SOLVER_SLAM2D_LINEAR}
                    ${G2O_SOLVER_STRUCTURE_ONLY}
                    ${G2O_SOLVER_EIGEN}
                    ${G2O_TYPES_DATA}
                    ${G2O_TYPES_ICP}
                    ${G2O_TYPES_SBA}
                    ${G2O_TYPES_SCLAM2D} 
                    ${G2O_TYPES_SIM3}
                    ${G2O_TYPES_SCLAM2D}
                    ${G2O_TYPES_SLAM3D})

# G2O solvers declared found if we found at least one solver
SET(G2O_SOLVERS_FOUND "NO")
IF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)
  SET(G2O_SOLVERS_FOUND "YES")
ENDIF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)

# G2O itself declared found if we found the core libraries and at least one solver
SET(G2O_FOUND "NO")
IF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
  SET(G2O_FOUND "YES")
ENDIF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
