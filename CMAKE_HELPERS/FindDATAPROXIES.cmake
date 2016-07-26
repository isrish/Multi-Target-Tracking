# ===================================================================================
# Date : 2015-03-16
# Modified  : Pelorson Quentin
# Copyright 2015, INRIA
#
#  The DataProxies CMake configuration file
#
#  Usage from an external project: 
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(DATAPROXIES REQUIRED )
#    INCLUDE_DIRECTORIES(DATAPROXIES_INCLUDE_DIRS)
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${DATAPROXIES_LIBRARIES})
#
#  If DATAPROXIES related include and libs are not installed in /usr/local/ directory, you should set DATAPROXIES_SDK_DIRECTORY
#    This file will define the following variables:
#      - DATAPROXIES_LIBRARIES     : The list of libraries to links against.
#      - DATAPROXIES_INCLUDE_DIRS  : The DataProxies related include directories.
#      - DATAPROXIES_FOUND         : Set if DataProxies linux library has been found
#
# ===================================================================================

SET ( DATAPROXIES_FOUND FALSE )


IF ( NOT DEFINED DATAPROXIES_SDK_DIRECTORY )
SET ( DATAPROXIES_SDK_DIRECTORY "/usr/local/" CACHE PATH "Path to find DataProxies SDK location" )
ENDIF ()

FIND_PATH ( DATAPROXIES_INCLUDE_DIR dataproxies.h
  PATHS ${DATAPROXIES_SDK_DIRECTORY}
   PATH_SUFFIXES include
 )

## List of libraries DataProxies related
set ( DATAPROXIES_LIB_LIST dataproxies )
foreach(lib ${DATAPROXIES_LIB_LIST})

FIND_LIBRARY ( DATAPROXIES_${lib}_LIBRARY ${lib}
  PATHS ${DATAPROXIES_SDK_DIRECTORY}
  PATH_SUFFIXES lib
  )
IF (DATAPROXIES_${lib}_LIBRARY)
  SET (DATAPROXIES_LIBRARIES ${DATAPROXIES_LIBRARIES} ${DATAPROXIES_${lib}_LIBRARY})
ENDIF()
endforeach()

MESSAGE ( STATUS "DATAPROXIES SYS   LIB DIR = ${DATAPROXIES_LIBRARIES}" )

MESSAGE ( STATUS "DATAPROXIES INCLUDE DIR = ${DATAPROXIES_INCLUDE_DIR}" )


IF ( DATAPROXIES_INCLUDE_DIR AND DATAPROXIES_LIBRARIES)
  SET( DATAPROXIES_INCLUDE_DIRS ${DATAPROXIES_INCLUDE_DIR})
  SET( DATAPROXIES_LIBRARIES ${DATAPROXIES_LIBRARIES} )
  SET( DATAPROXIES_FOUND TRUE )
ENDIF ()

MESSAGE ( STATUS "DATAPROXIES INCLUDE DIR = ${DATAPROXIES_INCLUDE_DIRS}" )

MARK_AS_ADVANCED (
  DATAPROXIES_INCLUDE_DIRS
  DATAPROXIES_LIBRARIES
  DATAPROXIES_FOUND
  )

# handle the QUIETLY and REQUIRED arguments
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS( DATAPROXIES DEFAULT_MSG DATAPROXIES_LIBRARIES DATAPROXIES_INCLUDE_DIRS )
