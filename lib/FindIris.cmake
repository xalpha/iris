#  Try to find Iris
#
#  Iris_FOUND - System has Iris
#  Iris_INCLUDE_DIRS - The Iris include directories
#  Iris_LIBRARY - The libraries needed to use Iris
#  Iris_DEFINITIONS - Compiler switches required for using Iris


# try to find the include dir
find_path( Iris_INCLUDE_DIR 
    NAMES
        iris/CameraCalibration.hpp
        iris/Finder.hpp
    PATHS
        ${Iris_DIR}/include
	    $ENV{Iris_DIR}/include
	    $ENV{HOME}/.local/include
	    ${CMAKE_INSTALL_PREFIX}/include
        /usr/include
        /usr/local/include
        /opt/include
        /opt/local/include
    PATH_SUFFIXES
        iris )
        
# find the path to the exported lib files
find_file( Iris_DEPENDS Iris.cmake
    PATHS
        $ENV{HOME}/.local/share
        ${CMAKE_INSTALL_PREFIX}/share
        /usr/share
        /usr/local/share
        /opt/share
        /opt/local/share )

# load all the targets
if( Iris_DEPENDS )
    include( "${Iris_DEPENDS}" )
else()
    MESSAGE( FATAL_ERROR "Iris: not installed.")
endif()

# check if this is a valid component
set( Iris_LIBRARY iris )
if( TARGET ${Iris_LIBRARY} )
    # include the component
    list( APPEND Iris_LIBRARIES ${Iris_LIBRARY} )
    MESSAGE( STATUS "Iris found.")
else()
    MESSAGE( FATAL_ERROR "Iris target not available.")
endif()


# set the include dirs
set( Iris_INCLUDE_DIRS 
    ${Iris_INCLUDE_DIR}
    ${Iris_INCLUDE_DIR}/iris )
    
    


#####
## Dependencies
###



# find Eigen3
if( NOT EIGEN3_FOUND )
    if( WIN32 )
        set( EIGEN3_INCLUDE_DIR $ENV{Eigen3_DIR} )
    endif()
    find_package( Eigen3 REQUIRED )
endif()
list( APPEND Iris_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} )


# find OpenCV
if( NOT OPENCV_FOUND )
    if( APPLE )
        set( OpenCV_DIR "/opt/local/lib/cmake" )
    endif()
    if( WIN32 )
        set( OpenCV_DIR $ENV{OpenCV_DIR} )
    endif()
    find_package( OpenCV REQUIRED )
endif()
list( APPEND Iris_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} )


