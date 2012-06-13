##############################################################################
#                                                                            #
# This file is part of iris, a lightweight C++ camera calibration library    #
#                                                                            #
# Copyright (C) 2012 Alexandru Duliu                                         #
#                                                                            #
# iris is free software; you can redistribute it and/or                      #
# modify it under the terms of the GNU Lesser General Public                 #
# License as published by the Free Software Foundation; either               #
# version 3 of the License, or (at your option) any later version.           #
#                                                                            #
# iris is distributed in the hope that it will be useful, but WITHOUT ANY    #
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  #
# FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the #
# GNU General Public License for more details.                               #
#                                                                            #
# You should have received a copy of the GNU Lesser General Public           #
# License along with iris. If not, see <http://www.gnu.org/licenses/>.       #
#                                                                            #
##############################################################################

# Config file for the iris library
# It defines the following variables
#
# Iris_INCLUDE_DIR - include directory for iris headers
# Iris_INCLUDE_DIRS - all include directories iris needs
# Iris_LIBRARY - library

# set path
set( Iris_DIR ${CMAKE_CURRENT_LIST_DIR} )
set( ENV{Iris_DIR} ${Iris_DIR} )

# add module paths
list( APPEND CMAKE_MODULE_PATH ${Iris_DIR}/cmake )

# find Eigen3
if( WIN32 )
    set( EIGEN3_INCLUDE_DIR $ENV{Eigen3_DIR} )
endif()
find_package( Eigen3 REQUIRED )

# find openCV
if( APPLE )
    set( OpenCV_DIR "/opt/local/lib/cmake" )
endif()
if( WIN32 )
    set( OpenCV_DIR $ENV{OpenCV_DIR} )
endif()
find_package( OpenCV REQUIRED )

# set the include dir
set( Iris_INCLUDE_DIR "${Iris_DIR}/include")

# set target names
set( Iris_TARGET iris )

# set compile definitions
set( Iris_COMPILE_DEFINITIONS IRIS )

# if this is 32-bit, disable alignment
if( NOT CMAKE_SIZEOF_VOID_P MATCHES "8")
    list( APPEND Iris_COMPILE_DEFINITIONS EIGEN_DONT_ALIGN)
endif()

# set linker flags
if( WIN32 )
	list( APPEND Iris_LINK_FLAGS " /MANIFEST:NO" )
endif()


# set library paths
set( Iris_LIBRARY ${Iris_TARGET} )

# set include directories
set( Iris_INCLUDE_DIRS
    ${Iris_INCLUDE_DIR}
    ${Iris_DIR}/extern/include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS} )

# link libraries
set( Iris_LINK_LIBRARIES
    -lm
    -lc
    ${OpenCV_LIBS} )


# enable C++11 support
if( NOT WIN32 )
#    if( CMAKE_COMPILER_IS_GNUXX )
        list( APPEND Iris_COMPILE_FLAGS "--\"std=c++0x\"" )
#    else( CMAKE_COMPILER_IS_GNUXX )
#        list( APPEND Iris_COMPILE_FLAGS "-std=c++11 -Qunused-arguments" )
#    endif()
endif()

