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


cmake_minimum_required(VERSION 2.8)
project(Iris)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# configure the project
include( IrisConfig.cmake )

# add the include files
list( APPEND Iris_INC
    include/iris/CameraCalibration.hpp
    include/iris/CameraSet.hpp
    include/iris/ChessboardFinder.hpp
    include/iris/Finder.hpp
    include/iris/OpenCVCalibration.hpp
    include/iris/OpenCVSingleCalibration.hpp
    include/iris/OpenCVStereoCalibration.hpp
    include/iris/RandomFeatureDescriptor.hpp
    include/iris/RandomFeatureFinder.hpp
    include/iris/util.hpp )
list( APPEND Iris_SRC
    src/CameraCalibration.cpp
    src/ChessboardFinder.cpp
    src/Finder.cpp
    src/OpenCVCalibration.cpp
    src/OpenCVSingleCalibration.cpp
    src/OpenCVStereoCalibration.cpp
    src/RandomFeatureFinder.cpp )

# external dependencies of iris
list( APPEND Iris_EXTERN_INC
    extern/cimg/CImg.h
    extern/tinyxml2/tinyxml2.h )
list( APPEND Iris_EXTERN_SRC
    extern/tinyxml2/tinyxml2.cpp )

# set include directories
include_directories( ${Iris_INCLUDE_DIRS} )

# add library
add_library( ${Iris_TARGET} ${Iris_INC} ${Iris_SRC} ${Iris_EXTERN_INC} ${Iris_EXTERN_SRC} )
target_link_libraries( ${Iris_TARGET} ${Iris_LINK_LIBRARIES} )

# configure target
set_target_properties(${Iris_TARGET}
    PROPERTIES
        PUBLIC_HEADER "${Iris_INC}"
        COMPILE_DEFINITIONS "${Iris_COMPILE_DEFINITIONS}"
        COMPILE_FLAGS "${Iris_COMPILE_FLAGS}"
        LINK_FLAGS "${Iris_LINK_FLAGS}" )

export( TARGETS ${Iris_TARGET} FILE "${CMAKE_BINARY_DIR}/Iris_Depends.cmake")

# install the lib's binary and headers
install(TARGETS ${Iris_TARGET}
        EXPORT Iris
        RUNTIME DESTINATION "bin"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib"
        PUBLIC_HEADER DESTINATION "include/iris" )

# install targets
install(EXPORT Iris DESTINATION share )

# install module finder
install(FILES "${CMAKE_CURRENT_LIST_DIR}/FindIris.cmake" DESTINATION share )

# install extern headers
install(FILES ${Iris_EXTERN_INC} DESTINATION "include" )


# testing
enable_testing()
add_subdirectory( test )

