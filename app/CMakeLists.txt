##############################################################################
#                                                                            #
# This file is part of IrisCC, a C++ UI for camera calibration               #
#                                                                            #
# Copyright (C) 2012 Alexandru Duliu                                         #
#                                                                            #
# IrisCC is free software; you can redistribute it and/or                    #
# modify it under the terms of the GNU  General Public License               #
# as published by the Free Software Foundation; either version 3             #
# of the License, or (at your option) any later version.                     #
#                                                                            #
# IrisCC is distributed in the hope that it will be useful,                  #
# but WITHOUT ANY WARRANTY; without even the implied warranty of             #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              #
# GNU General Public License for more details.                               #
#                                                                            #
# You should have received a copy of the GNU General Public License          #
# along with IrisCC. If not, see <http://www.gnu.org/licenses/>.             #
#                                                                            #
##############################################################################


cmake_minimum_required(VERSION 2.8)
project( IrisCC CXX )

# add module paths
list( APPEND CMAKE_MODULE_PATH
    ${Iris_DIR}/cmake
    ${CMAKE_INSTALL_PREFIX}/share
    ${CMAKE_CURRENT_LIST_DIR}/cmake
    $ENV{HOME}/.local/share )

# find GLEW
find_package( GLEW REQUIRED )

# find Qt
if( WIN32 )
    set( QT_QMAKE_EXECUTABLE "$ENV{Qt_DIR}/bin/qmake.exe" )
endif()
find_package( Qt4 COMPONENTS QtCore QtGui QtOpenGL QtXml )

# find openCV
if( APPLE )
    set( OpenCV_DIR "/opt/local/lib/cmake" )
endif()
if( WIN32 )
    set( OpenCV_DIR $ENV{OpenCV_DIR} )
endif()
find_package( OpenCV REQUIRED )


# some more qt stuff
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

# QCustomPlot (way smaller than QWT)
set( Q_CUSTOM_PLOT_INC extern/qcustomplot/qcustomplot.h )
set( Q_CUSTOM_PLOT_SRC extern/qcustomplot/qcustomplot.cpp )

# set the target
set( IrisCC_Target irisCC )

# set the header files of QObject derived classes
set( IrisCC_INC
    include/IrisCC.hpp
    include/WidgetQt.hpp
    include/WorldPoses.hpp )
set( IrisCC_SRC
    src/IrisCC.cpp
    src/WidgetQt.cpp
    src/WorldPoses.cpp
    src/main.cpp )
set( IrisCC_QT_UI
    ui/CameraConfig.ui
    ui/CameraInfo.ui
    ui/ChessboardFinder.ui
    ui/IrisCC.ui
    ui/OpenCVSingleCalibration.ui
    ui/OpenCVStereoCalibration.ui
    ui/RandomFeatureFinder.ui )
set( IrisCC_QT_RES res/IrisCC.qrc )

# find NOX
find_package( Nox QUIET )
if( NOT ${Nox_FOUND} )
    message(STATUS "Nox not found, pulling from server...")
    include(ExternalProject)
    ExternalProject_Add(Nox
        GIT_REPOSITORY "https://github.com/xalpha/nox.git"
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=$ENV{HOME}/.local )
endif()

# check if Random Dot Markers was found
if( UCHIYAMARKERS_FOUND )
    list( APPEND IrisCC_QT_UI ui/UCHIYAMARKERSFinder.ui )
endif()

# set include directories
set( IrisCC_INCLUDE_DIRS
    include
    extern/gl2ps
    extern/qcustomplot
    ${Iris_INCLUDE_DIRS}
    ${Nox_INCLUDE_DIRS}
    $ENV{HOME}/.local/include
    ${CMAKE_CURRENT_BINARY_DIR} )

# set link libraries
set( IrisCC_LINK_LIBS
    ${Nox_LIBRARIES}
    ${GLEW_LIBRARY}
    ${QT_LIBRARIES}
    ${Iris_LIBRARIES} )

# set compile definitions
set( IrisCC_COMPILE_DEFINITIONS
    ${Iris_COMPILE_DEFINITIONS}
    QT_NO_DEPRECATED
    _USE_MATH_DEFINES )

# run the qt moc & uic
QT4_ADD_RESOURCES( IrisCC_QT_RES_SRC ${IrisCC_QT_RES})
QT4_WRAP_CPP( IrisCC_QT_HPP_MOC
    ${IrisCC_INC}
    ${Q_CUSTOM_PLOT_INC} )
QT4_WRAP_UI( IrisCC_QT_UI_UIC ${IrisCC_QT_UI} )

# set include paths
include_directories( ${IrisCC_INCLUDE_DIRS} )

# add the exec
add_executable( ${IrisCC_Target}
    ${IrisCC_SRC}
    ${IrisCC_INC}
    ${IrisCC_QT_HPP_MOC}
    ${IrisCC_QT_UI_UIC}
    ${IrisCC_QT_RES_SRC}
    ${Q_CUSTOM_PLOT_INC}
    ${Q_CUSTOM_PLOT_SRC} )

# enable C++11 support
if( NOT WIN32 )
    if( CMAKE_COMPILER_IS_GNUCXX )
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++0x")
    else( CMAKE_COMPILER_IS_GNUCXX )
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Qunused-arguments")
    endif()
endif()

# set link libraries
target_link_libraries( ${IrisCC_Target} ${IrisCC_LINK_LIBS} )

# configure target
set_target_properties(${IrisCC_Target}
    PROPERTIES
        PUBLIC_HEADER "${IrisCC_INC}"
        COMPILE_DEFINITIONS "${IrisCC_COMPILE_DEFINITIONS}"
        COMPILE_FLAGS "${Iris_COMPILE_FLAGS}"
        LINK_FLAGS "${Iris_LINK_FLAGS}" )
