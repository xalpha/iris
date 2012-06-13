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
project( IrisCC )

# find openGL
find_package( OpenGL REQUIRED )

# find Qt
if( WIN32 )
    set( QT_QMAKE_EXECUTABLE "$ENV{Qt_DIR}/bin/qmake.exe" )
endif()
find_package( Qt4 COMPONENTS QtCore QtGui QtOpenGL )

# try to find Bazar
list( APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share" )
find_package( Bazar )

# some more qt stuff
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

# QCustomPlot (way smaller than QWT)
set( Q_CUSTOM_PLOT_INC extern/qcustomplot/qcustomplot.h )
set( Q_CUSTOM_PLOT_SRC extern/qcustomplot/qcustomplot.cpp )

# set the target
set( IrisCC_Target irisCC )

# set the header files of QObject derived classes
set( IrisCC_INC include/IrisCC.hpp )
set( IrisCC_SRC src/IrisCC.cpp src/main.cpp )
set( IrisCC_QT_UI
    ui/ChessboardFinder.ui
    ui/IrisCC.ui
    ui/OpenCVCalibration.ui
    ui/OpenCVStereoCalibration.ui )
set( IrisCC_QT_RES res/IrisCC.qrc )

# set include directories
set( IrisCC_INCLUDE_DIRS
    include
    extern/gl2ps
    extern/qcustomplot
    extern/qwtplot3d/include
    ${Iris_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR} )

# set link libraries
set( IrisCC_LINK_LIBS
    ${OPENGL_LIBRARIES}
    ${QT_LIBRARIES}
    ${Iris_LIBRARY} )

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

# if Bazar
if( Bazar_FOUND )
    list( APPEND IrisCC_INCLUDE_DIRS ${Bazar_INCLUDE_DIR} )
    list( APPEND IrisCC_LINK_LIBS ${Bazar_LIBRARY} )
endif()


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

# set link libraries
target_link_libraries( ${IrisCC_Target} ${IrisCC_LINK_LIBS} )

# configure target
set_target_properties(${IrisCC_Target}
    PROPERTIES
        PUBLIC_HEADER "${IrisCC_INC}"
        COMPILE_DEFINITIONS "${IrisCC_COMPILE_DEFINITIONS}"
        COMPILE_FLAGS "${Iris_COMPILE_FLAGS}" )


