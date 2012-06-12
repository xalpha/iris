cmake_minimum_required(VERSION 2.8)
project( IrisCC )

# find openGL
find_package( OpenGL REQUIRED )

# find Qt
if( WIN32 )
    set( QT_QMAKE_EXECUTABLE "$ENV{Qt_DIR}/bin/qmake.exe" )
endif()
find_package( Qt4 COMPONENTS QtCore QtGui QtOpenGL )

# find Iris
#set( Iris_DIR $ENV{Iris_DIR} )
#find_package( Iris REQUIRED )

# set include paths
include_directories(
    include
    extern/gl2ps
    extern/qcustomplot
    extern/qwtplot3d/include
    ui
    ${Iris_INCLUDE_DIRS}
	${OPENGL_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR} )

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
    ui/IrisCC.ui )
set( IrisCC_QT_RES res/IrisCC.qrc )

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
target_link_libraries( ${IrisCC_Target}
    ${OPENGL_LIBRARIES}
    ${QT_LIBRARIES}
    ${Iris_LIBRARY} )



# configure target
set_target_properties(${IrisCC_Target}
    PROPERTIES
        PUBLIC_HEADER "${IrisCC_INC}"
        COMPILE_DEFINITIONS "${IrisCC_COMPILE_DEFINITIONS}"
        COMPILE_FLAGS "${Iris_COMPILE_FLAGS}" )

