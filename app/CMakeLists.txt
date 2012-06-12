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

# set the header files of QWT Plot 3D
set( QWT_PLOT_3D_INC
    extern/qwtplot3d/include/qwt3d_extglwidget.h
    extern/qwtplot3d/include/qwt3d_color.h
    extern/qwtplot3d/include/qwt3d_global.h
    extern/qwtplot3d/include/qwt3d_types.h
    extern/qwtplot3d/include/qwt3d_axis.h
    extern/qwtplot3d/include/qwt3d_coordsys.h
    extern/qwtplot3d/include/qwt3d_drawable.h
    extern/qwtplot3d/include/qwt3d_helper.h
    extern/qwtplot3d/include/qwt3d_label.h
    extern/qwtplot3d/include/qwt3d_openglhelper.h
    extern/qwtplot3d/include/qwt3d_curve.h
    extern/qwtplot3d/include/qwt3d_colorlegend.h
    extern/qwtplot3d/include/qwt3d_plot.h
    extern/qwtplot3d/include/qwt3d_enrichment.h
    extern/qwtplot3d/include/qwt3d_enrichment_std.h
    extern/qwtplot3d/include/qwt3d_autoscaler.h
    extern/qwtplot3d/include/qwt3d_autoptr.h
    extern/qwtplot3d/include/qwt3d_io.h
    extern/qwtplot3d/include/qwt3d_io_reader.h
    extern/qwtplot3d/include/qwt3d_scale.h
    extern/qwtplot3d/include/qwt3d_portability.h
    extern/qwtplot3d/include/qwt3d_mapping.h
    extern/qwtplot3d/include/qwt3d_gridmapping.h
    extern/qwtplot3d/include/qwt3d_parametricsurface.h
    extern/qwtplot3d/include/qwt3d_function.h
    extern/qwtplot3d/include/qwt3d_volumeplot.h
    extern/qwtplot3d/include/qwt3d_graphplot.h
    extern/qwtplot3d/include/qwt3d_multiplot.h
    extern/qwtplot3d/include/qwt3d_io_gl2ps.h
	extern/gl2ps/gl2ps.h )

# the source files for QWT Plot 3D
set( QWT_PLOT_3D_SRC
    extern/qwtplot3d/src/qwt3d_extglwidget.cpp
    extern/qwtplot3d/src/qwt3d_axis.cpp
    extern/qwtplot3d/src/qwt3d_color.cpp
    extern/qwtplot3d/src/qwt3d_coordsys.cpp
    extern/qwtplot3d/src/qwt3d_drawable.cpp
    extern/qwtplot3d/src/qwt3d_mousekeyboard.cpp
    extern/qwtplot3d/src/qwt3d_movements.cpp
    extern/qwtplot3d/src/qwt3d_lighting.cpp
    extern/qwtplot3d/src/qwt3d_curve.cpp
    extern/qwtplot3d/src/qwt3d_colorlegend.cpp
    extern/qwtplot3d/src/qwt3d_plot.cpp
    extern/qwtplot3d/src/qwt3d_label.cpp
    extern/qwtplot3d/src/qwt3d_types.cpp
    extern/qwtplot3d/src/qwt3d_enrichment_std.cpp
    extern/qwtplot3d/src/qwt3d_autoscaler.cpp
    extern/qwtplot3d/src/qwt3d_io_reader.cpp
    extern/qwtplot3d/src/qwt3d_io.cpp
    extern/qwtplot3d/src/qwt3d_scale.cpp
    extern/qwtplot3d/src/qwt3d_gridmapping.cpp
    extern/qwtplot3d/src/qwt3d_parametricsurface.cpp
    extern/qwtplot3d/src/qwt3d_function.cpp
    extern/qwtplot3d/src/qwt3d_gridplot.cpp
    extern/qwtplot3d/src/qwt3d_meshplot.cpp
    extern/qwtplot3d/src/qwt3d_io_gl2ps.cpp
	extern/gl2ps/gl2ps.c )

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

