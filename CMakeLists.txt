cmake_minimum_required(VERSION 2.8)
project(Iris)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# set some variabiles
set( CMAKE_BUILD_TYPE Release )
if( NOT WIN32 )
    set( BUILD_SHARED_LIBS TRUE )
endif()

# configure the project
include( IrisConfig.cmake )

# add the include files
list( APPEND Iris_INC
    include/iris/Calibration.hpp
    include/iris/CameraCalibration.hpp
    include/iris/ChessboardFinder.hpp
    include/iris/Finder.hpp
    include/iris/OpenCVCalibration.hpp
    include/iris/vision.hpp
    extern/include/CImg.h )

# add the source files
list( APPEND Iris_SRC
    src/Calibration.cpp
    src/CameraCalibration.cpp
    src/ChessboardFinder.cpp
    src/Finder.cpp
    src/OpenCVCalibration.cpp )

# set include directories
include_directories( ${Iris_INCLUDE_DIRS} )

# add release library
add_library( ${Iris_TARGET} ${Iris_INC} ${Iris_SRC} ${LIBPPM_SRC} )
target_link_libraries( ${Iris_TARGET} ${Iris_LINK_LIBRARIES} )

# configure target
set_target_properties(${Iris_TARGET}
    PROPERTIES
        PUBLIC_HEADER "${Iris_INC}"
        COMPILE_DEFINITIONS "${Iris_COMPILE_DEFINITIONS}"
        COMPILE_FLAGS "${Iris_COMPILE_FLAGS}"
        LINK_FLAGS "${Iris_LINK_FLAGS}" )

export( TARGETS ${Iris_TARGET} FILE "${CMAKE_BINARY_DIR}/Iris_Export_Depends.cmake")

# install the lib's binary and headers
install(TARGETS ${Iris_TARGET}
        EXPORT Iris
        RUNTIME DESTINATION "bin"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib"
        PUBLIC_HEADER DESTINATION "include/iris" )

# install targets
install(EXPORT Iris DESTINATION share )



# add the app
add_subdirectory( app )
