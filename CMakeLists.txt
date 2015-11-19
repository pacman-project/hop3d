cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# HOP3D Project configuration
PROJECT(HOP3D)

# HOP3D Project configuration
SET( PROJ_NAME      "HOP3D" )
SET( PROJ_PATH      ${CMAKE_SOURCE_DIR} )
SET( PROJ_OUT_PATH  ${CMAKE_BINARY_DIR} )

SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread -O0")

option(BUILD_HOP3D "Build HOP3D libraries" ON)
option(BUILD_HOP3D_UTILITIES "Build HOP3D Utilities library" ON)
mark_as_advanced(BUILD_HOP3D_UTILITIES)
option(BUILD_HOP3D_FILTER "Build HOP3D filter library" ON)
mark_as_advanced(BUILD_HOP3D_FILTER)
option(BUILD_HOP3D_STATISTICS "Build HOP3D statistics builder library" ON)
mark_as_advanced(BUILD_HOP3D_STATISTICS)
option(BUILD_HOP3D_PART_SELECTOR "Build HOP3D part selector library" ON)
mark_as_advanced(BUILD_HOP3D_PART_SELECTOR)
option(BUILD_HOP3D_DATASET "Build HOP3D dataset library" ON)
mark_as_advanced(BUILD_HOP3D_DATASET)
option(BUILD_HOP3D_OBJECT_COMPOSITION "Build HOP3D object composition library" ON)
mark_as_advanced(BUILD_HOP3D_OBJECT_COMPOSITION)
option(BUILD_VISUALIZATION_QT "Build HOP3D visualization QT" ON)
mark_as_advanced(BUILD_VISUALIZATION_QT)

# demos
option(BUILD_HOP3D_DEMO "Build HOP3D demonstration programs and libraries" ON)
option(BUILD_HOP3D_DEMO_STATS "Build hop3d build statistics test" ON)
mark_as_advanced(BUILD_HOP3D_DEMO_STATS)
option(BUILD_HOP3D_DEMO_VISUALIZER "Build hop3d visualizer demo" ON)
mark_as_advanced(BUILD_HOP3D_DEMO_VISUALIZER)
option(BUILD_HOP3D_DEMO_QVISUALIZER "Build hop3d Qt visualizer demo" ON)
mark_as_advanced(BUILD_HOP3D_DEMO_QVISUALIZER)
option(BUILD_HOP3D_DEMO_NORMAL_FILTERER "Build hop3d Normal Filterer demo" ON)
mark_as_advanced(BUILD_HOP3D_DEMO_NORMAL_FILTERER)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")

# Project root directory
GET_FILENAME_COMPONENT(PROJECT_PATH . ABSOLUTE CACHE INTERNAL "Path prefix for the project")
# Windows 32/64 bit
IF (WIN32)
        SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
        IF(X86_64)
                SET(PROGRAM_FILES $ENV{ProgramW6432})
                SET(X86_WIN "64")
                SET(X86_X64 "/x64")
        ELSE(X86_64)
                SET(PROGRAM_FILES $ENV{ProgramFiles})
                SET(X86_WIN "32")
                SET(X86_X64 "")
        ENDIF(X86_64)
ENDIF()

#OpenGL library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
FIND_PACKAGE(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${OPENGL_LIBRARY})

if (BUILD_VISUALIZATION_QT)
    #QGLViewer library
    FIND_PACKAGE(QGLViewer)
    include_directories(${QGLVIEWER_INCLUDE_DIR})
    include_directories(${QT_INCLUDES})
    link_directories(${QGLVIEWER_LIBRARY})
endif (BUILD_VISUALIZATION_QT)


#Adding OpenMP
IF (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()

IF (WIN32)
# Eigen
        SET(EIGEN_INCLUDE "${PROGRAM_FILES}/eigen" CACHE PATH "Path prefix for EIGEN include")
ENDIF()
INCLUDE_DIRECTORIES( ${EIGEN_INCLUDE} ${PROJ_INCLUDES} )

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON) 
FIND_PACKAGE( Boost 1.54 COMPONENTS system filesystem REQUIRED )

# PCL
FIND_PACKAGE(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
        SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
        SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

IF (UNIX)
#"-Wall -Wextra -Wconversion" warning flags for equivalence with VS W4
SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wconversion -std=c++11 -pthread -fopenmp")
ENDIF()

SET( PROJ_INCLUDES  "${PROJECT_PATH}/include/" )

###############################################################################
#
# tinyXML2 library
#
###############################################################################

IF (WIN32)
	add_library(tinyxml2 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/external/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/external/tinyXML/tinyxml2.cpp)
ELSE (UNIX)
	add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/external/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/external/tinyXML/tinyxml2.cpp)
ENDIF()

###############################################################################
#
# External Octree library
#
###############################################################################

include_directories(${CMAKE_SOURCE_DIR}/external/octree)

###############################################################################
#
# HOP3D Data
#
###############################################################################

if(BUILD_HOP3D)
        file(GLOB DATA_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Data/*.cpp"
        )
        file(GLOB DATA_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Data/*.h"
        )
        ADD_LIBRARY(hop3dData STATIC ${DATA_SOURCES} ${DATA_HEADERS})
        target_link_libraries(hop3dData)# tinyxml2 opencv_core opencv_highgui opencv_contrib boost_filesystem boost_system)
        INSTALL(TARGETS hop3dData RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${DATA_HEADERS} DESTINATION include/hop3d/Data/)
endif(BUILD_HOP3D)

###############################################################################
#
# HOP3D Utilities library
#
###############################################################################

if(BUILD_HOP3D AND BUILD_HOP3D_UTILITIES)
        file(GLOB UTILITIES_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Utilities/*.cpp"
        )
        file(GLOB UTILITIES_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Utilities/*.h"
        )

        ADD_LIBRARY(Utilities STATIC ${UTILITIES_SOURCES} ${UTILITIES_HEADERS})
        target_link_libraries(Utilities tinyxml2 opencv_core opencv_highgui opencv_contrib boost_filesystem boost_system)
        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${UTILITIES_HEADERS} DESTINATION include/hop3d/Utilities/)
endif(BUILD_HOP3D AND BUILD_HOP3D_UTILITIES)

###############################################################################
#
# HOP3D image filter library
#
###############################################################################

if(BUILD_HOP3D AND BUILD_HOP3D_FILTER)
        file(GLOB FILTER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/ImageFilter/*.cpp"
        )
        file(GLOB FILTER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/ImageFilter/*.h"
        )

        ADD_LIBRARY(imageFilter STATIC ${FILTER_SOURCES} ${FILTER_HEADERS})
        TARGET_LINK_LIBRARIES(imageFilter Utilities hop3dData)
        INSTALL(TARGETS imageFilter RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${FILTER_HEADERS} DESTINATION include/hop3d/ImageFilter/)
endif(BUILD_HOP3D AND BUILD_HOP3D_FILTER)

###############################################################################
#
# HOP3D statistics builder library
#
###############################################################################

if(BUILD_HOP3D AND BUILD_HOP3D_STATISTICS)
        file(GLOB FILTER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/StatisticsBuilder/*.cpp"
        )
        file(GLOB FILTER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/StatisticsBuilder/*.h"
        )

        ADD_LIBRARY(statisticsBuilder STATIC ${FILTER_SOURCES} ${FILTER_HEADERS})
        TARGET_LINK_LIBRARIES(statisticsBuilder hop3dData)
        INSTALL(TARGETS statisticsBuilder RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${FILTER_HEADERS} DESTINATION include/hop3d/StatisticsBuilder/)
endif(BUILD_HOP3D AND BUILD_HOP3D_STATISTICS)

###############################################################################
#
# HOP3D part selector library
#
###############################################################################

if(BUILD_HOP3D AND BUILD_HOP3D_PART_SELECTOR)
        file(GLOB SELECTOR_SOURCES
            "${CMAKE_SOURCE_DIR}/src/PartSelector/*.cpp"
        )
        file(GLOB SELECTOR_HEADERS
            "${CMAKE_SOURCE_DIR}/include/PartSelector/*.h"
        )

        ADD_LIBRARY(partSelector STATIC ${SELECTOR_SOURCES} ${SELECTOR_HEADERS})
        TARGET_LINK_LIBRARIES(partSelector hop3dData imageFilter)
        INSTALL(TARGETS partSelector RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${SELECTOR_HEADERS} DESTINATION include/hop3d/PartSelector/)
endif(BUILD_HOP3D AND BUILD_HOP3D_PART_SELECTOR)

###############################################################################
#
# HOP3D object composition library
#
###############################################################################

if(BUILD_HOP3D AND BUILD_HOP3D_OBJECT_COMPOSITION)
        file(GLOB OBJ_COMPOSITION_SOURCES
            "${CMAKE_SOURCE_DIR}/src/ObjectComposition/*.cpp"
        )
        file(GLOB OBJ_COMPOSITION_HEADERS
            "${CMAKE_SOURCE_DIR}/include/ObjectComposition/*.h"
        )

        ADD_LIBRARY(objectComposition STATIC ${OBJ_COMPOSITION_SOURCES} ${OBJ_COMPOSITION_HEADERS})
        TARGET_LINK_LIBRARIES(objectComposition hop3dData)
        INSTALL(TARGETS objectComposition RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${OBJ_COMPOSITION_HEADERS} DESTINATION include/hop3d/ObjectComposition/)
endif(BUILD_HOP3D AND BUILD_HOP3D_OBJECT_COMPOSITION)

###############################################################################
#
# HOP3D Visualizer library
#
###############################################################################

if(BUILD_HOP3D AND BUILD_VISUALIZATION_QT)
        add_definitions( -DQVisualizerBuild=1 )
        file(GLOB QVISUALIZER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/QVisualizer/*.cpp"
        )
        file(GLOB QVISUALIZER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/QVisualizer/*.h"
        )
        ADD_LIBRARY(QVisualizer STATIC ${QVISUALIZER_SOURCES} ${QVISUALIZER_HEADERS})
        TARGET_LINK_LIBRARIES(QVisualizer tinyxml2 ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY})
        INSTALL(TARGETS QVisualizer RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${QVISUALIZER_HEADERS} DESTINATION include/hop3d/QVisualizer/)
endif(BUILD_HOP3D AND BUILD_VISUALIZATION_QT)

###############################################################################
#
# HOP3D Dataset library
#
###############################################################################

if(BUILD_HOP3D AND BUILD_HOP3D_DATASET)
        file(GLOB DATASET_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Dataset/*.cpp"
        )
        file(GLOB DATASET_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Dataset/*.h"
        )
        ADD_LIBRARY(Dataset STATIC ${DATASET_SOURCES} ${DATASET_HEADERS})
        TARGET_LINK_LIBRARIES(Dataset tinyxml2 ${PCL_LIBRARIES})
        INSTALL(TARGETS Dataset RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${DATASET_HEADERS} DESTINATION include/hop3d/Dataset/)
endif(BUILD_HOP3D AND BUILD_HOP3D_DATASET)

###############################################################################
#
# HOP3D library
#
###############################################################################

if(BUILD_HOP3D)
        file(GLOB LIBHOP3D_SOURCES
            "${CMAKE_SOURCE_DIR}/src/HOP3D/*.cpp"
        )
        file(GLOB LIBHOP3D_HEADERS
            "${CMAKE_SOURCE_DIR}/include/HOP3D/*.h"
        )
        ADD_LIBRARY(LIBHOP3D STATIC ${LIBHOP3D_SOURCES} ${LIBHOP3D_HEADERS})
        TARGET_LINK_LIBRARIES(LIBHOP3D tinyxml2 Dataset statisticsBuilder Utilities partSelector imageFilter objectComposition)
        INSTALL(TARGETS LIBHOP3D RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${LIBHOP3D_HEADERS} DESTINATION include/hop3D/HOP3D/)
endif(BUILD_HOP3D)


###############################################################################
#
# HOP3D DEMO stats builder
#
###############################################################################

if(BUILD_HOP3D_DEMO AND BUILD_HOP3D_DEMO_STATS)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./src/Apps/DemoStatsBuilder/statsBuilder.cpp)
        ADD_EXECUTABLE(statsBuilder ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(statsBuilder tinyxml2 statisticsBuilder imageFilter partSelector)
        INSTALL(TARGETS statsBuilder RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_HOP3D_DEMO AND BUILD_HOP3D_DEMO_STATS)

###############################################################################
#
# HOP3D demo Visualizer
#
###############################################################################

if(BUILD_HOP3D_DEMO AND BUILD_HOP3D_DEMO_QVISUALIZER AND BUILD_VISUALIZATION_QT)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./src/Apps/DemoQVisualizer/demoQVisualizer.cpp)
        ADD_EXECUTABLE(demoQVisualizer ${DEMO_SOURCES})
#        add_dependencies(demoVisualizer Map)
        TARGET_LINK_LIBRARIES(demoQVisualizer LIBHOP3D QVisualizer tinyxml2 ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${OPENGL_LIBRARY} glut)
        INSTALL(TARGETS demoQVisualizer RUNTIME DESTINATION bin)
endif(BUILD_HOP3D_DEMO AND BUILD_HOP3D_DEMO_QVISUALIZER AND BUILD_VISUALIZATION_QT)

###############################################################################
#
# HOP3D demo Normal filterer
#
###############################################################################

if(BUILD_HOP3D_DEMO AND BUILD_HOP3D_DEMO_NORMAL_FILTERER)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./src/Apps/DemoNormalFilterer/demoNormalFilterer.cpp)
        ADD_EXECUTABLE(demoNormalFilterer ${DEMO_SOURCES})
#        add_dependencies(demoVisualizer Map)
        TARGET_LINK_LIBRARIES(demoNormalFilterer imageFilter Dataset tinyxml2 boost_system)
        INSTALL(TARGETS demoNormalFilterer RUNTIME DESTINATION bin)
endif(BUILD_HOP3D_DEMO AND BUILD_HOP3D_DEMO_NORMAL_FILTERER)

ADD_EXECUTABLE(${PROJ_NAME} "main.cpp"   )


#TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
#TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" Utilities imageFilter tinyxml2 opencv_core)
