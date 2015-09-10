CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

option(BUILD_HOP3D "Build HOP3D libraries" ON)
option(BUILD_VISUALIZATION_MODULE "Build HOP3D" ON)
mark_as_advanced(BUILD_VISUALIZATION_MODULE)
option(BUILD_HOP3D_FILTER "Build HOP3D filter library" ON)
mark_as_advanced(BUILD_HOP3D_FILTER)
option(BUILD_HOP3D_STATISTICS "Build HOP3D statistics builder library" ON)
mark_as_advanced(BUILD_HOP3D_STATISTICS)

# demos
option(BUILD_HOP3D_DEMO "Build HOP3D demonstration programs and libraries" ON)
option(BUILD_HOP3D_DEMO_STATS "Build hop3d build statistics test" ON)
mark_as_advanced(BUILD_HOP3D_DEMO_STATS)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# HOP3D Project configuration
SET( PROJ_NAME      "HOP3D" )
SET( PROJ_PATH      ${CMAKE_SOURCE_DIR} )
SET( PROJ_OUT_PATH  ${CMAKE_BINARY_DIR} )

SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

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


#Adding flan to the project
find_package(PkgConfig)
pkg_check_modules(PC_FLANN flann)
set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
    HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS})

find_library(FLANN_LIBRARY flann
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS})

set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
set(FLANN_LIBRARIES ${FLANN_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Flann DEFAULT_MSG
    FLANN_LIBRARY FLANN_INCLUDE_DIR)

mark_as_advanced(FLANN_LIBRARY FLANN_INCLUDE_DIR)


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
#FIND_PACKAGE( Boost 1.54 COMPONENTS system filesystem REQUIRED )


find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    link_libraries(${GLEW_LIBRARIES})
endif()

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

SET(DATA_SOURCES

        ${PROJECT_PATH}/src/Data/Cloud.cpp
        ${PROJECT_PATH}/src/Data/Defs.cpp
        ${PROJECT_PATH}/src/Data/Graph.cpp
        ${PROJECT_PATH}/src/Data/Part.cpp
        ${PROJECT_PATH}/src/Data/Vocabulary.cpp
)
SET(DATA_HEADERS
        ${PROJECT_PATH}/include/Data/Cloud.h
        ${PROJECT_PATH}/include/Data/Defs.h
        ${PROJECT_PATH}/include/Data/Graph.h
        ${PROJECT_PATH}/include/Data/Part.h
        ${PROJECT_PATH}/include/Data/Vocabulary.h
)
SET(DATA_FILES
)
SOURCE_GROUP("Data" FILES ${DATA_HEADERS})
SOURCE_GROUP("Data" FILES ${DATA_SOURCES})


SET(CORE_SOURCES
        ${PROJECT_PATH}/src/Core/GraphBuilder.cpp
        ${PROJECT_PATH}/src/Core/LayerFilterer.cpp
        ${PROJECT_PATH}/src/Core/LocalInhibiter.cpp
        ${PROJECT_PATH}/src/Core/PartSelector.cpp
)
SET(CORE_HEADERS
        ${PROJECT_PATH}/include/Core/GraphBuilder.h
        ${PROJECT_PATH}/include/Core/LayerFilterer.h
        ${PROJECT_PATH}/include/Core/LocalInhibiter.h
        ${PROJECT_PATH}/include/Core/PartSelector.h
)
SET(CORE_FILES
)
SOURCE_GROUP("Core" FILES ${CORE_HEADERS})
SOURCE_GROUP("Core" FILES ${CORE_SOURCES})

SET(UTILITIES_SOURCES
        ${PROJECT_PATH}/src/Utilities/Reader.cpp
        ${PROJECT_PATH}/src/Utilities/Writer.cpp
)

SET(UTILITIES_HEADERS
        ${PROJECT_PATH}/include/Utilities/Reader.h
        ${PROJECT_PATH}/include/Utilities/Writer.h

)
SET(UTILITIES_FILES
)

IF (BUILD_VISUALIZATION_MODULE)
SET(VISUALIZER_SOURCES
        ${PROJECT_PATH}/src/Visualizer/Visualizer.cpp
        ${PROJECT_PATH}/src/Visualizer/shader.cpp
        ${PROJECT_PATH}/src/Visualizer/texture.cpp
        ${PROJECT_PATH}/src/Visualizer/objloader.cpp
        ${PROJECT_PATH}/src/Visualizer/vboindexer.cpp
        ${PROJECT_PATH}/src/Visualizer/quaternion_utils.cpp

)

SET(VISUALIZER_HEADERS
        ${PROJECT_PATH}/include/Visualizer/Visualizer.h
        ${PROJECT_PATH}/include/Visualizer/shader.hpp
        ${PROJECT_PATH}/include/Visualizer/texture.hpp
        ${PROJECT_PATH}/include/Visualizer/objloader.hpp
        ${PROJECT_PATH}/include/Visualizer/vboindexer.hpp
        ${PROJECT_PATH}/include/Visualizer/quaternion_utils.hpp
)
SET(VISUALIZER_FILES
        ${PROJECT_PATH}/src/shaders/StandardShading.fragmentshader
        ${PROJECT_PATH}/src/shaders/StandardShading.vertexshader
)
SOURCE_GROUP("VISUALIZER" FILES ${VISUALISATION_HEADERS})
SOURCE_GROUP("VISUALIZER" FILES ${VISUALISATION_SOURCES})
ENDIF (BUILD_VISUALIZATION_MODULE)

SET(APPS_SOURCES
        ${PROJECT_PATH}/src/Apps/Learning/Data.cpp
        ${PROJECT_PATH}/src/Apps/Learning/Learning.cpp
        ${PROJECT_PATH}/src/Apps/Inference/Data.cpp
        ${PROJECT_PATH}/src/Apps/Inference/Inference.cpp
        ${PROJECT_PATH}/src/Apps/Visualization/Data.cpp
        ${PROJECT_PATH}/src/Apps/Visualization/Visualization.cpp
)
SET(APPS_HEADERS
        ${PROJECT_PATH}/include/Apps/Learning/Data.h
        ${PROJECT_PATH}/include/Apps/Learning/Learning.h
        ${PROJECT_PATH}/include/Apps/Inference/Data.h
        ${PROJECT_PATH}/include/Apps/Inference/Inference.h
        ${PROJECT_PATH}/include/Apps/Visualization/Data.h
        ${PROJECT_PATH}/include/Apps/Visualization/Visualization.h
)
SET(DATA_FILES
)
SOURCE_GROUP("APPS" FILES ${APPS_HEADERS})
SOURCE_GROUP("APPS" FILES ${APPS_SOURCES})


SET( PROJ_LIBRARIES "flann" )
SET( PROJ_INCLUDES  "${PROJECT_PATH}/include/" )

IF (BUILD_VISUALIZATION_MODULE)
    FIND_PACKAGE ( OpenGL REQUIRED)
# Compile external dependencies
add_subdirectory (external)



# On Visual 2005 and above, this module can set the debug working directory
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/rpavlik-cmake-modules-1c73e35")
include(CreateLaunchers)
include(MSVCMultipleProcessCompile) # /MP

include_directories(
        external/AntTweakBar-1.16/include/
        external/glfw-3.0.3/include/GLFW/
        include/
        .
)

set(ALL_LIBS
        ${OPENGL_LIBRARY}
        GLFW_303
        ANTTWEAKBAR_116_OGLCORE_GLFW
)

add_definitions(
        -DTW_STATIC
        -DTW_NO_LIB_PRAGMA
        -DTW_NO_DIRECT3D
        -D_CRT_SECURE_NO_WARNINGS
)

ENDIF (BUILD_VISUALIZATION_MODULE)

###############################################################################
#
# tinyXML2 library
#
###############################################################################

add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/external/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/external/tinyXML/tinyxml2.cpp)

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
        TARGET_LINK_LIBRARIES(imageFilter)
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
        TARGET_LINK_LIBRARIES(statisticsBuilder)
        INSTALL(TARGETS statisticsBuilder RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${FILTER_HEADERS} DESTINATION include/hop3d/StatisticsBuilder/)
endif(BUILD_HOP3D AND BUILD_HOP3D_STATISTICS)

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
        TARGET_LINK_LIBRARIES(statsBuilder tinyxml2 statisticsBuilder)
        INSTALL(TARGETS statsBuilder RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_HOP3D_DEMO AND BUILD_HOP3D_DEMO_STATS)



ADD_LIBRARY("${PROJ_NAME}_data" ${DATA_HEADERS} ${DATA_SOURCES})
ADD_LIBRARY("${PROJ_NAME}_core" ${CORE_HEADERS} ${CORE_SOURCES})
ADD_LIBRARY("${PROJ_NAME}_utilities" ${UTILITIES_HEADERS} ${UTILITIES_SOURCES})
TARGET_LINK_LIBRARIES( "${PROJ_NAME}_utilities" "${PROJ_NAME}_data" tinyxml2)
ADD_LIBRARY("${PROJ_NAME}_visualizer" ${VISUALIZER_HEADERS} ${VISUALIZER_SOURCES})

ADD_EXECUTABLE(${PROJ_NAME} "main.cpp"   )
ADD_EXECUTABLE("${PROJ_NAME}_Visualization" "src/Apps/Visualization/Visualization.cpp")
ADD_EXECUTABLE("${PROJ_NAME}_Learning" "src/Apps/Learning/Learning.cpp")
ADD_EXECUTABLE("${PROJ_NAME}_Inference" "src/Apps/Inference/Inference.cpp")

#TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" "${PROJ_NAME}_utilities" imageFilter tinyxml2)
TARGET_LINK_LIBRARIES( "${PROJ_NAME}_Visualization" ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" "${PROJ_NAME}_utilities" "${PROJ_NAME}_visualizer" ${ALL_LIBS} )
TARGET_LINK_LIBRARIES( "${PROJ_NAME}_Learning" ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" "${PROJ_NAME}_utilities")
