CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

option(BUILD_VISUALIZATION_MODULE "Build HOP3D OpenGL 3.0 visualization module" ON)

SET( PROJ_NAME      "HOP3D" )
SET( PROJ_PATH      ${CMAKE_SOURCE_DIR} )
SET( PROJ_OUT_PATH  ${CMAKE_BINARY_DIR} )

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
        ${PROJECT_PATH}/src/Core/StatisticsBuilder.cpp
)
SET(CORE_HEADERS
        ${PROJECT_PATH}/include/Core/GraphBuilder.h
        ${PROJECT_PATH}/include/Core/LayerFilterer.h
        ${PROJECT_PATH}/include/Core/LocalInhibiter.h
        ${PROJECT_PATH}/include/Core/PartSelector.h
        ${PROJECT_PATH}/include/Core/StatisticsBuilder.h
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

PROJECT( ${PROJ_NAME} )

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
        external/glm-0.9.4.0/
        external/glew-1.9.0/include/
        external/assimp-3.0.1270/include/
        include/
        .
)

set(ALL_LIBS
        ${OPENGL_LIBRARY}
        GLFW_303
        GLEW_190
        ANTTWEAKBAR_116_OGLCORE_GLFW
)

add_definitions(
        -DTW_STATIC
        -DTW_NO_LIB_PRAGMA
        -DTW_NO_DIRECT3D
        -DGLEW_STATIC
        -D_CRT_SECURE_NO_WARNINGS
)

ENDIF (BUILD_VISUALIZATION_MODULE)


FIND_PACKAGE(OpenMP)


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

#FIND_PACKAGE( OpenCV REQUIRED )
#FIND_PACKAGE( Boost 1.54 COMPONENTS system filesystem REQUIRED )

#INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES( ${EIGEN_INCLUDE} ${PROJ_INCLUDES} )
IF (UNIX)
#"-Wall -Wextra -Wconversion" warning flags for equivalence with VS W4
SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wconversion -std=c++11 -pthread -fopenmp")
ENDIF()

###############################################################################
#
# tinyXML2 library
#
###############################################################################

add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/external/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/external/tinyXML/tinyxml2.cpp)

ADD_LIBRARY("${PROJ_NAME}_data" ${DATA_HEADERS} ${DATA_SOURCES})
ADD_LIBRARY("${PROJ_NAME}_core" ${CORE_HEADERS} ${CORE_SOURCES})
ADD_LIBRARY("${PROJ_NAME}_utilities" ${UTILITIES_HEADERS} ${UTILITIES_SOURCES} ${DATA_HEADERS} ${DATA_SOURCES})
ADD_LIBRARY("${PROJ_NAME}_visualizer" ${VISUALIZER_HEADERS} ${VISUALIZER_SOURCES})


ADD_EXECUTABLE(${PROJ_NAME} "main.cpp"   )
ADD_EXECUTABLE("${PROJ_NAME}_Visualization" "src/Apps/Visualization/Visualization.cpp")
ADD_EXECUTABLE("${PROJ_NAME}_Learning" "src/Apps/Learning/Learning.cpp")
ADD_EXECUTABLE("${PROJ_NAME}_Inference" "src/Apps/Inference/Inference.cpp")

#TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" "${PROJ_NAME}_utilities")
TARGET_LINK_LIBRARIES( "${PROJ_NAME}_Visualization" ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" "${PROJ_NAME}_utilities" "${PROJ_NAME}_visualizer" ${ALL_LIBS} )
TARGET_LINK_LIBRARIES( "${PROJ_NAME}_Learning" ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" "${PROJ_NAME}_utilities")
