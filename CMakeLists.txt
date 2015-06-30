CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

SET( PROJ_NAME      "HOP3D" )
SET( PROJ_PATH      ${CMAKE_SOURCE_DIR} )
SET( PROJ_OUT_PATH  ${CMAKE_BINARY_DIR} )

# Project root directory
GET_FILENAME_COMPONENT(PROJECT_PATH . ABSOLUTE CACHE INTERNAL "Path prefix for the project")
# Windows 32/64 bit
IF (WIN32)
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
SET(UTILITIES_SOURCES
        ${PROJECT_PATH}/src/Utilities/Reader.cpp
        ${PROJECT_PATH}/src/Utilities/Visualizer.cpp
        ${PROJECT_PATH}/src/Utilities/Writer.cpp
)
SET(UTILITIES_HEADERS
        ${PROJECT_PATH}/include/Utilities/Reader.h
        ${PROJECT_PATH}/include/Utilities/Visualizer.h
        ${PROJECT_PATH}/include/Utilities/Writer.h
)
SET(UTILITIES_FILES
)
SOURCE_GROUP("Utilities" FILES ${UTILITIES_HEADERS})

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


SET( PROJ_LIBRARIES "" )
SET( PROJ_INCLUDES  "${PROJECT_PATH}/include/" )

PROJECT( ${PROJ_NAME} )

FIND_PACKAGE(OpenMP)
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
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
ENDIF()

ADD_LIBRARY("${PROJ_NAME}_data" ${DATA_HEADERS} ${DATA_SOURCES})
ADD_LIBRARY("${PROJ_NAME}_core" ${CORE_HEADERS} ${CORE_SOURCES})
ADD_LIBRARY("${PROJ_NAME}_utilities" ${UTILITIES_HEADERS} ${UTILITIES_SOURCES})

ADD_EXECUTABLE(${PROJ_NAME} "main.cpp"   )
ADD_EXECUTABLE("${PROJ_NAME}_Visualization" "src/Apps/Visualization/Visualization.cpp")
ADD_EXECUTABLE("${PROJ_NAME}_Learning" "src/Apps/Learning/Learning.cpp")
ADD_EXECUTABLE("${PROJ_NAME}_Inference" "src/Apps/Inference/Inference.cpp")

#TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" "${PROJ_NAME}_utilities")
TARGET_LINK_LIBRARIES( "${PROJ_NAME}_Visualization" ${PROJ_LIBRARIES} "${PROJ_NAME}_data" "${PROJ_NAME}_core" "${PROJ_NAME}_utilities")

