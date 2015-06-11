CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

SET( PROJ_NAME      "HOP3D" )
SET( PROJ_PATH      ${CMAKE_SOURCE_DIR} )
SET( PROJ_OUT_PATH  ${CMAKE_BINARY_DIR} )
#header files added to sources to display include files in QtCreator
SET( PROJ_SOURCES   "main.cpp" "src/GraphBuilder.cpp" "src/LayerFilter.cpp" "src/LayerVocabulary.cpp" "src/LocalInhibitor.cpp" "src/ObjectCloud.cpp" "src/ObjectGraph.cpp" "src/ObjectReader.cpp" "src/ObjectWriter.cpp" "src/Part.cpp" "src/PartRealization.cpp" "src/PartSelector.cpp" "src/StatisticsBuilder.cpp" "src/Vocabulary.cpp" "include/GraphBuilder.h" "include/LayerFilter.h" "include/LayerVocabulary.h" "include/LocalInhibitor.h" "include/ObjectCloud.h" "include/ObjectGraph.h" "include/ObjectReader.h" "include/ObjectWriter.h" "include/Part.h" "include/PartRealization.h" "include/PartSelector.h" "include/StatisticsBuilder.h" "include/Vocabulary.h")
SET( PROJ_HEADERS   "include/GraphBuilder.h" "include/LayerFilter.h" "include/LayerVocabulary.h" "include/LocalInhibitor.h" "include/ObjectCloud.h" "include/ObjectGraph.h" "include/ObjectReader.h" "include/ObjectWriter.h" "include/Part.h" "include/PartRealization.h" "include/PartSelector.h" "include/StatisticsBuilder.h" "include/Vocabulary.h")
SET( PROJ_LIBRARIES "" )
SET( PROJ_INCLUDES  "include/" )

PROJECT( ${PROJ_NAME} )

FIND_PACKAGE(OpenMP)
IF (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( Boost 1.54 COMPONENTS system filesystem REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES( ${PROJ_INCLUDES} )
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
ADD_EXECUTABLE(${PROJ_NAME} ${PROJ_SOURCES})
TARGET_LINK_LIBRARIES( ${PROJ_NAME} ${PROJ_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

