CMAKE_MINIMUM_REQUIRED(VERSION 3.21)
PROJECT(BiAIT_Release)

SET(CMAKE_CXX_STANDARD 20)

#SET(IS_DEBUG ON)
SET(IS_DEBUG OFF)
IF(IS_DEBUG)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated-anon-enum-enum-conversion")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-anon-enum-enum-conversion")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdeprecated-anon-enum-enum-conversion")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-enum-enum-conversion")
ELSE()
    set(RELEASE_FLAGS "-Ofast -DNDEBUG -DBOOST_DISABLE_ASSERTS")
    set(DEBUG_FLAGS "-O0 -Wno-deprecated-anon-enum-enum-conversion")

    SET(CMAKE_CXX_FLAGS_RELEASE ${RELEASE_FLAGS})
    SET(CMAKE_C_FLAGS_RELEASE ${RELEASE_FLAGS})

    SET(CMAKE_CXX_FLAGS_DEBUG ${DEBUG_FLAGS})
    SET(CMAKE_C_FLAGS_DEBUG ${DEBUG_FLAGS})
ENDIF()


MESSAGE("CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS})
MESSAGE("CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
MESSAGE("CMAKE_C_FLAGS_RELEASE: " ${CMAKE_C_FLAGS_RELEASE})
MESSAGE("CMAKE_CXX_FLAGS_RELEASE: " ${CMAKE_CXX_FLAGS_RELEASE})
MESSAGE("CMAKE_C_FLAGS_DEBUG: " ${CMAKE_C_FLAGS_DEBUG})
MESSAGE("CMAKE_CXX_FLAGS_DEBUG: " ${CMAKE_CXX_FLAGS_DEBUG})


SET(IS_TEST ON)

MESSAGE("OMPL:")
FIND_PACKAGE(ompl REQUIRED)
MESSAGE(STATUS "    OMPL_VERSION: " ${OMPL_VERSION})
MESSAGE(STATUS "    OMPL_LIBRARIES: "  ${OMPL_LIBRARIES})
MESSAGE(STATUS "    OMPLAPP_LIBRARIES: "  ${OMPLAPP_LIBRARIES})

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}
        ${OMPL_INCLUDE_DIRS}
)

ADD_LIBRARY(${PROJECT_NAME} SHARED
        BiAITstar/BiAIT.h
        BiAITstar/Edge.h
        BiAITstar/GenerateId.h
        BiAITstar/ImplicitGraph.h
        BiAITstar/Queuetypes.h
        BiAITstar/Utility.h
        BiAITstar/Vertex.h
        BiAITstar/WeakEdge.h

        BiAITstar/src/BiAIT.cpp
        BiAITstar/src/Edge.cpp
        BiAITstar/src/ImplicitGraph.cpp
        BiAITstar/src/Utility.cpp
        BiAITstar/src/Vertex.cpp
        BiAITstar/src/WeakEdge.cpp
        )


TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        ${OMPL_LIBRARIES}
        )


MESSAGE("TEST_STATE: " ${IS_TEST})
IF(${IS_TEST})
    ADD_EXECUTABLE(testRelease main.cpp)

    TARGET_LINK_LIBRARIES(testRelease
            ${PROJECT_NAME}
            ${OMPLAPP_LIBRARIES}
            )
ENDIF()
