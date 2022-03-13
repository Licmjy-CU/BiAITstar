ADD_LIBRARY(${PROJECT_NAME} SHARED
        include/BiAIT.h
        include/Edge.h
        include/GenerateId.h
        include/ImplicitGraph.h
        include/Queuetypes.h
        include/Utility.h
        include/Vertex.h
        include/WeakEdge.h

        src/BiAIT.cpp
        src/Edge.cpp
        src/ImplicitGraph.cpp
        src/Utility.cpp
        src/Vertex.cpp
        src/WeakEdge.cpp
        )


TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        ${OMPL_LIBRARIES}
        )

