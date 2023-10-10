add_library(mysolver SHARED
        vertex.cpp
        edge.cpp
        solver.cpp
)


target_link_libraries(mysolver ${THIRD_PARTY_LIBS})