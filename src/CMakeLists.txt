# add_library(meshtools STATIC border.cpp surface.cpp volume.cpp normal.cpp reader.cpp)
add_library(playtectools STATIC 
    playtec/base.cpp    
    playtec/border.cpp 
    playtec/surface.cpp 
    playtec/volume.cpp 
    playtec/normal.cpp 
    playtec/reader.cpp
)

# target_link_libraries(playtectools INTERFACE tinyply)
