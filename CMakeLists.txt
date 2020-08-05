cmake_minimum_required(VERSION 3.0)
project(ltr_volume)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## IF NOT DEFINED SRC
# add_library(meshtools STATIC IMPORTED)
# set_property(TARGET meshtools PROPERTY IMPORTED_LOCATION ${LIBRARY_OUTPUT_PATH}/libmeshtools.a)
add_subdirectory(src)
 

