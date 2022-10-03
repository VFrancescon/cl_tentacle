cmake_minimum_required (VERSION 3.10)
project (cl_tentacle)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# find_package(NLOPT QUIET)
# if (NOT ${NLOPT_FOUND})
#     include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindNLOPT.cmake")
# endif()

# include_directories(${NLOPT_INCLUDE_DIRS})

add_executable (main "src/main.cpp")

target_link_libraries (main Eigen3::Eigen)

file(GLOB_RECURSE DataTypes
        "${CMAKE_SOURCE_DIR}/src/DataTypes.hpp")


install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/DataTypes DESTINATION include)