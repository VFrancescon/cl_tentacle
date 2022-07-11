cmake_minimum_required (VERSION 3.10)
project (cl_tentacle)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(NLOPT QUIET)
if (NOT ${NLOPT_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindNLOPT.cmake")
endif()


add_executable (main "src/main.cpp")
add_executable( nlopt "src/nlopt_test.cpp")

target_link_libraries (main Eigen3::Eigen)
target_link_libraries(nlopt Eigen3::Eigen ${NLOPT_LIBRARIES})