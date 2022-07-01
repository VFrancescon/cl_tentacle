cmake_minimum_required (VERSION 3.10)
project (cl_tentacle)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (main "src/main.cpp")
target_link_libraries (main Eigen3::Eigen)
