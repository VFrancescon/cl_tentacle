cmake_minimum_required (VERSION 3.10)
project (cl_tentacle)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (main "src/main.cpp")
add_executable( eigen_test "src/eigen_test.cpp")
target_link_libraries (main Eigen3::Eigen)
target_link_libraries (eigen_test Eigen3::Eigen)