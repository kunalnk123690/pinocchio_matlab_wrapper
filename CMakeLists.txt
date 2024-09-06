cmake_minimum_required(VERSION 3.22)
project(pin)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(Matlab COMPONENTS MX_LIBRARY)
find_package(pinocchio REQUIRED)

include_directories(include)

matlab_get_release_name_from_version(${Matlab_VERSION_STRING} release)
message(STATUS "Matlab release: ${release}")

file(GLOB SRC_FILES src/*.cpp)
matlab_add_mex(NAME ${PROJECT_NAME} SHARED SRC ${SRC_FILES} 
LINK_TO Eigen3::Eigen pinocchio::pinocchio 
)