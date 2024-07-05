cmake_minimum_required(VERSION 3.22)
project(pin)

find_package(Eigen3 REQUIRED)
find_package(Matlab REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include)

matlab_get_release_name_from_version(${Matlab_VERSION_STRING} release)
message(STATUS "Matlab release: ${release}")

file(GLOB SRC_FILES src/*.cpp)
matlab_add_mex(NAME ${PROJECT_NAME} SHARED SRC ${SRC_FILES} LINK_TO pinocchio::pinocchio Eigen3::Eigen)