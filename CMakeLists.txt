cmake_minimum_required(VERSION 3.10)
project(BA)

set(CMAKE_BUILD_TYPE  Debug)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_CXX_STANDARD 11)
find_package(G2O REQUIRED)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
message(${Sophus_INCLUDE_DIRS})

include_directories(${G2O_INCLUDE_DIR}
        "usr/include/eigen3")
#message(${G2O_INCLUDE_DIR})





add_executable(BA main.cpp)
target_link_libraries(BA "/usr/local/lib/libg2o_core.so"
        "/usr/local/lib/libg2o_stuff.so"

         ${Sophus_LIBRARIES})
