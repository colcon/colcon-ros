cmake_minimum_required(VERSION 2.8.3)
project(test-nodes-deps)

set (CMAKE_CXX_STANDARD 11)

find_package(AWSSDK)
set(AWS_DEPS monitoring)
AWSSDK_DETERMINE_LIBS_TO_LINK(AWS_DEPS AWS_LIBS)
link_directories("${AWSSDK_LIB_DIR}")

find_library(NODES_LIB helloworld_dynamic_lib PATHS "${CMAKE_INSTALL_PREFIX}/../test-nodes/lib")

add_executable(helloworld_cmake src/helloworld_cmake.cpp)
target_link_libraries(helloworld_cmake ${AWS_LIBS} ${NODES_LIB})

# Mark executables and/or libraries for installation
install(TARGETS helloworld_cmake
  RUNTIME DESTINATION bin
)