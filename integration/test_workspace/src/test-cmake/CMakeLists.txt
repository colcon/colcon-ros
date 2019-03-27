cmake_minimum_required(VERSION 2.8.3)
project(test-cmake)

set (CMAKE_CXX_STANDARD 11)

find_package(AWSSDK)
set(AWS_DEPS monitoring)
AWSSDK_DETERMINE_LIBS_TO_LINK(AWS_DEPS AWS_LIBS)
link_directories("${AWSSDK_LIB_DIR}")

add_executable(helloworld_cmake src/helloworld_cmake.cpp)
target_link_libraries(helloworld_cmake ${AWS_LIBS})

add_library(helloworld_cmake_lib src/helloworld_cmake.cpp)
target_link_libraries(helloworld_cmake_lib ${AWS_LIBS})

# Mark executables and/or libraries for installation
install(TARGETS helloworld_cmake helloworld_cmake_lib
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)