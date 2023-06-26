cmake_minimum_required(VERSION 3.16)

project(serial-cpp)

# The libserial-cpp library itself.
add_library(${PROJECT_NAME}
  STATIC
  include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
  include/${PROJECT_NAME}/${PROJECT_NAME}-utils.hpp
  src/listPorts.cpp
  src/SerialPortHandler.cpp
  src/SerialDevice.cpp
  src/utils.cpp
)

find_package(Boost 1.81 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
find_package(Qt6 REQUIRED COMPONENTS SerialPort)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
)

target_compile_features(${PROJECT_NAME}
  PUBLIC
  cxx_std_17
)

target_compile_options(${PROJECT_NAME} PUBLIC
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -pedantic -Werror>
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  $<$<CXX_COMPILER_ID:MSVC>:WIN32_LEAN_AND_MEAN;UNICODE;_CRT_SECURE_NO_WARNINGS>
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:Ws2_32>
  Qt6::SerialPort
)

# Common testing options.
enable_testing()
