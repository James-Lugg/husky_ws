cmake_minimum_required(VERSION 3.14)
project(husky_essentials)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

pybind11_add_module(a200_serial_pybind
  src/a200_serial_pybind.cpp
  src/crc.cpp
  src/horizon_legacy_wrapper.cpp
  src/linux_serial.cpp
  src/Logger.cpp
  src/Message.cpp
  src/Message_cmd.cpp
  src/Message_data.cpp
  src/Message_request.cpp
  src/Number.cpp
  src/Transport.cpp
)

target_include_directories(a200_serial_pybind PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(a200_serial_pybind PRIVATE pthread)
