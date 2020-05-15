cmake_minimum_required(VERSION 3.5)

project(rmf_mock_adapter)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

find_package(ament_cmake REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

find_package(rmf_traffic REQUIRED)

add_library(rmf_mock_adapter SHARED src/adapter.cpp)

target_link_libraries(rmf_mock_adapter
  PUBLIC
    rmf_traffic::rmf_traffic
)

target_include_directories(rmf_mock_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_export_interfaces(rmf_mock_adapter HAS_LIBRARY_TARGET)
ament_export_dependencies(rmf_traffic)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS rmf_mock_adapter
  EXPORT  rmf_mock_adapter
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

add_executable(test_fleet_adapter test/test_fleet_adapter.cpp)
target_link_libraries(test_fleet_adapter
  PRIVATE
    rmf_mock_adapter
)

ament_package()
