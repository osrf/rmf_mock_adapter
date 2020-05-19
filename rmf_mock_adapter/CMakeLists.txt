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

add_library(rmf_mock_adapter SHARED src/adapter.cpp src/Transport.cpp)

target_link_libraries(rmf_mock_adapter
  PUBLIC
    rmf_traffic::rmf_traffic
)

target_include_directories(rmf_mock_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/RxCpp-4.1.0/Rx/v2/src>
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

if(BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(rmf_utils REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_catch2(
    rmf_rx_jobs_test
      test/main.cpp
      test/RxJobsTest.cpp
      test/TransportTest.cpp
      test/jobs/FollowPath.cpp
  )
  target_include_directories(rmf_rx_jobs_test
    PRIVATE
      ${std_msgs_INCLUDE_DIRS}
  )
  target_link_libraries(rmf_rx_jobs_test
    PRIVATE
      rmf_mock_adapter
      rmf_utils::rmf_utils
      ${std_msgs_LIBRARIES}
  )
endif()

ament_package()
