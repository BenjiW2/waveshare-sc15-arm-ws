cmake_minimum_required(VERSION 3.10)
project(sc15_hw_iface_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# library
add_library(sc15_bus SHARED src/sc15_bus.cpp)
target_include_directories(sc15_bus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(sc15_bus
  rclcpp hardware_interface pluginlib)

# headers
install(DIRECTORY include/
        DESTINATION include)

install(TARGETS sc15_bus
        LIBRARY DESTINATION lib)

install(FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

ament_package()
