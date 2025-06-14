cmake_minimum_required(VERSION 3.5)
project(diffdrive_arduino LANGUAGES CXX)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# Link to libserial.so manually
link_directories(/usr/lib/x86_64-linux-gnu)  # ✅ đúng thư mục libserial của bạn

# Main plugin library
add_library(diffdrive_arduino SHARED
  src/diffdrive_arduino.cpp
)

target_include_directories(diffdrive_arduino PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  diffdrive_arduino
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

# ✅ Link libserial thủ công (chữ thường)
target_link_libraries(diffdrive_arduino serial)

# Plugin export
pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch + config
install(DIRECTORY launch controllers
  DESTINATION share/${PROJECT_NAME}
)

# Install library
install(TARGETS diffdrive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export for downstream
ament_export_libraries(diffdrive_arduino)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

ament_package()
