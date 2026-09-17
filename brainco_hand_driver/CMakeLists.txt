cmake_minimum_required(VERSION 3.16)
project(brainco_hand_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(STARK_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/dist/include")
set(STARK_SDK_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/dist/shared/linux")
set(STARK_SDK_LIBRARY "${STARK_SDK_LIBRARY_DIR}/libbc_stark_sdk.so")

if(NOT EXISTS "${STARK_SDK_INCLUDE_DIR}/stark-sdk.h")
  message(FATAL_ERROR "Missing Stark SDK header: ${STARK_SDK_INCLUDE_DIR}/stark-sdk.h")
endif()
if(NOT EXISTS "${STARK_SDK_LIBRARY}")
  message(FATAL_ERROR "Missing Stark SDK library: ${STARK_SDK_LIBRARY}")
endif()

add_library(brainco_stark_sdk SHARED IMPORTED GLOBAL)
set_target_properties(brainco_stark_sdk PROPERTIES
  IMPORTED_LOCATION "${STARK_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${STARK_SDK_INCLUDE_DIR}"
)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/brainco_hand_api.cpp
  src/brainco_hand_hardware.cpp
  src/modbus_session.cpp
  src/sdk_helpers.cpp
  src/transport_session_base.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME} brainco_stark_sdk)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${STARK_SDK_LIBRARY_DIR}"
  INSTALL_RPATH "\$ORIGIN"
)

pluginlib_export_plugin_description_file(
  hardware_interface brainco_hand_driver_plugins.xml
)

install(DIRECTORY include/ DESTINATION include)
install(FILES "${STARK_SDK_LIBRARY}" DESTINATION lib)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES brainco_hand_driver_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE vendor/dist/include/stark-sdk.h)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_revo1_configuration
    test/test_revo1_configuration.py
    TIMEOUT 30
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
