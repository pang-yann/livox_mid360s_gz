cmake_minimum_required(VERSION 3.16)
project(livox_mid360s_gz)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(livox_mid360s_core
  src/point_cloud_sampler.cpp
  src/scan_pattern.cpp
)
set_target_properties(livox_mid360s_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(livox_mid360s_core PUBLIC cxx_std_17)
target_include_directories(livox_mid360s_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(livox_mid360s_core
  PUBLIC
    gz-msgs10::gz-msgs10
)

add_library(livox_mid360s_system SHARED
  src/livox_mid360s_system.cpp
)
target_compile_features(livox_mid360s_system PUBLIC cxx_std_17)
target_link_libraries(livox_mid360s_system
  PRIVATE
    livox_mid360s_core
    ament_index_cpp::ament_index_cpp
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-plugin2::gz-plugin2-register
)
set_target_properties(livox_mid360s_system PROPERTIES
  OUTPUT_NAME livox_mid360s_gz_system
)

install(
  TARGETS livox_mid360s_core livox_mid360s_system
  EXPORT export_livox_mid360s_gz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch models scan_patterns worlds
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_livox_mid360s_gz HAS_LIBRARY_TARGET)
ament_export_dependencies(gz_msgs_vendor gz-msgs10)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_scan_pattern test/test_scan_pattern.cpp)
  target_link_libraries(test_scan_pattern livox_mid360s_core)
  target_compile_definitions(test_scan_pattern PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
  )

  ament_add_gtest(test_point_cloud_sampler test/test_point_cloud_sampler.cpp)
  target_link_libraries(test_point_cloud_sampler livox_mid360s_core)
  target_compile_definitions(test_point_cloud_sampler PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
  )
endif()

ament_package()
