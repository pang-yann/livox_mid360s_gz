# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 pang-yann

cmake_minimum_required(VERSION 3.16)
project(livox_mid360s_gz_demos)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
