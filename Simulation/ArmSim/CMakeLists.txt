cmake_minimum_required(VERSION 3.16)
project(armsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo SDK 根目录。Linux 一般是 /opt/mujoco；
# Windows 解压 mujoco-3.x.x-windows-x86_64 后指向该目录（含 include/ lib/ bin/）。
set(MUJOCO_ROOT "/opt/mujoco" CACHE PATH "MuJoCo SDK root")

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
          HINTS "${MUJOCO_ROOT}/include" REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco
             HINTS "${MUJOCO_ROOT}/lib" REQUIRED)

add_executable(armsim
    main.cpp
    Algorithm/pid.cpp
    Algorithm/dh.cpp
    Algorithm/gravity.cpp
    Driver/motor.cpp
    Driver/host_link.cpp
    Module/arm.cpp
    Module/host.cpp
    System/sim_plant.cpp
    System/system.cpp
)

target_include_directories(armsim PRIVATE
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/Algorithm"
    "${CMAKE_CURRENT_SOURCE_DIR}/Driver"
    "${CMAKE_CURRENT_SOURCE_DIR}/Module"
    "${CMAKE_CURRENT_SOURCE_DIR}/System"
)

target_compile_definitions(armsim PRIVATE
    ARMSIM_DEFAULT_XML="${CMAKE_CURRENT_SOURCE_DIR}/../mujoco/DIY/robot.xml")

target_compile_options(armsim PRIVATE -Wall -Wextra)

target_link_libraries(armsim PRIVATE "${MUJOCO_LIBRARY}")

# 让可执行文件直接找到 libmujoco.so
if(UNIX AND NOT APPLE)
  set_target_properties(armsim PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}/lib")
endif()
