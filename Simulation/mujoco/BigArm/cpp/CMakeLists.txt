cmake_minimum_required(VERSION 3.16)
project(bigarm_ctrl CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo SDK root.  Linux: /opt/mujoco.  Windows: the extracted
# mujoco-3.x.x-windows-x86_64 folder (contains include/, lib/, bin/).
set(MUJOCO_ROOT "/opt/mujoco" CACHE PATH "MuJoCo SDK root")

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
          HINTS "${MUJOCO_ROOT}/include" REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco
             HINTS "${MUJOCO_ROOT}/lib" REQUIRED)

add_executable(bigarm_ctrl main.cpp)
target_include_directories(bigarm_ctrl PRIVATE "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(bigarm_ctrl PRIVATE "${MUJOCO_LIBRARY}")

# Let the binary find libmujoco.so without installing it system-wide.
if(UNIX AND NOT APPLE)
  set_target_properties(bigarm_ctrl PROPERTIES
                        BUILD_RPATH "${MUJOCO_ROOT}/lib")
endif()
