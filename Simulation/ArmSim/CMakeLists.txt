cmake_minimum_required(VERSION 3.16)
project(armsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 每次 configure 都生成 build/compile_commands.json，供 clangd 做代码跳转
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MuJoCo SDK 根目录。Linux 一般是 /opt/mujoco；
# Windows 解压 mujoco-3.x.x-windows-x86_64 后指向该目录（含 include/ lib/ bin/）。
set(MUJOCO_ROOT "/opt/mujoco" CACHE PATH "MuJoCo SDK root")

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
          HINTS "${MUJOCO_ROOT}/include" REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco
             HINTS "${MUJOCO_ROOT}/lib" REQUIRED)

# 可选：MuJoCo 窗口。优先用系统的 GLFW（apt install libglfw3-dev），
# 其次找 GLFW_ROOT（含 include/GLFW/glfw3.h 和 lib/libglfw3.a），都没有就无窗口构建。
set(GLFW_ROOT "" CACHE PATH "GLFW 根目录（可选）")
set(ARMSIM_GLFW_HINTS "${GLFW_ROOT}" "$ENV{HOME}/software/mujoco/build")
find_library(MUJOCO_SIMULATE_LIBRARY NAMES simulate
             HINTS "${MUJOCO_ROOT}/lib")
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
  find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h
            HINTS ${ARMSIM_GLFW_HINTS}
            PATH_SUFFIXES include _deps/glfw3-src/include)
  find_library(GLFW_LIBRARY NAMES glfw3 glfw
               HINTS ${ARMSIM_GLFW_HINTS} PATH_SUFFIXES lib)
endif()

add_executable(armsim
    main.cpp
    Algorithm/pid.cpp
    Algorithm/kinematics.cpp
    Algorithm/ik.cpp
    Algorithm/ik_cost.cpp
    Algorithm/gravity.cpp
    Driver/mujoco_world.cpp
    Driver/host_link.cpp
    Module/arm.cpp
    Module/exchange_station.cpp
    Module/host.cpp
)

target_include_directories(armsim PRIVATE
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/Algorithm"
    "${CMAKE_CURRENT_SOURCE_DIR}/Driver"
    "${CMAKE_CURRENT_SOURCE_DIR}/Module"
)

target_compile_definitions(armsim PRIVATE
    ARMSIM_DEFAULT_XML="${CMAKE_CURRENT_SOURCE_DIR}/../mujoco/BigArm/robot.xml")

set(ARMSIM_DEFAULT_EXCHANGE
    "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco/scene/exchange_station.xml"
    CACHE FILEPATH "Exchange station MJCF")
get_filename_component(ARMSIM_DEFAULT_EXCHANGE "${ARMSIM_DEFAULT_EXCHANGE}" ABSOLUTE)
if(EXISTS "${ARMSIM_DEFAULT_EXCHANGE}")
  target_compile_definitions(armsim PRIVATE
      ARMSIM_DEFAULT_EXCHANGE="${ARMSIM_DEFAULT_EXCHANGE}")
else()
  target_compile_definitions(armsim PRIVATE ARMSIM_DEFAULT_EXCHANGE="")
  message(WARNING "找不到兑换站模型: ${ARMSIM_DEFAULT_EXCHANGE}")
endif()

# 默认使用普通无限地面，机械臂和兑换站作为附件合并进去。
# 如需其他场地，运行时传 --arena <xml>，不再隐式加载 2027 场地。
set(ARMSIM_DEFAULT_ARENA
    "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco/scene/plain_floor.xml"
    CACHE FILEPATH "Default arena MJCF")
get_filename_component(ARMSIM_DEFAULT_ARENA "${ARMSIM_DEFAULT_ARENA}" ABSOLUTE)
if(EXISTS "${ARMSIM_DEFAULT_ARENA}")
  target_compile_definitions(armsim PRIVATE
      ARMSIM_DEFAULT_ARENA="${ARMSIM_DEFAULT_ARENA}")
  message(STATUS "arena: ${ARMSIM_DEFAULT_ARENA}")
else()
  target_compile_definitions(armsim PRIVATE ARMSIM_DEFAULT_ARENA="")
  message(WARNING "找不到默认地面: ${ARMSIM_DEFAULT_ARENA}")
endif()

target_compile_definitions(armsim PRIVATE ARMSIM_DEFAULT_VIEW_ARENA="")

if(MSVC)
  # 源码是 UTF-8（含中文字符串），不加这个 MSVC 会按本地代码页读
  target_compile_options(armsim PRIVATE /W3 /utf-8)
else()
  target_compile_options(armsim PRIVATE -Wall -Wextra)
endif()

target_link_libraries(armsim PRIVATE "${MUJOCO_LIBRARY}")

# 窗口用 MuJoCo 官方 simulate（SDK 自带 libsimulate.a），需要它和 GLFW 都找到
set(ARMSIM_VIEWER OFF)
if(MUJOCO_SIMULATE_LIBRARY AND EXISTS "${MUJOCO_INCLUDE_DIR}/simulate/simulate.h")
  if(glfw3_FOUND)
    target_link_libraries(armsim PRIVATE "${MUJOCO_SIMULATE_LIBRARY}" glfw)
    set(ARMSIM_VIEWER ON)
    message(STATUS "viewer: simulate + 系统 GLFW")
  elseif(GLFW_INCLUDE_DIR AND GLFW_LIBRARY)
    target_include_directories(armsim PRIVATE "${GLFW_INCLUDE_DIR}")
    target_link_libraries(armsim PRIVATE "${MUJOCO_SIMULATE_LIBRARY}" "${GLFW_LIBRARY}"
                          GL ${CMAKE_DL_LIBS} pthread X11)
    set(ARMSIM_VIEWER ON)
    message(STATUS "viewer: simulate + ${GLFW_LIBRARY}")
  endif()
endif()
if(ARMSIM_VIEWER)
  target_sources(armsim PRIVATE Driver/viewer.cpp)
  if(NOT MSVC)
    # simulate.h 是 MuJoCo 的头文件，-Wextra 会对它报缺字段初始化
    set_source_files_properties(Driver/viewer.cpp PROPERTIES COMPILE_OPTIONS -Wno-missing-field-initializers)
  endif()
  target_compile_definitions(armsim PRIVATE ARMSIM_HAS_VIEWER)
else()
  message(STATUS "viewer: 缺 libsimulate 或 GLFW，无窗口构建（apt install libglfw3-dev 后重新配置）")
endif()

# 让可执行文件直接找到 libmujoco.so
if(UNIX AND NOT APPLE)
  set_target_properties(armsim PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}/lib")
endif()

# Windows 没有 rpath：把 mujoco.dll 复制到可执行文件旁边
if(WIN32)
  find_file(MUJOCO_DLL NAMES mujoco.dll HINTS "${MUJOCO_ROOT}/bin")
  if(MUJOCO_DLL)
    add_custom_command(TARGET armsim POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${MUJOCO_DLL}" "$<TARGET_FILE_DIR:armsim>")
  else()
    message(WARNING "找不到 mujoco.dll（${MUJOCO_ROOT}/bin），运行时需要把它放到 PATH 里")
  endif()
endif()
