cmake_minimum_required(VERSION 3.8)
project(mine_detector)

# Enforce C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Find ROS 2 Packages ───
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ─── Find System Libraries ───
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)

# ─── Robust GPIO Discovery (Manual fallback if pkg-config fails) ───
pkg_check_modules(GPIODCXX libgpiodcxx)

if(NOT GPIODCXX_FOUND)
    message(STATUS "pkg-config failed to find libgpiodcxx, searching manually...")
    find_path(GPIODCXX_INCLUDE_DIRS NAMES gpiod.hpp)
    find_library(GPIODCXX_LIBRARIES NAMES gpiodcxx)
    
    if(GPIODCXX_INCLUDE_DIRS AND GPIODCXX_LIBRARIES)
        set(GPIODCXX_FOUND TRUE)
        message(STATUS "Manual find success: Found gpiod.hpp in ${GPIODCXX_INCLUDE_DIRS}")
    else()
        message(FATAL_ERROR "libgpiodcxx not found! Install it with: sudo apt install libgpiodcxx-dev")
    endif()
endif()

# ─── Build mine_recorder Node ───
add_executable(mine_recorder src/mine_recorder.cpp)

target_include_directories(mine_recorder PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Eigen3_INCLUDE_DIRS}
  ${GPIODCXX_INCLUDE_DIRS}
)

ament_target_dependencies(mine_recorder
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(mine_recorder
  Eigen3::Eigen
  ${GPIODCXX_LIBRARIES}
)

# ─── Find libiio for mine_detector ───
pkg_check_modules(IIO libiio)

if(IIO_FOUND)
  add_executable(mine_detector src/port.cpp)
  
  target_include_directories(mine_detector PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${IIO_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
  )

  ament_target_dependencies(mine_detector
    rclcpp
    std_msgs
    geometry_msgs
  )

  target_link_libraries(mine_detector
    ${IIO_LIBRARIES}
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
  )
else()
  message(WARNING "mine_detector (port.cpp) will NOT be built - libiio missing")
endif()

# ─── Installation ───
if(IIO_FOUND)
    install(TARGETS mine_detector mine_recorder DESTINATION lib/${PROJECT_NAME})
else()
    install(TARGETS mine_recorder DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()