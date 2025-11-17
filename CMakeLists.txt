cmake_minimum_required(VERSION 3.22)
project(mapskit LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

set(PYBIND11_FINDPYTHON ON)
find_package(Python 3.10 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

######################### Install cxx modules ##################################                              
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
file(GLOB SOURCES "src/*.cpp")
add_executable(mapskit_cxx ${SOURCES})
ament_target_dependencies(mapskit_cxx rclcpp)
target_link_libraries(mapskit_cxx pybind11::embed)

# Install Cpp executables
install(TARGETS
  mapskit_cxx
  DESTINATION lib/${PROJECT_NAME}
)


######################### Install Python modules ###############################                              
# ament_python_install_package(${PROJECT_NAME})

# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                 [NO_EXTRAS] [THIN_LTO] [OPT_SIZE] source1 [source2 ...])


set(py_sources src/mapskit.cpp)
pybind11_add_module(mapskit SHARED ${py_sources})

# Install Python executables
install(TARGETS
  mapskit
  DESTINATION  ${Python_SITELIB}
)

install(PROGRAMS
  scripts/py_node.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
