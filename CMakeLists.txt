cmake_minimum_required(VERSION 3.22)
project(mapskit LANGUAGES CXX)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose build type: Debug, Release, RelWithDebInfo, MinSizeRel." FORCE)
endif()


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set compiler flags based on build type
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Common flags for all builds
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

  # Release flags (optimizations, no debug info)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

  # Debug flags (debug info, no optimizations)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

set(PYBIND11_FINDPYTHON ON)
find_package(Python 3.10 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Get the CMAKE_PREFIX_PATH value (or another variable)
# Note: The value might be a list of paths, so this approach is best for a single path
# or a joined list if your application can handle parsing a string list.
string(JOIN ";" PREFIX_PATHS_STR "${CMAKE_PREFIX_PATH}")

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  # add_executable(mapskit_core_cxx_debug ${SOURCES})
  # ament_target_dependencies(mapskit_core_cxx_debug rclcpp)
  # target_link_libraries(mapskit_core_cxx_debug pybind11::embed)

  add_executable(test_preproc src/cpp_node.cpp)
  target_compile_definitions(test_preproc PRIVATE 
      "MY_APP_PREFIX_PATH=\"${CMAKE_INSTALL_PREFIX}\""
  )

  install(TARGETS
    test_preproc
    DESTINATION lib/${PROJECT_NAME}
  )
else() # Release, RelWithDebInfo, MinSizeRel

  ######################### Install cxx modules ################################                              
  # Include Cpp "include" directory
  include_directories(include)

  # Create Cpp executable
  set(mapskit_core_sources src/mapskit.cpp)
  file(GLOB SOURCES "src/*.cpp") # USE LATER FOR OTHER THINGS ? 

add_executable(mapskit_core_cxx_release ${SOURCES})
  ament_target_dependencies(mapskit_core_cxx_release rclcpp)
  target_link_libraries(mapskit_core_cxx_release pybind11::embed)
  install(TARGETS
    mapskit_core_cxx_release
    DESTINATION lib/${PROJECT_NAME}
  )
  ######################### Install Python modules #############################                             
  ament_python_install_package(${PROJECT_NAME})
  pybind11_add_module(mapskit_core_py SHARED ${mapskit_core_sources})

  # Install Python executables
  install(TARGETS
    mapskit_core_py
    DESTINATION  ${Python_SITELIB}
  )

  install(PROGRAMS
    scripts/py_node.py
    DESTINATION lib/${PROJECT_NAME}
  )


endif()



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
