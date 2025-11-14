# mapskit

`mapskit` package responsible for both C++ (`.cpp`) and Python (`.py`) nodes that interact with the ompl and octomap protocol.

This package uses a combination of `ament_cmake` for C++ components and `ament_cmake_python` for Python nodes, allowing for a unified build process managed by CMake.

## Adding a new Python Node

To add a new Python node to `mapskit`, follow these steps:

1.  **Create your Python script**

    Create your Python script inside the `scripts` directory (e.g., `scripts/my_new_node.py`).

2.  **Add a Shebang**

    Add the following shebang at the very top of your Python script. This ensures the script is executed with the correct Python interpreter.

    ```python
    #!/usr/bin/env python3
    ```

3.  **Make the script executable**

    Before committing your file, make it executable using the following command:

    ```bash
    chmod +x scripts/my_new_node.py
    ```

4.  **Update `CMakeLists.txt`**

    To ensure your new Python node is installed correctly, add it to the `install(PROGRAMS ...)` section in your `CMakeLists.txt` file. This will install the script to the `lib/${PROJECT_NAME}` directory, making it available as an executable.

    ```cmake
    install(PROGRAMS
      scripts/py_node.py
      scripts/my_new_node.py # <-- Add your new node here
      DESTINATION lib/${PROJECT_NAME}
    )
    ```

By following these steps, your new Python node will be properly integrated into the `mapskit` package and built alongside the C++ components using `ament_cmake`.

## Adding a new C++ Node

To add a new C++ node to `mapskit`, follow these steps:

1.  **Create your C++ source and header files**

    Create your header file inside the `include/mapskit` directory (e.g., `include/mapskit/my_new_node.hpp`) and your source file inside the `src` directory (e.g., `src/my_new_node.cpp`).

2.  **Include the header in your source file**

    In your `.cpp` file, include the corresponding header. The `mapskit` package is configured to find headers from the `include` directory.

    ```cpp
    #include "mapskit/my_new_node.hpp"
    // ... rest of your node code
    ```

3.  **Update `CMakeLists.txt`**

    To build and install your new C++ node, you need to add it to the `CMakeLists.txt` file.

    First, define the new executable and link its dependencies:
    ```cmake
    # Add your new executable
    add_executable(my_new_node_executable src/my_new_node.cpp)

    # Link against required libraries (e.g., rclcpp)
    ament_target_dependencies(my_new_node_executable rclcpp)
    ```

    Next, add the new executable target to the `install(TARGETS ...)` section. This will install the compiled binary to the `lib/${PROJECT_NAME}` directory.

    ```cmake
    install(TARGETS
      cpp_executable
      my_new_node_executable # <-- Add your new executable here
      DESTINATION lib/${PROJECT_NAME}
    )
    ```
