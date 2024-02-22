cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(railtrack_ui VERSION 2.8.1)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


add_custom_target(
 run ALL
 COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_config_paths.py ${CMAKE_CURRENT_SOURCE_DIR}
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME})


install(PROGRAMS
        railtrack_ui/railtrack_ui.py
        railtrack_ui/railtrack_emulator.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()

