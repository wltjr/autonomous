################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.20)
project(autonomous)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
set(INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    rclcpp_components
    sensor_msgs
)

foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()


################################################################################
# Build
################################################################################
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# shared object libraries, lib*.so
add_library(autonomy SHARED src/autonomy.cpp)

# executable binaries
add_executable(autonomy_node src/autonomy_node.cpp)

# add ROS dependencies
ament_target_dependencies(autonomy ${INCLUDE_DEPENDS})

# link executable binaries to shared object libraries
target_link_libraries(autonomy_node autonomy)

# register ROS nodes
rclcpp_components_register_nodes(autonomy "autonomous::Autonomy")


################################################################################
# Install
################################################################################
install(
    DIRECTORY launch2
    DESTINATION share/${PROJECT_NAME}
)

set(INSTALL_TARGETS
    autonomy
    autonomy_node
)

foreach(target IN ITEMS ${INSTALL_TARGETS})
    install(TARGETS ${target}
        EXPORT ${target}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
endforeach()


################################################################################
# Macro for ament package
################################################################################
ament_package()
