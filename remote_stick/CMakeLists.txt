cmake_minimum_required(VERSION 3.8)
project(remote_stick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(INCLUDE_DEPENDS
        rclcpp
        rcpputils
        rclcpp_lifecycle
        std_msgs
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# find_package(<dependency> REQUIRED)

include_directories(
                    include
                    )


add_executable(joy_stick src/joy_stick.cpp)
ament_target_dependencies(joy_stick ${INCLUDE_DEPENDS})


install(TARGETS joy_stick
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY include/ 
    DESTINATION include/joy_stick
)

install(
  DIRECTORY launch 
  DESTINATION share/remote_stick
)


ament_package()
