cmake_minimum_required(VERSION 3.8)
project(venusbluecpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

add_executable(venusbluecpp src/main.cpp src/venusblue.cpp)

ament_target_dependencies(venusbluecpp rclcpp std_msgs sensor_msgs)

target_link_libraries(
    venusbluecpp
    libpaho-mqttpp3.a
    libpaho-mqtt3as.a
    Threads::Threads
    OpenSSL::SSL
    nlohmann_json::nlohmann_json
)

install(TARGETS
    venusbluecpp
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
