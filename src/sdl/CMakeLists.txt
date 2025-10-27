cmake_minimum_required(VERSION 3.5)
project(sdl)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/tamp/server/tamp.py
  scripts/tamp/server/tamp_env.py
  scripts/tamp/server/tamp_server.py
  scripts/tamp/client/tamp_client.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
