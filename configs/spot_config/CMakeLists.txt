cmake_minimum_required(VERSION 3.5)
project(spot_config)

# Find the necessary dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch maps worlds
  DESTINATION share/${PROJECT_NAME}
)

# Mark the package as ament package
ament_package()
