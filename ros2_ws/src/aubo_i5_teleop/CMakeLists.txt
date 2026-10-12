cmake_minimum_required(VERSION 3.16)
project(aubo_i5_teleop)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf launch config DESTINATION share/${PROJECT_NAME})
# Helpers live next to entry scripts so source and installed imports are identical.
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  FILES_MATCHING PATTERN "*.py" PATTERN "*.sh")
ament_package()
