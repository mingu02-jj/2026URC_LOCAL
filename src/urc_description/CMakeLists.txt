cmake_minimum_required(VERSION 3.5)
project(urc_description)

# 필수 의존성
find_package(ament_cmake REQUIRED)

# === 설치할 리소스 ===

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/description)
  install(DIRECTORY description DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()


ament_package()

