cmake_minimum_required(VERSION 3.5)
project(nav2_urc_bringup)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)

nav2_package()

# 데이터는 그대로 share/에 설치
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY example DESTINATION share/${PROJECT_NAME})

# ✅ 실행 스크립트는 lib/<pkg>/ 에 설치 (ros2 run이 찾는 위치)
install(PROGRAMS
  example/simple_path.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  example/curved_path.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

