cmake_minimum_required(VERSION 3.8)
project(craft_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Importable Python modules live under src/; executable nodes live under scripts/.
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src)

install(PROGRAMS scripts/planning_master.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME planning_master)
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.stl" PATTERN "*.pgm" PATTERN "*.png" PATTERN "*.yaml" PATTERN "*.txt")
install(DIRECTORY config third_party DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_path_planner test/test_path_planner.py
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR})
endif()

ament_package()
