cmake_minimum_required(VERSION 3.5.1)
project(ament_cmake_virtualenv NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

ament_package(
  CONFIG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(roslint REQUIRED)
  # roslint_python()
  # roslint_add_test()

  # catkin_add_nosetests(test)
  # TODO: add some linters
endif()
