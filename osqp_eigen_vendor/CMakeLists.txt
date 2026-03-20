cmake_minimum_required(VERSION 3.16)
project(osqp_eigen_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)

set(OSQP_EIGEN_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/osqp-eigen")

ament_vendor(${PROJECT_NAME}
  VCS_TYPE path
  VCS_URL "${OSQP_EIGEN_SOURCE_DIR}"
)

ament_export_dependencies(Eigen3 osqp_vendor)
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-extras.cmake")

install(
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    "${OSQP_EIGEN_SOURCE_DIR}/LICENSE"
  DESTINATION share/${PROJECT_NAME}
  RENAME osqp-eigen-LICENSE
)

ament_package()
