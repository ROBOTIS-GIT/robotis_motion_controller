include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED)
find_dependency(osqp_vendor REQUIRED)

if(NOT TARGET OsqpEigen::OsqpEigen)
  find_dependency(OsqpEigen REQUIRED CONFIG)
endif()
