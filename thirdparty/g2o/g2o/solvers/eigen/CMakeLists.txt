add_library(solver_eigen ${G2O_LIB_TYPE}
  solver_eigen.cpp
  linear_solver_eigen.h
)
set_target_properties(solver_eigen PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_eigen)
target_link_libraries(solver_eigen core)