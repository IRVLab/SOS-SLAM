add_library(types_slam3d ${G2O_LIB_TYPE}
  dquat2mat.cpp dquat2mat.h
  isometry3d_mappings.h
  isometry3d_mappings.cpp
  isometry3d_gradients.cpp
  isometry3d_gradients.h
  vertex_se3.cpp
  vertex_se3.h
  edge_se3.cpp
  edge_se3.h
  se3quat.h
  se3_ops.h se3_ops.hpp
  types_slam3d.cpp
  types_slam3d.h
)
set_target_properties(types_slam3d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d)
target_link_libraries(types_slam3d core)